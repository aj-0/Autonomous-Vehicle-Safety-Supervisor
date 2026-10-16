/*
 * event_logger.c - low-priority event logger and black box.
 *
 * Reads the shared-memory buffer written by the supervisor (the supervisor never waits for
 * the disk) and writes:
 *   console / safety_log.txt   easy-to-read log with the values at every event,
 *                              plus a LIVE line with all values once per second
 *   safety_events.csv          every safety event (open in Excel)
 *   live_values.csv            100 Hz snapshot: state, speed, distance, commands
 *   incident_NNN.csv           black box: 0.5 s before + 1 s after every emergency or blocked command
 *
 * usage: event_logger [-d dir] [-q]      default dir /tmp/av_logs, -q = no console output
 */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/neutrino.h>
#include <sys/stat.h>

#include "qnx_helpers.h"
#include "event_buffer.h"
#include "settings.h"

#define BB_CYCLES      200     /* 2 s of history kept for the black box */
#define BB_PRE         50      /* 0.5 s before the incident             */
#define BB_POST        100     /* 1 s after the incident                */
#define LIVE_EVERY_MS  1000    /* LIVE values line on the console       */

static av_ring_t *g_ring;
static FILE *g_ev, *g_cyc, *g_inc;
static const char *g_dir = "/tmp/av_logs";
static int g_quiet;
static uint64_t g_t0, g_last_live_us;
static av_event_t g_bb[BB_CYCLES];
static int g_bb_n, g_bb_head, g_post_left, g_incident_no;
static uint32_t g_last_verdict_reasons;     /* repeated identical verdicts are summarised */
static uint64_t g_last_verdict_us;
static int g_verdict_repeats;

static double secs(const av_event_t *e)
{
    return (double)(e->t_us - g_t0) / 1e6;
}

static const char *kind_name(const av_event_t *e)
{
    switch (e->kind) {
    case EV_CYCLE:    return "LIVE";
    case EV_VERDICT:  return e->verdict == V_CLAMP ? "CORRECTED" : "BLOCKED";
    case EV_STATE:    return "STATE";
    case EV_COND:     return e->x1 > 0.5f ? "PROBLEM" : "SOLVED";
    case EV_INJECT:   return "FAULT";
    case EV_REACTION: return "REACTION";
    case EV_CLIENT:   return "NAVIGATION";
    case EV_OVERRUN:  return "SLOW CYCLE";
    case EV_DEADLINE: return "DEADLINE";
    default:          return "INFO";
    }
}

static const char *obstacle_str(float d, char *buf, int len)
{
    if (d < 0.0f)
        snprintf(buf, (size_t)len, "none");
    else
        snprintf(buf, (size_t)len, "%.2f m", d);
    return buf;
}

/* one-line summary of the measured values, printed under every important event */
static void values_line(const av_event_t *e, char *out, int len)
{
    char ob[24];
    snprintf(out, (size_t)len,
             "speed %.2f m/s | obstacle %s | turning %.1f deg/s | car gets %.2f m/s, steer %.1f deg",
             e->v, obstacle_str(e->dist, ob, sizeof ob), e->gyro, e->out_speed, e->out_steer);
}

/* plain-English description of an event (one line, used for console and CSV) */
static void describe(const av_event_t *e, char *out, int len)
{
    char b[300];
    switch (e->kind) {
    case EV_STATE:
        snprintf(out, (size_t)len, "%s -> %s  (%s)", av_state_name(e->prev_state),
                 av_state_name(e->state), av_state_text(e->state));
        break;
    case EV_VERDICT:
        snprintf(out, (size_t)len,
                 "nav asked %.2f m/s, steer %.1f deg  ->  car gets %.2f m/s, steer %.1f deg   "
                 "why: %s (%s)",
                 e->cmd_speed, e->cmd_steer, e->x1, e->x2, av_reason_text(e->reasons),
                 av_reasons_str(e->reasons, b, sizeof b));
        break;
    case EV_COND: {
        int pos = 0;
        out[0] = '\0';
        for (int i = 0; i < C__COUNT && pos < len - 1; i++)
            if (e->aux & (1u << i))
                pos += snprintf(out + pos, (size_t)(len - pos), "%s%s: %s", pos ? "  +  " : "",
                                av_cond_name(i), e->x1 > 0.5f ? av_cond_text(i) : "back to normal");
        break;
    }
    case EV_INJECT:
        snprintf(out, (size_t)len, "operator injected fault \"%s\"", av_fault_name((int)e->aux));
        break;
    case EV_REACTION:
        snprintf(out, (size_t)len, "supervisor reacted to \"%s\" after %.1f ms",
                 av_fault_name((int)e->aux), e->x1);
        break;
    case EV_CLIENT:
        snprintf(out, (size_t)len, "navigation program (pid %u) %s", e->aux,
                 e->x1 > 0.5f ? "connected" : "DISCONNECTED");
        break;
    case EV_OVERRUN:
        snprintf(out, (size_t)len, "one supervisor cycle took %.0f us (budget %d us)", e->x1,
                 AV_OVERRUN_US);
        break;
    case EV_DEADLINE:
        switch (e->aux) {
        case DL_START:
            snprintf(out, (size_t)len,
                     "stop started at %.2f m/s, braking %.1f m/s^2 -> must be standing still within %.0f ms",
                     e->v, e->x2, e->x1);
            break;
        case DL_PASS:
            snprintf(out, (size_t)len, "PASS: safe state reached in %.0f ms (deadline %.0f ms, %.0f ms to spare)",
                     e->x1, e->x2, e->x2 - e->x1);
            break;
        case DL_FAIL:
            snprintf(out, (size_t)len, "FAIL: safe state reached only after %.0f ms (deadline %.0f ms)",
                     e->x1, e->x2);
            break;
        default:
            snprintf(out, (size_t)len,
                     "MISSED: still moving after %.0f ms (deadline %.0f ms) -> emergency stop, motor power cut",
                     e->x1, e->x2);
            break;
        }
        break;
    case EV_CYCLE:
        snprintf(out, (size_t)len,
                 "state %s | who drives: %s | nav asks %.2f m/s, steer %.1f deg | problems: %s",
                 av_state_name(e->state), av_auth_name(e->authority), e->cmd_speed, e->cmd_steer,
                 av_conds_str(e->conds, b, sizeof b));
        break;
    default:
        out[0] = '\0';
    }
}

static void print_event(const av_event_t *e)
{
    char text[600], vals[200];
    describe(e, text, sizeof text);
    values_line(e, vals, sizeof vals);
    printf("%8.2f s  %-10s %s\n", secs(e), kind_name(e), text);
    /* show the measured values with the events where they matter */
    if (e->kind == EV_CYCLE || e->kind == EV_VERDICT || e->kind == EV_STATE ||
        e->kind == EV_DEADLINE || (e->kind == EV_COND && e->x1 > 0.5f))
        printf("%8s    %-10s %s\n", "", "", vals);
}

static void write_cycle(FILE *f, const av_event_t *e)
{
    char b[300];
    fprintf(f, "%.2f,%s,%s,%.3f,%.3f,%.2f,%.3f,%.1f,%.3f,%.1f,%s\n", secs(e) * 1000.0,
            av_state_name(e->state), av_auth_name(e->authority), e->v, e->dist, e->gyro,
            e->cmd_speed, e->cmd_steer, e->out_speed, e->out_steer,
            av_conds_str(e->conds, b, sizeof b));
}

#define CYCLE_HEADER "time_ms,state,who_drives,speed_mps,obstacle_m(-1=none),turn_rate_dps," \
                     "nav_speed_mps,nav_steer_deg,car_speed_mps,car_steer_deg,problems"

static void open_incident(const av_event_t *trigger)
{
    char path[256], text[600];
    snprintf(path, sizeof path, "%s/incident_%03d.csv", g_dir, ++g_incident_no);
    g_inc = fopen(path, "w");
    if (!g_inc)
        return;
    describe(trigger, text, sizeof text);
    fprintf(g_inc, "# incident %d at %.2f s: %s %s\n", g_incident_no, secs(trigger),
            kind_name(trigger), text);
    fprintf(g_inc, "%s\n", CYCLE_HEADER);
    int n = g_bb_n < BB_PRE ? g_bb_n : BB_PRE;
    for (int i = n; i > 0; i--)
        write_cycle(g_inc, &g_bb[(g_bb_head - i + BB_CYCLES) % BB_CYCLES]);
    g_post_left = BB_POST;
    if (!g_quiet)
        printf("%8.2f s  %-10s saved %s (0.5 s before + 1 s after)\n", secs(trigger), "BLACK BOX",
               path);
}

static void handle(const av_event_t *e)
{
    if (!g_t0)
        g_t0 = e->t_us;

    if (e->kind == EV_CYCLE) {
        write_cycle(g_cyc, e);
        g_bb[g_bb_head] = *e;
        g_bb_head = (g_bb_head + 1) % BB_CYCLES;
        if (g_bb_n < BB_CYCLES)
            g_bb_n++;
        if (g_inc) {
            write_cycle(g_inc, e);
            if (--g_post_left <= 0) {
                fclose(g_inc);
                g_inc = NULL;
            }
        }
        if (!g_quiet && e->t_us - g_last_live_us >= (uint64_t)LIVE_EVERY_MS * 1000u) {
            g_last_live_us = e->t_us;
            print_event(e);
        }
        return;
    }

    char text[600], b[300];
    describe(e, text, sizeof text);
    fprintf(g_ev, "%.2f,%s,%s,%s,%.3f,%.3f,%.2f,%.3f,%.1f,%.3f,%.1f,%s,\"%s\"\n", secs(e),
            kind_name(e), av_state_name(e->state), av_auth_name(e->authority), e->v, e->dist,
            e->gyro, e->cmd_speed, e->cmd_steer, e->out_speed, e->out_steer,
            av_conds_str(e->conds, b, sizeof b), text);
    if (!g_quiet) {
        int repeat = e->kind == EV_VERDICT && e->reasons == g_last_verdict_reasons &&
                     e->t_us - g_last_verdict_us < 1000000u;
        if (repeat) {
            g_verdict_repeats++;            /* same verdict again within 1 s: count, do not print */
        } else {
            if (e->kind == EV_VERDICT) {
                if (g_verdict_repeats)
                    printf("%8s    %-10s (previous verdict repeated %d more times)\n", "", "",
                           g_verdict_repeats);
                g_verdict_repeats = 0;
                g_last_verdict_reasons = e->reasons;
                g_last_verdict_us = e->t_us;
            }
            print_event(e);
        }
    }

    int trigger = (e->kind == EV_STATE && (e->state == ST_MRM || e->state == ST_ESTOP)) ||
                  (e->kind == EV_VERDICT && e->verdict == V_REJECT) ||
                  (e->kind == EV_DEADLINE && e->aux == DL_MISSED);
    if (trigger && !g_inc)
        open_incident(e);
}

static void drain(void)
{
    av_event_t e;
    int n = 0;
    while (ring_pop(g_ring, &e)) {
        handle(&e);
        n++;
    }
    if (n) {
        fflush(g_ev);
        fflush(g_cyc);
        if (g_inc)
            fflush(g_inc);
        fflush(stdout);
    }
}

static FILE *open_csv(const char *name, const char *header)
{
    char path[256];
    snprintf(path, sizeof path, "%s/%s", g_dir, name);
    FILE *f = fopen(path, "w");
    if (!f) {
        fprintf(stderr, "[LOGGER] cannot open %s: %s\n", path, strerror(errno));
        exit(1);
    }
    fprintf(f, "%s\n", header);
    return f;
}

int main(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "d:qh")) != -1) {
        switch (opt) {
        case 'd': g_dir = optarg; break;
        case 'q': g_quiet = 1; break;
        default:
            fprintf(stderr, "usage: %s [-d dir] [-q]\n", argv[0]);
            return 1;
        }
    }
    qnx_set_priority(AV_PRIO_LOGGER);
    mkdir(g_dir, 0777);
    g_ev = open_csv("safety_events.csv",
                    "time_s,event,state,who_drives,speed_mps,obstacle_m(-1=none),turn_rate_dps,"
                    "nav_speed_mps,nav_steer_deg,car_speed_mps,car_steer_deg,problems,description");
    g_cyc = open_csv("live_values.csv", CYCLE_HEADER);

    g_ring = ring_open();
    if (!g_ring)
        return 1;
    qnx_server_t srv;
    if (qnx_server_create(&srv, AV_NAME_LOGGER, 0) == -1)
        return 1;
    qnx_pulse_timer(srv.chid, AV_PRIO_LOGGER, AV_PULSE_TICK, AV_LOGGER_POLL_MS * 1000u, NULL);

    printf("======================================================================================\n");
    printf(" AV SAFETY SUPERVISOR - SAFETY LOG        files in %s\n", g_dir);
    printf("======================================================================================\n");
    printf(" time       event      what happened  (second line = measured values at that moment)\n");
    printf("   LIVE       = all values once per second      FAULT     = fault injected by operator\n");
    printf("   PROBLEM    = supervisor detected a problem   SOLVED    = problem gone\n");
    printf("   CORRECTED  = nav command made safer (CLAMP)  BLOCKED   = nav command refused (REJECT)\n");
    printf("   STATE      = safety level changed            REACTION  = time from fault to reaction\n");
    printf("   DEADLINE   = stop must end within a time limit: started / PASS / FAIL / MISSED\n");
    printf("--------------------------------------------------------------------------------------\n");
    fflush(stdout);

    for (;;) {
        union {
            uint16_t      type;
            struct _pulse pulse;
            char          raw[256];
        } m;
        int rcvid = MsgReceive(srv.chid, &m, sizeof m, NULL);
        if (rcvid == -1)
            continue;
        if (rcvid > 0) {
            if (!qnx_handle_system_msg(rcvid, &m))
                MsgError(rcvid, ENOSYS);
            continue;
        }
        if (m.pulse.code == _PULSE_CODE_DISCONNECT)
            ConnectDetach(m.pulse.scoid);
        else
            drain();
    }
    return 0;
}
