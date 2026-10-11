/*
 * supervisor_main.c - AV Safety Supervisor Safety Supervisor (QNX process).
 *
 * One thread, SCHED_FIFO priority 50, memory locked, channel with fixed priority.
 * Everything enters through ONE channel and is handled in arrival order:
 *
 *   messages : HELLO, NAV_CMD (nav waits for our verdict), SENSOR_STATE, PERCEPTION,
 *              RECOVERY_CMD, CLI
 *   pulses   : 10 ms TICK timer, NAV heartbeat, kernel DISCONNECT (a client process died)
 *
 * usage: safety_supervisor [-s robot|car] [-b] [-c core] [-v]
 *   -s  vehicle profile (default robot)
 *   -b  bench mode: car on a stand, yaw voting off
 *   -c  pin the supervisor to one CPU core (Raspberry Pi 5 has 4)
 *   -v  verbose: print every event and GPIO change
 */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/neutrino.h>

#include "qnx_helpers.h"
#include "event_buffer.h"
#include "access_control.h"
#include "settings.h"
#include "decision_core.h"
#include "send_output.h"
#include "leds_buzzer.h"

typedef union {
    uint16_t           type;
    struct _pulse      pulse;
    av_hdr_t          hdr;
    av_hello_t        hello;
    av_nav_cmd_t      nav;
    av_sensor_state_t sens;
    av_perception_t   perc;
    av_recovery_cmd_t recov;
    av_cli_req_t      cli;
} sup_msg_t;

static sup_ctx_t    g_ctx;
static access_roles_t  g_roles;
static qnx_server_t  g_srv;
static sup_output_t g_out;
static av_ring_t  *g_ring;
static int          g_verbose;

/* ------------------------------------------------------------------ */

static int too_short(const struct _msg_info *info, size_t need)
{
    return info->srcmsglen < 0 || (size_t)info->srcmsglen < need;
}

static void on_event(void *user, const av_event_t *ev)
{
    (void)user;
    if (g_ring)
        ring_push(g_ring, ev);
    if (!g_verbose || ev->kind == EV_CYCLE)
        return;

    char buf[256];
    switch (ev->kind) {
    case EV_STATE:
        printf("[SUPERVISOR] STATE %s -> %s\n", av_state_name(ev->prev_state), av_state_name(ev->state));
        break;
    case EV_VERDICT:
        printf("[SUPERVISOR] %s %s  cmd=%.2f/%.1f  out=%.2f/%.1f\n", av_verdict_name(ev->verdict),
               av_reasons_str(ev->reasons, buf, sizeof buf), ev->cmd_speed, ev->cmd_steer,
               ev->x1, ev->x2);
        break;
    case EV_COND:
        printf("[SUPERVISOR] %s %s\n", ev->x1 > 0.5f ? "PROBLEM" : "SOLVED ", av_conds_str(ev->aux, buf, sizeof buf));
        break;
    case EV_REACTION:
        printf("[SUPERVISOR] reaction to '%s' after %.1f ms\n", av_fault_name((int)ev->aux), ev->x1);
        break;
    case EV_CLIENT:
        printf("[SUPERVISOR] nav %s (pid %u)\n", ev->x1 > 0.5f ? "connected" : "DISCONNECTED", ev->aux);
        break;
    default:
        break;
    }
}

static void update_gpio(void)
{
    av_state_t s = g_ctx.state;
    int blink = (g_ctx.stats.ticks / 25) & 1;              /* 2 Hz */
    int green = s == ST_NORMAL ||
                ((s == ST_CAUTION || s == ST_DEGRADED || s == ST_RECOVERY) && blink);
    int red = s == ST_MRM || s == ST_ESTOP || s == ST_SAFE;
    gpio_set(AV_GPIO_GREEN, green);
    gpio_set(AV_GPIO_RED, red);
    gpio_set(AV_GPIO_BUZZER, s == ST_MRM || s == ST_ESTOP);
    gpio_set(AV_GPIO_SAFE_REQ, red);
}

static void send_output(uint64_t now, uint64_t t_in_cycles)
{
    g_ctx.out.seq++;
    g_ctx.out.t_us = now;
    int ok = out_send_vehicle(&g_out, &g_ctx.out);
    sup_core_on_output_result(&g_ctx, ok);
    update_gpio();
    if (t_in_cycles)
        stats_out_latency(&g_ctx.stats, (float)qnx_cycles_to_us(qnx_cycles() - t_in_cycles));
}

static void do_actions(uint64_t now, uint64_t t_in_cycles)
{
    uint32_t a = g_ctx.actions;
    g_ctx.actions = 0;
    if (a & ACT_OUTPUT_NOW)
        send_output(now, t_in_cycles);                     /* fast path: do not wait a tick */
    if (a & ACT_RECOV_START)
        out_pulse_recovery(&g_out, AV_PULSE_RECOV_START, (int)(g_ctx.mrm_v0 * 1000.0f));
    if (a & ACT_RECOV_STOP)
        out_pulse_recovery(&g_out, AV_PULSE_RECOV_STOP, 0);
    if (a & ACT_RESTART_NAV)
        out_pulse_recovery(&g_out, AV_PULSE_RESTART_NAV, g_ctx.nav_pid);
    if (a & ACT_LOG_KICK)
        out_pulse_logger(&g_out);
}

static void on_tick(uint64_t now)
{
    static uint64_t last_tick_cyc;
    uint64_t t0 = qnx_cycles();
    if (last_tick_cyc)
        stats_tick_interval(&g_ctx.stats, (float)qnx_cycles_to_us(t0 - last_tick_cyc),
                            AV_TICK_MS * 1000.0f);
    last_tick_cyc = t0;

    out_reconnect(&g_out, now);
    sup_core_tick(&g_ctx, now);
    send_output(now, 0);
    do_actions(now, 0);

    float cycle_us = (float)qnx_cycles_to_us(qnx_cycles() - t0);
    int overrun = cycle_us > AV_OVERRUN_US;
    stats_cycle(&g_ctx.stats, cycle_us, overrun);
    if (overrun)
        sup_emit(&g_ctx, EV_OVERRUN, (uint8_t)g_ctx.state, 0, cycle_us, 0.0f);
    if ((g_ctx.stats.ticks % 10) == 0)
        out_pulse_logger(&g_out);                          /* batch cycle records, 10 Hz */
}

static void on_pulse(const struct _pulse *p, uint64_t now)
{
    switch (p->code) {
    case AV_PULSE_TICK:
        on_tick(now);
        break;

    case AV_PULSE_NAV_HB:
        if (access_has_role(&g_roles, p->scoid, AV_ROLE_NAV))
            sup_core_on_nav_heartbeat(&g_ctx, now);
        break;

    case _PULSE_CODE_DISCONNECT: {
        /* The kernel tells us a client process closed its connection or died. */
        int32_t pid = 0;
        char name[16] = "";
        uint32_t roles = access_drop(&g_roles, p->scoid, &pid, name, sizeof name);
        ConnectDetach(p->scoid);
        if (roles & AV_ROLE_NAV)
            sup_core_on_nav_disconnect(&g_ctx, now);
        if (roles)
            printf("[SUPERVISOR] client '%s' (pid %d) disconnected\n", name, pid);
        do_actions(now, 0);
        break;
    }

    default:
        break;
    }
}

static void on_message(int rcvid, sup_msg_t *m, const struct _msg_info *info, uint64_t now,
                       uint64_t t_in)
{
    switch (m->hdr.type) {
    case AV_MSG_HELLO: {
        if (too_short(info, sizeof m->hello)) {
            MsgError(rcvid, EBADMSG);
            return;
        }
        m->hello.name[sizeof m->hello.name - 1] = '\0';
        int err = access_hello(&g_roles, info->scoid, info->pid, m->hello.roles, m->hello.name);
        if (err) {
            MsgError(rcvid, err);
            printf("[SUPERVISOR] HELLO from '%s' (pid %d) refused: %s\n", m->hello.name, info->pid,
                   strerror(err));
            return;
        }
        MsgReply(rcvid, EOK, NULL, 0);
        printf("[SUPERVISOR] client '%s' (pid %d) roles 0x%x\n", m->hello.name, info->pid, m->hello.roles);
        if (m->hello.roles & AV_ROLE_NAV)
            sup_core_on_nav_connect(&g_ctx, info->pid, now);
        break;
    }

    case AV_MSG_NAV_CMD: {
        if (!access_has_role(&g_roles, info->scoid, AV_ROLE_NAV)) {
            MsgError(rcvid, EPERM);                        /* the gate: only the nav process */
            return;
        }
        if (too_short(info, sizeof m->nav)) {
            MsgError(rcvid, EBADMSG);
            return;
        }
        av_verdict_t v;
        sup_core_on_nav_cmd(&g_ctx, &m->nav, now, &v);
        MsgReply(rcvid, EOK, &v, sizeof v);
        stats_decision(&g_ctx.stats, (float)qnx_cycles_to_us(qnx_cycles() - t_in));
        break;
    }

    case AV_MSG_SENSOR_STATE:
        if (!access_has_role(&g_roles, info->scoid, AV_ROLE_SENSOR)) {
            MsgError(rcvid, EPERM);
            return;
        }
        if (too_short(info, sizeof m->sens)) {
            MsgError(rcvid, EBADMSG);
            return;
        }
        MsgReply(rcvid, EOK, NULL, 0);                     /* release the sender first */
        sup_core_on_sensor(&g_ctx, &m->sens, now);
        break;

    case AV_MSG_PERCEPTION:
        if (!access_has_role(&g_roles, info->scoid, AV_ROLE_PERCEPTION)) {
            MsgError(rcvid, EPERM);
            return;
        }
        if (too_short(info, sizeof m->perc)) {
            MsgError(rcvid, EBADMSG);
            return;
        }
        MsgReply(rcvid, EOK, NULL, 0);
        sup_core_on_perception(&g_ctx, &m->perc, now);
        break;

    case AV_MSG_RECOVERY_CMD: {
        if (!access_has_role(&g_roles, info->scoid, AV_ROLE_RECOVERY)) {
            MsgError(rcvid, EPERM);
            return;
        }
        if (too_short(info, sizeof m->recov)) {
            MsgError(rcvid, EBADMSG);
            return;
        }
        av_verdict_t v;
        sup_core_on_recovery_cmd(&g_ctx, &m->recov, now, &v);
        MsgReply(rcvid, EOK, &v, sizeof v);
        break;
    }

    case AV_MSG_CLI: {
        av_status_t st;
        int ack = 0;
        if (m->cli.cmd == CLI_ACK)
            ack = sup_core_on_ack(&g_ctx, now) ? 1 : 2;
        else if (m->cli.cmd == CLI_INJECT_MARK)
            sup_core_on_inject_mark(&g_ctx, m->cli.arg, now);
        else if (m->cli.cmd == CLI_RESET_STATS)
            stats_init(&g_ctx.stats);
        sup_core_fill_status(&g_ctx, &st);
        st.ack_result = (uint8_t)ack;
        MsgReply(rcvid, EOK, &st, sizeof st);
        break;
    }

    default:
        MsgError(rcvid, ENOSYS);
        return;
    }
    do_actions(now, t_in);
}

int main(int argc, char *argv[])
{
    const char *profile = "robot";
    int bench = 0, core = -1, opt;

    while ((opt = getopt(argc, argv, "s:bc:vh")) != -1) {
        switch (opt) {
        case 's': profile = optarg; break;
        case 'b': bench = 1; break;
        case 'c': core = atoi(optarg); break;
        case 'v': g_verbose = 1; break;
        default:
            fprintf(stderr, "usage: %s [-s robot|car] [-b] [-c core] [-v]\n", argv[0]);
            return 1;
        }
    }

    const av_profile_t *P = av_profile_get(profile);
    qnx_set_priority(AV_PRIO_SUPERVISOR);
    qnx_lock_memory();
    qnx_pin_to_core(core);

    g_ring = ring_open();
    access_init(&g_roles);
    out_init(&g_out);
    gpio_init(g_verbose);
    sup_core_init(&g_ctx, P, bench, on_event, NULL, qnx_now_us());

    if (qnx_server_create(&g_srv, AV_NAME_SUPERVISOR, 1) == -1)
        return 1;
    if (qnx_pulse_timer(g_srv.chid, AV_PRIO_SUPERVISOR, AV_PULSE_TICK, AV_TICK_MS * 1000u, NULL) == -1)
        return 1;

    printf("[SUPERVISOR] AV Safety Supervisor started  profile=%s  prio=%d  fixed-priority channel=%s%s\n",
           P->name, AV_PRIO_SUPERVISOR, g_srv.fixed_priority ? "yes" : "NO",
           bench ? "  BENCH MODE" : "");

    for (;;) {
        sup_msg_t msg;
        struct _msg_info info;
        int rcvid = MsgReceive(g_srv.chid, &msg, sizeof msg, &info);
        uint64_t t_in = qnx_cycles();
        uint64_t now = qnx_now_us();

        if (rcvid == -1) {
            if (errno != EINTR)
                perror("[SUPERVISOR] MsgReceive");
            continue;
        }
        if (rcvid == 0) {
            on_pulse(&msg.pulse, now);
            continue;
        }
        if (qnx_handle_system_msg(rcvid, &msg))
            continue;
        on_message(rcvid, &msg, &info, now, t_in);
    }
    return 0;
}
