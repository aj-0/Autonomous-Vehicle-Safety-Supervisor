/*
 * navigation.c - stand-in for the autonomous navigation task (the untrusted "doer").
 *
 * 20 Hz: read the latest perception, plan (cruise / swerve / brake), send a heartbeat pulse,
 * then MsgSend the command to the supervisor and stay blocked until the verdict comes back.
 * Fault injection (from the CLI) makes it misbehave the way real AI/planning software can.
 *
 * usage: navigation [-s robot|car] [-a]      -a = start driving immediately
 */
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/neutrino.h>

#include "qnx_helpers.h"
#include "settings.h"

typedef union {
    uint16_t         type;
    struct _pulse    pulse;
    av_hdr_t        hdr;
    av_perception_t perc;
    av_nav_ctrl_t   ctrl;
} nav_msg_t;

static const av_profile_t *P;
static int      g_sup = -1;
static int      g_running;
static uint32_t g_fault, g_seq;
static int      g_env_cycles;
static float    g_steer, g_env_steer;
static av_perception_t g_perc;
static uint64_t g_perc_rx;
static av_verdict_t g_last = { .verdict = 255 };

static float brake_distance(float v)
{
    return v * 0.1f + v * v / (2.0f * P->a_comf) * 1.5f + 2.0f * P->margin_m;
}

static void connect_supervisor(void)
{
    for (;;) {
        g_sup = qnx_name_open_retry(AV_NAME_SUPERVISOR, 1, 0);
        if (g_sup != -1) {
            int err = qnx_hello(g_sup, AV_ROLE_NAV, "navigation");
            if (err == 0)
                break;
            fprintf(stderr, "[NAVIGATION] supervisor refused NAV role: %s, retrying\n", strerror(err));
            name_close(g_sup);
            g_sup = -1;
        }
        usleep(200000);
    }
    printf("[NAVIGATION] connected to supervisor as NAV (pid %d)\n", getpid());
}

static void plan_and_send(void)
{
    uint64_t now = qnx_now_us();

    if (g_fault == FAULT_NAV_HANG) {
        printf("[NAVIGATION] HANG injected: spinning forever, no heartbeat, no commands\n");
        fflush(stdout);
        for (;;) {
        }
    }
    if (g_fault == FAULT_NAV_CRASH) {
        printf("[NAVIGATION] CRASH injected: abort()\n");
        fflush(stdout);
        abort();
    }

    MsgSendPulse(g_sup, -1, AV_PULSE_NAV_HB, (int)g_seq);

    int fresh = g_perc_rx && now - g_perc_rx < 200000u;
    float v_est = fresh ? g_perc.odom_speed : g_last.out_speed;
    float speed = 0.0f, target_steer = 0.0f;
    uint8_t obstacle = 0, zone = ZONE_NONE;

    if (g_running) {
        speed = P->v_cruise;
        if (fresh && g_perc.obstacle) {
            obstacle = 1;
            zone = g_perc.zone;
            if (zone == ZONE_CENTER && g_perc.cam_dist_m < brake_distance(v_est))
                speed = 0.0f;                         /* brake for the obstacle we see */
            else if (zone == ZONE_LEFT)
                target_steer = -15.0f;                /* swerve right */
            else if (zone == ZONE_RIGHT)
                target_steer = 15.0f;
        }
    }

    /* a well-behaved planner moves the steering smoothly */
    float rate = 0.8f * P->steer_rate_dps * AV_NAV_PERIOD_MS * 1e-3f;
    float d = target_steer - g_steer;
    g_steer += d > rate ? rate : (d < -rate ? -rate : d);
    float steer = g_steer;

    switch (g_fault) {
    case FAULT_NAV_SLEW:                              /* hallucination: 0 -> 30 deg at once */
        steer = 30.0f;
        g_fault = FAULT_NONE;
        break;
    case FAULT_NAV_CORRUPT:                           /* memory corruption */
        steer = 999.0f;
        g_fault = FAULT_NONE;
        break;
    case FAULT_NAV_ENVELOPE:                          /* slowly turn far too hard for the speed */
        g_env_steer += rate;
        if (g_env_steer > 30.0f)
            g_env_steer = 30.0f;
        steer = g_steer = g_env_steer;
        if (++g_env_cycles > 60) {
            g_fault = FAULT_NONE;
            g_env_steer = 0.0f;
        }
        break;
    case FAULT_NAV_SPEEDEST:
        v_est *= 0.5f;                                /* believes it is slower than it is */
        break;
    default:
        break;
    }

    av_nav_cmd_t cmd;
    memset(&cmd, 0, sizeof cmd);
    cmd.hdr.type = AV_MSG_NAV_CMD;
    cmd.hdr.version = AV_PROTO_VERSION;
    cmd.seq = ++g_seq;
    cmd.t_us = now;
    cmd.target_speed = speed;
    cmd.steer_deg = steer;
    cmd.nav_speed_est = v_est;
    cmd.nav_obstacle = obstacle;
    cmd.nav_zone = zone;

    av_verdict_t v;
    if (qnx_send_timeout(g_sup, &cmd, sizeof cmd, &v, sizeof v, 30000) == -1) {
        if (errno == EBADF || errno == ESRCH) {
            printf("[NAVIGATION] supervisor gone, reconnecting\n");
            name_close(g_sup);
            connect_supervisor();
        }
        return;
    }
    if (v.verdict != g_last.verdict || v.state != g_last.state || v.reasons != g_last.reasons) {
        char buf[128];
        printf("[NAVIGATION] I asked %.2f m/s, steer %5.1f deg -> supervisor: %-6s %-24s car gets %.2f m/s, %5.1f deg (state %s)\n", speed,
               steer, av_verdict_name(v.verdict), av_reasons_str(v.reasons, buf, sizeof buf),
               v.out_speed, v.out_steer, av_state_name(v.state));
    }
    g_last = v;
}

int main(int argc, char *argv[])
{
    const char *profile = "robot";
    int opt;
    while ((opt = getopt(argc, argv, "s:ah")) != -1) {
        switch (opt) {
        case 's': profile = optarg; break;
        case 'a': g_running = 1; break;
        default:
            fprintf(stderr, "usage: %s [-s robot|car] [-a]\n", argv[0]);
            return 1;
        }
    }
    P = av_profile_get(profile);
    qnx_set_priority(AV_PRIO_NAV);

    qnx_server_t srv;
    if (qnx_server_create(&srv, AV_NAME_NAV, 0) == -1)
        return 1;
    connect_supervisor();
    qnx_pulse_timer(srv.chid, AV_PRIO_NAV, AV_PULSE_TICK, AV_NAV_PERIOD_MS * 1000u, NULL);

    for (;;) {
        nav_msg_t m;
        struct _msg_info info;
        int rcvid = MsgReceive(srv.chid, &m, sizeof m, &info);
        if (rcvid == -1)
            continue;
        if (rcvid == 0) {
            if (m.pulse.code == AV_PULSE_TICK)
                plan_and_send();
            else if (m.pulse.code == _PULSE_CODE_DISCONNECT)
                ConnectDetach(m.pulse.scoid);
            continue;
        }
        if (qnx_handle_system_msg(rcvid, &m))
            continue;

        switch (m.hdr.type) {
        case AV_MSG_PERCEPTION:
            g_perc = m.perc;
            g_perc_rx = qnx_now_us();
            MsgReply(rcvid, EOK, NULL, 0);
            break;
        case AV_MSG_NAV_CTRL:
            switch (m.ctrl.cmd) {
            case NAV_CTRL_START: g_running = 1; printf("[NAVIGATION] START\n"); break;
            case NAV_CTRL_STOP:  g_running = 0; printf("[NAVIGATION] STOP\n"); break;
            case NAV_CTRL_FAULT:
                g_fault = m.ctrl.fault;
                g_env_cycles = 0;
                g_env_steer = g_steer;
                printf("[NAVIGATION] fault injected: %s\n", av_fault_name((int)g_fault));
                break;
            case NAV_CTRL_CLEAR: g_fault = FAULT_NONE; break;
            default: break;
            }
            MsgReply(rcvid, EOK, NULL, 0);
            break;
        default:
            MsgError(rcvid, ENOSYS);
            break;
        }
    }
    return 0;
}
