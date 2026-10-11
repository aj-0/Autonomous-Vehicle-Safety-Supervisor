/*
 * recovery_manager.c - executes the controlled stop (minimal risk manoeuvre) and restarts
 * a crashed or hung navigation process.
 *
 * Pulses from the supervisor (never MsgSend, so no send-cycle deadlock is possible):
 *   RECOV_START (value = speed mm/s) : start the deceleration ramp
 *   RECOV_STOP                        : stop sending setpoints
 *   RESTART_NAV (value = old pid)     : kill the old nav (if hung) and spawn a new one
 * While active it MsgSends RECOVERY_CMD setpoints to the supervisor every 20 ms. The
 * supervisor only accepts them while it is in MRM (authority = RECOVERY).
 *
 * usage: recovery_manager [-s robot|car] [-n /path/to/navigation]
 */
#include <errno.h>
#include <signal.h>
#include <spawn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/neutrino.h>

#include "qnx_helpers.h"
#include "settings.h"

extern char **environ;

static const av_profile_t *P;
static const char *g_nav_path = "./navigation";
static const char *g_profile = "robot";
static int      g_sup = -1;
static int      g_active;
static float    g_v0;
static uint64_t g_t0, g_last_restart;
static uint32_t g_seq;

static void connect_supervisor(void)
{
    for (;;) {
        g_sup = qnx_name_open_retry(AV_NAME_SUPERVISOR, 1, 0);
        if (g_sup != -1 && qnx_hello(g_sup, AV_ROLE_RECOVERY, "recovery") == 0)
            break;
        if (g_sup != -1)
            name_close(g_sup);
        usleep(200000);
    }
    printf("[RECOVERY] connected to supervisor as RECOVERY\n");
}

static void restart_nav(int old_pid, uint64_t now)
{
    if (now - g_last_restart < 1000000u)
        return;
    g_last_restart = now;
    if (old_pid > 0 && kill(old_pid, 0) == 0) {
        printf("[RECOVERY] killing unresponsive nav (pid %d)\n", old_pid);
        kill(old_pid, SIGKILL);
        usleep(50000);
    }
    char *argv[] = { (char *)g_nav_path, "-s", (char *)g_profile, "-a", NULL };
    pid_t pid;
    int rc = posix_spawn(&pid, g_nav_path, NULL, NULL, argv, environ);
    if (rc != 0)
        printf("[RECOVERY] cannot restart nav '%s': %s\n", g_nav_path, strerror(rc));
    else
        printf("[RECOVERY] navigation restarted (new pid %d)\n", pid);
}

static void ramp_step(uint64_t now)
{
    float t = (float)(now - g_t0) * 1e-6f;
    float v = g_v0 - P->a_comf * t;
    if (v < 0.0f)
        v = 0.0f;

    av_recovery_cmd_t cmd;
    memset(&cmd, 0, sizeof cmd);
    cmd.hdr.type = AV_MSG_RECOVERY_CMD;
    cmd.hdr.version = AV_PROTO_VERSION;
    cmd.seq = ++g_seq;
    cmd.speed = v;
    cmd.steer_deg = 0.0f;

    av_verdict_t verdict;
    if (qnx_send_timeout(g_sup, &cmd, sizeof cmd, &verdict, sizeof verdict, 5000) == -1)
        return;
    if (verdict.state != ST_MRM) {
        g_active = 0;
        printf("[RECOVERY] controlled stop finished, supervisor state %s\n", av_state_name(verdict.state));
    }
}

int main(int argc, char *argv[])
{
    int opt;
    while ((opt = getopt(argc, argv, "s:n:h")) != -1) {
        switch (opt) {
        case 's': g_profile = optarg; break;
        case 'n': g_nav_path = optarg; break;
        default:
            fprintf(stderr, "usage: %s [-s robot|car] [-n nav_path]\n", argv[0]);
            return 1;
        }
    }
    P = av_profile_get(g_profile);
    signal(SIGCHLD, SIG_IGN);                 /* children are reaped automatically */
    qnx_set_priority(AV_PRIO_RECOVERY);
    qnx_lock_memory();

    qnx_server_t srv;
    if (qnx_server_create(&srv, AV_NAME_RECOVERY, 0) == -1)
        return 1;
    connect_supervisor();
    qnx_pulse_timer(srv.chid, AV_PRIO_RECOVERY, AV_PULSE_TICK, AV_RECOV_PERIOD_MS * 1000u, NULL);
    printf("[RECOVERY] recovery manager ready (profile %s, nav=%s)\n", P->name, g_nav_path);

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
        uint64_t now = qnx_now_us();
        switch (m.pulse.code) {
        case AV_PULSE_TICK:
            if (g_active)
                ramp_step(now);
            break;
        case AV_PULSE_RECOV_START:
            g_active = 1;
            g_v0 = (float)m.pulse.value.sival_int / 1000.0f;
            g_t0 = now;
            printf("[RECOVERY] MRM: controlled stop from %.2f m/s at %.1f m/s^2\n", g_v0, P->a_comf);
            break;
        case AV_PULSE_RECOV_STOP:
            g_active = 0;
            break;
        case AV_PULSE_RESTART_NAV:
            restart_nav(m.pulse.value.sival_int, now);
            break;
        case _PULSE_CODE_DISCONNECT:
            ConnectDetach(m.pulse.scoid);
            break;
        default:
            break;
        }
    }
    return 0;
}
