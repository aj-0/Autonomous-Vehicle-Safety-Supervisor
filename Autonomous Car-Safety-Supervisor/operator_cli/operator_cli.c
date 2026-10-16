/*
 * operator_cli.c - operator console (the mandatory CLI of the problem statement).
 *
 *   operator_cli                 interactive prompt
 *   operator_cli <command ...>   run one command and exit
 *
 * Commands
 *   status | watch [sec] | ack | start | stop | reset-stats
 *   inject <fault>      slew envelope speedest hang crash corrupt
 *                       flicker miss phantom cam-dead cam-frozen cam-lag
 *                       imu-frozen imu-dead enc-dead us-dead steer-stuck brake-fail link-drop
 *   clear               remove all injected faults
 *   obstacle <m> [known|unknown] | obstacle off
 *   estop on|off | sim reset | sim status
 *   rogue               prove the gate: an unregistered process tries to drive the car
 */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>

#include "qnx_helpers.h"
#include "settings.h"

static int g_sup = -1, g_nav = -1, g_link = -1;

static int conn(int *coid, const char *name)
{
    if (*coid == -1)
        *coid = name_open(name, 0);
    return *coid;
}

static int send_to(int *coid, const char *name, const void *m, size_t len, void *r, size_t rlen)
{
    if (conn(coid, name) == -1) {
        printf("  %s is not running\n", name);
        return -1;
    }
    if (qnx_send_timeout(*coid, m, len, r, rlen, 500000) == -1) {
        printf("  %s: %s\n", name, strerror(errno));
        name_close(*coid);
        *coid = -1;
        return -1;
    }
    return 0;
}

static int sup_req(uint32_t cmd, uint32_t arg, av_status_t *st)
{
    av_cli_req_t r = { { AV_MSG_CLI, AV_PROTO_VERSION }, cmd, arg };
    return send_to(&g_sup, AV_NAME_SUPERVISOR, &r, sizeof r, st, sizeof *st);
}

static int nav_ctrl(uint32_t cmd, uint32_t fault)
{
    av_nav_ctrl_t r = { { AV_MSG_NAV_CTRL, AV_PROTO_VERSION }, cmd, fault };
    return send_to(&g_nav, AV_NAME_NAV, &r, sizeof r, NULL, 0);
}

static int sim_ctrl(uint32_t cmd, uint32_t fault, float value, int flag, av_sim_status_t *st)
{
    av_sim_ctrl_t r = { { AV_MSG_SIM_CTRL, AV_PROTO_VERSION }, cmd, fault, value, flag };
    av_sim_status_t tmp;
    return send_to(&g_link, AV_NAME_LINK, &r, sizeof r, st ? st : &tmp, sizeof tmp);
}

static const char *health_word(int h)
{
    return h == H_OK ? "OK" : av_health_name(h);
}

static void print_status(const av_status_t *s)
{
    char b[400];
    printf("================ AV SAFETY SUPERVISOR - STATUS ================\n");
    printf("Profile          : %s%s      running for %.1f s\n", s->profile,
           s->bench ? " (bench)" : "", s->uptime_ms / 1000.0);
    printf("Safety state     : %s  (%s)\n", av_state_name(s->state), av_state_text(s->state));
    printf("Who drives       : %s\n", s->authority == AUTH_NAV ? "NAVIGATION" :
           s->authority == AUTH_RECOVERY ? "RECOVERY MANAGER (safe stop)" : "NOBODY (car held stopped)");
    printf("Active problems  : %s\n", av_conds_str(s->conds, b, sizeof b));
    for (int i = 0; i < C__COUNT; i++)
        if (s->conds & (1u << i))
            printf("                   - %s\n", av_cond_text(i));
    printf("--- measured values ---\n");
    printf("Real speed       : %.2f m/s        (wheel sensors)\n", s->v_true);
    if (s->obstacle_in_range)
        printf("Obstacle ahead   : %.2f m          (ultrasonic)\n", s->dist_m);
    else
        printf("Obstacle ahead   : none            (ultrasonic, nothing within %.1f m)\n", s->dist_m);
    printf("Turning rate     : %.1f deg/s       (IMU gyro)\n", s->gyro_dps);
    if (s->perc_seen)
        printf("Camera sees      : %s%s%s     (result %u ms old)%s\n",
               s->perc_obstacle ? "obstacle " : "nothing", s->perc_obstacle ? "at " : "",
               s->perc_obstacle ? av_zone_name(s->perc_zone) : "", s->perc_age_ms,
               s->perc_unstable ? "  FLICKERING" : "");
    else
        printf("Camera sees      : no data\n");
    printf("Sensor health    : left wheel %s | right wheel %s | ultrasonic %s | IMU %s\n",
           health_word(s->health[SENS_ENC_L]), health_word(s->health[SENS_ENC_R]),
           health_word(s->health[SENS_US]), health_word(s->health[SENS_IMU]));
    if (s->sensor_pid)
        printf("Sensor monitor   : running (pid %d), judges every sensor every %d ms\n",
               s->sensor_pid, AV_TICK_MS);
    else
        printf("Sensor monitor   : NOT RUNNING (no sensor data reaches the supervisor)\n");
    printf("Motor watchdog   : %s | emergency button %s\n",
           s->island_tripped ? "STOPPED THE MOTORS" : "ok", s->estop ? "PRESSED" : "not pressed");
    printf("--- command check ---\n");
    printf("Navigation asked : %.2f m/s, steer %.1f deg\n", s->nav_speed, s->nav_steer);
    printf("Car really gets  : %.2f m/s, steer %.1f deg\n", s->out_speed, s->out_steer);
    printf("Last decision    : %s%s%s\n",
           s->last_verdict == V_PASS ? "PASS (allowed)" :
           s->last_verdict == V_CLAMP ? "CLAMP (made safer)" : "REJECT (blocked)",
           s->last_verdict == V_PASS ? "" : " - ", s->last_verdict == V_PASS ? "" : av_reason_text(s->last_reasons));
    printf("Decisions so far : %u allowed | %u made safer | %u blocked\n",
           s->pass_count, s->clamp_count, s->reject_count);
    printf("Navigation prog  : %s (pid %d), last heartbeat %u ms ago\n",
           s->nav_connected ? "running" : "NOT RUNNING", s->nav_pid, s->nav_hb_age_ms);
    printf("--- QNX real-time timing ---\n");
    printf("Supervisor cycle : average %.1f us, worst %.1f us   (runs every %d ms)\n",
           s->cycle_us_avg, s->cycle_us_max, AV_TICK_MS);
    printf("Decision time    : worst %.1f us per command\n", s->decision_us_max);
    printf("Timer accuracy   : worst delay %.2f ms | slow cycles %u | failed sends %u\n",
           s->jitter_us_max / 1000.0, s->overruns, s->output_fails);
    if (s->inject_count)
        printf("Reaction time    : last fault '%s' -> %.1f ms   (best %.1f / worst %.1f ms, %u faults)\n",
               av_fault_name((int)s->last_inject_fault), s->inject_ms_last, s->inject_ms_min,
               s->inject_ms_max, s->inject_count);
    printf("--- safe-state deadline ---\n");
    if (s->dl_active)
        printf("Stop in progress : %.0f ms used of %.0f ms allowed\n", s->dl_elapsed_ms, s->dl_budget_ms);
    if (s->dl_pass + s->dl_fail)
        printf("Stops so far     : %u in time (PASS) | %u too late (FAIL) | last %.0f ms of %.0f ms | "
               "slowest %.0f ms | smallest spare %.0f ms\n",
               s->dl_pass, s->dl_fail, s->dl_elapsed_ms, s->dl_budget_ms, s->dl_worst_ms,
               s->dl_min_margin_ms);
    else if (!s->dl_active)
        printf("Stops so far     : none yet\n");
    printf("===============================================================\n");
}

static void watch(int seconds)
{
    av_sim_status_t sim;
    av_status_t s;
    char b[200], ob[16];
    printf("   time  state     who drives  real speed  obstacle  nav asked      car gets       problems\n");
    for (int i = 0; i < seconds * 5; i++) {
        if (sup_req(CLI_STATUS, 0, &s) == -1)
            return;
        int have_sim = sim_ctrl(SIM_CTRL_STATUS, 0, 0, 0, &sim) == 0;
        if (s.obstacle_in_range)
            snprintf(ob, sizeof ob, "%.2f m", s.dist_m);
        else
            snprintf(ob, sizeof ob, "none");
        printf("%6.1fs  %-9s %-10s  %.2f m/s    %-8s  %.2f m/s %5.1f  %.2f m/s %5.1f  %s%s\n",
               s.uptime_ms / 1000.0, av_state_name(s.state),
               s.authority == AUTH_NAV ? "NAV" : s.authority == AUTH_RECOVERY ? "RECOVERY" : "NOBODY",
               s.v_true, ob, s.nav_speed, s.nav_steer, s.out_speed, s.out_steer,
               av_conds_str(s.conds, b, sizeof b),
               have_sim && sim.collided ? "  *** COLLIDED ***" : "");
        fflush(stdout);
        usleep(200000);
    }
}

static int is_nav_fault(int f)
{
    return f >= FAULT_NAV_SLEW && f <= FAULT_NAV_CORRUPT;
}

static void rogue(void)
{
    printf("rogue test: a process WITHOUT the NAV role sends a full-speed command to the supervisor\n");
    int c = name_open(AV_NAME_SUPERVISOR, 0);
    if (c != -1) {
        av_nav_cmd_t cmd;
        memset(&cmd, 0, sizeof cmd);
        cmd.hdr.type = AV_MSG_NAV_CMD;
        cmd.target_speed = 99.0f;
        av_verdict_t v;
        if (qnx_send_timeout(c, &cmd, sizeof cmd, &v, sizeof v, 200000) == -1)
            printf("  -> refused by the supervisor: %s  (gate works)\n", strerror(errno));
        else
            printf("  -> ACCEPTED (gate broken!)\n");
        name_close(c);
    }
    printf("rogue test: the same process tries to drive the vehicle link directly\n");
    c = name_open(AV_NAME_LINK, 0);
    if (c != -1) {
        av_vehicle_cmd_t vc;
        memset(&vc, 0, sizeof vc);
        vc.hdr.type = AV_MSG_VEHICLE_CMD;
        vc.speed_sp = 99.0f;
        vc.motor_enable = 1;
        if (qnx_send_timeout(c, &vc, sizeof vc, NULL, 0, 200000) == -1)
            printf("  -> refused by the vehicle link: %s  (only the supervisor may drive)\n", strerror(errno));
        else
            printf("  -> ACCEPTED (gate broken!)\n");
        name_close(c);
    }
}

static void help(void)
{
    printf("commands:\n"
           "  status | watch [sec] | ack | start | stop | reset-stats\n"
           "  inject <fault>   nav : slew envelope speedest hang crash corrupt\n"
           "                   cam : flicker miss phantom cam-dead cam-frozen cam-lag\n"
           "                   veh : imu-frozen imu-dead enc-dead us-dead steer-stuck brake-fail link-drop\n"
           "  clear | obstacle <m> [known|unknown] | obstacle off | estop on|off\n"
           "  sim reset | sim status | rogue | help | quit\n");
}

static void run(int argc, char **argv)
{
    av_status_t st;
    av_sim_status_t sim;
    const char *cmd = argv[0];

    if (!strcmp(cmd, "status")) {
        if (sup_req(CLI_STATUS, 0, &st) == 0)
            print_status(&st);
    } else if (!strcmp(cmd, "watch")) {
        watch(argc > 1 ? atoi(argv[1]) : 10);
    } else if (!strcmp(cmd, "ack")) {
        if (sup_req(CLI_ACK, 0, &st) == 0)
            printf("  ack %s (state %s)\n", st.ack_result == 1 ? "ACCEPTED" : "not possible now",
                   av_state_name(st.state));
    } else if (!strcmp(cmd, "start")) {
        nav_ctrl(NAV_CTRL_START, 0);
    } else if (!strcmp(cmd, "stop")) {
        nav_ctrl(NAV_CTRL_STOP, 0);
    } else if (!strcmp(cmd, "reset-stats")) {
        sup_req(CLI_RESET_STATS, 0, &st);
    } else if (!strcmp(cmd, "inject") && argc > 1) {
        int f = av_fault_from_name(argv[1]);
        if (f == FAULT_NONE) {
            printf("  unknown fault '%s'\n", argv[1]);
            return;
        }
        sup_req(CLI_INJECT_MARK, (uint32_t)f, &st);          /* start the reaction stopwatch */
        if (is_nav_fault(f))
            nav_ctrl(NAV_CTRL_FAULT, (uint32_t)f);
        else
            sim_ctrl(SIM_CTRL_FAULT, (uint32_t)f, 0, 0, NULL);
        printf("  injected '%s'\n", argv[1]);
    } else if (!strcmp(cmd, "clear")) {
        nav_ctrl(NAV_CTRL_CLEAR, 0);
        sim_ctrl(SIM_CTRL_CLEAR, 0, 0, 0, NULL);
    } else if (!strcmp(cmd, "obstacle") && argc > 1) {
        if (!strcmp(argv[1], "off"))
            sim_ctrl(SIM_CTRL_OBSTACLE_OFF, 0, 0, 0, NULL);
        else
            sim_ctrl(SIM_CTRL_OBSTACLE, 0, (float)atof(argv[1]),
                     !(argc > 2 && !strcmp(argv[2], "unknown")), NULL);
    } else if (!strcmp(cmd, "estop") && argc > 1) {
        sim_ctrl(SIM_CTRL_ESTOP, 0, 0, !strcmp(argv[1], "on"), NULL);
    } else if (!strcmp(cmd, "sim") && argc > 1) {
        if (!strcmp(argv[1], "reset"))
            sim_ctrl(SIM_CTRL_RESET, 0, 0, 0, NULL);
        else if (sim_ctrl(SIM_CTRL_STATUS, 0, 0, 0, &sim) == 0)
            printf("  sim: x=%.2f y=%.2f v=%.2f dist=%.2f obstacle=%s at %.2f (%s) faults=0x%x%s%s%s\n",
                   sim.x, sim.y, sim.v, sim.dist, sim.obs_present ? "yes" : "no", sim.obs_x,
                   sim.obs_known ? "known" : "unknown", sim.faults, sim.collided ? " COLLIDED" : "",
                   sim.island_tripped ? " ISLAND-TRIPPED" : "", sim.motor_on ? " motor-on" : "");
    } else if (!strcmp(cmd, "rogue")) {
        rogue();
    } else {
        help();
    }
}

int main(int argc, char *argv[])
{
    if (argc > 1) {
        run(argc - 1, argv + 1);
        return 0;
    }
    help();
    char line[256];
    for (;;) {
        printf("safety> ");
        fflush(stdout);
        if (!fgets(line, sizeof line, stdin))
            break;
        char *av[8];
        int ac = 0;
        for (char *t = strtok(line, " \t\r\n"); t && ac < 8; t = strtok(NULL, " \t\r\n"))
            av[ac++] = t;
        if (ac == 0)
            continue;
        if (!strcmp(av[0], "quit") || !strcmp(av[0], "exit"))
            break;
        run(ac, av);
    }
    return 0;
}
