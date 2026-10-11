/*
 * vehicle_simulator.c - Stage 0 stand-in for STM32 safety island + robot car + sensors + camera.
 *
 * It registers as "av_vehicle_link", the same name the real vehicle_link will use, and speaks
 * exactly the same messages. The supervisor cannot tell the simulation from the hardware.
 *
 *  server thread  : VEHICLE_CMD from the supervisor (only the SUPERVISOR role may drive),
 *                   SIM_CTRL from the CLI (faults, obstacle, reset, E-stop)
 *  physics thread : 100 Hz vehicle dynamics, sensors with noise, sensor health, island
 *                   watchdog, sends SENSOR_STATE + PERCEPTION (camera model) every cycle
 *
 * usage: vehicle_simulator [-s robot|car] [-v]
 */
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/neutrino.h>

#include "qnx_helpers.h"
#include "access_control.h"
#include "sensor_health.h"
#include "settings.h"
#include "vehicle_physics.h"

#define RAD2DEG 57.29577951308232
#define FBIT(f) (1u << (f))
#define LAG_FRAMES 9                      /* ~300 ms at 30 Hz */

enum { ISL_ARMING = 0, ISL_ARMED, ISL_TRIPPED };

typedef struct {
    pthread_mutex_t      mtx;
    const av_profile_t *P;
    int                  verbose;
    uint64_t             start_us;

    /* world */
    double x, y, psi, vl, vr, v_prev;
    int    obs_present, obs_known, collided;
    double obs_x;

    /* last command from the supervisor */
    av_vehicle_cmd_t cmd;
    uint64_t cmd_rx_us;
    int      cmd_seen;

    /* STM32 island emulation */
    int island, rearm, estop;

    /* faults */
    uint32_t faults;
    float    frozen_gyro, frozen_ax, frozen_ay;

    /* sensors */
    health_tracker_t tr[SENS_COUNT];
    float    enc_l, enc_r, dist, gyro, ax, ay;
    int      no_echo;
    uint32_t sens_seq, tick;

    /* camera */
    uint32_t perc_seq;
    int      flick;
    av_perception_t last_perc, lag[LAG_FRAMES];
    int      lag_idx;
} sim_t;

static sim_t S;
static access_roles_t g_roles;
static unsigned g_seed = 12345;

static double gauss(void)
{
    double u1 = (rand_r(&g_seed) + 1.0) / ((double)RAND_MAX + 2.0);
    double u2 = (rand_r(&g_seed) + 1.0) / ((double)RAND_MAX + 2.0);
    return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
}

static double approach(double v, double target, double up, double down, double dt)
{
    double diff = target - v;
    double lim = (fabs(target) > fabs(v) ? up : down) * dt;
    if (diff > lim)
        diff = lim;
    if (diff < -lim)
        diff = -lim;
    return v + diff;
}

static void reset_world(void)
{
    S.x = S.y = S.psi = S.vl = S.vr = S.v_prev = 0.0;
    S.collided = 0;
    S.obs_present = 0;
    S.faults = 0;
    S.estop = 0;
}

/* ------------------------------------------------------------------ */
/* physics thread                                                      */
/* ------------------------------------------------------------------ */

static void step_vehicle(uint64_t now, double dt)
{
    const av_profile_t *P = S.P;

    /* STM32 island: valid frames must keep coming */
    int fresh = S.cmd_seen && now - S.cmd_rx_us <= (uint64_t)AV_ISLAND_TIMEOUT_MS * 1000u;
    if (S.island == ISL_ARMED && S.cmd_seen && !fresh) {
        S.island = ISL_TRIPPED;
        S.rearm = 0;
        printf("[SIMULATOR] ISLAND TRIP: no valid supervisor frame for %d ms -> motors stopped, relay open\n",
               AV_ISLAND_TIMEOUT_MS);
    }
    if (!fresh)
        S.rearm = 0;
    if (S.island != ISL_ARMED && S.rearm >= AV_ISLAND_REARM_FRAMES) {
        if (S.island == ISL_TRIPPED)
            printf("[SIMULATOR] island re-armed after %d valid frames\n", AV_ISLAND_REARM_FRAMES);
        S.island = ISL_ARMED;
    }

    int powered = S.island == ISL_ARMED && fresh && S.cmd.motor_enable && !S.estop && !S.collided;
    double tl = 0.0, tr = 0.0, down;
    if (powered) {
        float l, r;
        physics_steer_to_wheels(S.cmd.speed_sp, S.cmd.steer_deg, P->wheelbase_m, P->track_m, &l, &r);
        tl = l;
        tr = r;
        if (S.faults & FBIT(FAULT_STEER_STUCK))
            tl = tr = S.cmd.speed_sp;                /* vehicle ignores steering */
        down = S.cmd.brake_mode == BRAKE_MAX ? P->a_phys_max : P->a_comf;
        if (S.faults & FBIT(FAULT_BRAKE_FAIL))
            down = 0.1 * P->a_comf;
    } else if (S.island != ISL_ARMED || S.estop) {
        down = (S.faults & FBIT(FAULT_BRAKE_FAIL)) ? 0.1 * P->a_comf : P->a_phys_max;  /* island brakes */
    } else {
        down = 0.5 * P->a_comf;                      /* relay open: coasting on friction */
    }
    S.vl = approach(S.vl, tl, P->accel_max, down, dt);
    S.vr = approach(S.vr, tr, P->accel_max, down, dt);

    double v = 0.5 * (S.vl + S.vr);
    double omega = (S.vr - S.vl) / P->track_m;
    S.psi += omega * dt;
    S.x += v * cos(S.psi) * dt;
    S.y += v * sin(S.psi) * dt;

    if (S.obs_present && !S.collided && fabs(S.y) < P->lane_half_m && S.x >= S.obs_x) {
        S.collided = 1;
        printf("[SIMULATOR] *** COLLISION at %.2f m/s ***\n", v);
        S.x = S.obs_x;
        S.vl = S.vr = 0.0;
        v = omega = 0.0;
    }

    /* ---- sensors ---- */
    int moving = fabs(v) > 2.0 * P->standstill_v;
    S.ax = (float)((v - S.v_prev) / dt);
    S.ay = (float)(v * omega);
    S.v_prev = v;

    if (!(S.faults & FBIT(FAULT_ENC_L_DEAD))) {
        S.enc_l = (float)(S.vl + P->enc_noise * gauss());
        health_sample(&S.tr[SENS_ENC_L], S.enc_l, now, moving);
    }
    S.enc_r = (float)(S.vr + P->enc_noise * gauss());
    health_sample(&S.tr[SENS_ENC_R], S.enc_r, now, moving);

    if (!(S.faults & FBIT(FAULT_IMU_DEAD))) {
        if (S.faults & FBIT(FAULT_IMU_FROZEN)) {
            S.gyro = S.frozen_gyro;
            S.ax = S.frozen_ax;
            S.ay = S.frozen_ay;
        } else {
            S.gyro = (float)(omega * RAD2DEG + P->gyro_noise_dps * gauss());
        }
        health_sample(&S.tr[SENS_IMU], S.gyro, now, moving);
    }

    if ((S.tick % AV_US_EVERY_TICKS) == 0 && !(S.faults & FBIT(FAULT_US_DEAD))) {
        double rel = S.obs_x - S.x;
        int visible = S.obs_present && rel > P->us_min_m && rel < P->us_max_m &&
                      fabs(S.y) < P->lane_half_m && fabs(S.psi) < 0.26;
        S.no_echo = !visible;
        S.dist = visible ? (float)(rel + P->us_noise * gauss()) : P->us_max_m;
        health_sample(&S.tr[SENS_US], S.dist, now, moving && visible);
    }
    for (int i = 0; i < SENS_COUNT; i++)
        health_poll(&S.tr[i], now);
}

static void make_sensor_msg(av_sensor_state_t *m, uint64_t now)
{
    memset(m, 0, sizeof *m);
    m->hdr.type = AV_MSG_SENSOR_STATE;
    m->hdr.version = AV_PROTO_VERSION;
    m->seq = ++S.sens_seq;
    m->t_us = now;
    m->v_left = S.enc_l;
    m->v_right = S.enc_r;
    m->distance_m = S.dist;
    m->gyro_z_dps = S.gyro;
    m->accel_x = S.ax;
    m->accel_y = S.ay;
    for (int i = 0; i < SENS_COUNT; i++)
        m->health[i] = S.tr[i].health;
    m->us_no_echo = (uint8_t)S.no_echo;
    m->island_state = S.island == ISL_TRIPPED;
    m->estop = (uint8_t)S.estop;
    m->relay_closed = S.island == ISL_ARMED && S.cmd.motor_enable;
}

/* Camera + AI model. Returns 0 if no packet is sent this cycle. */
static int make_perception(av_perception_t *p, uint64_t now)
{
    const av_profile_t *P = S.P;

    if (S.faults & FBIT(FAULT_CAM_DEAD))
        return 0;
    if (S.faults & FBIT(FAULT_CAM_FROZEN)) {           /* same frame again, same seq */
        *p = S.last_perc;
        return S.perc_seq != 0;
    }

    memset(p, 0, sizeof *p);
    p->hdr.type = AV_MSG_PERCEPTION;
    p->hdr.version = AV_PROTO_VERSION;
    p->seq = ++S.perc_seq;
    p->t_capture_us = now;
    p->fps = 1000.0f / (AV_TICK_MS * AV_PERC_EVERY_TICKS);
    p->odom_speed = (float)(0.5 * (S.vl + S.vr) + P->enc_noise * gauss());

    double rel = S.obs_x - S.x;
    if (S.obs_present && S.obs_known && rel > 0.0 && rel < P->cam_range_m &&
        fabs(S.y) < 2.0 * P->lane_half_m) {
        p->obstacle = 1;
        p->zone = fabs(S.y) < P->lane_half_m ? ZONE_CENTER : (S.y > 0 ? ZONE_RIGHT : ZONE_LEFT);
        p->cam_dist_m = (float)(rel * (1.0 + 0.03 * gauss()));
        p->confidence = 85;
        p->class_id = 39;                               /* COCO "bottle" */
    }

    if (S.faults & FBIT(FAULT_CAM_MISS)) {
        p->obstacle = 0;
        p->zone = ZONE_NONE;
        p->confidence = 0;
    }
    if (S.faults & FBIT(FAULT_CAM_PHANTOM)) {          /* e.g. photo of a person */
        p->obstacle = 1;
        p->zone = ZONE_CENTER;
        p->cam_dist_m = 0.4f * P->xcheck_max_m;
        p->confidence = 70;
        p->class_id = 0;
    }
    if (S.faults & FBIT(FAULT_CAM_FLICKER)) {          /* bad light at the left edge */
        S.flick ^= 1;
        p->obstacle = (uint8_t)S.flick;
        p->zone = S.flick ? ZONE_LEFT : ZONE_NONE;
        p->cam_dist_m = 0.5f * P->cam_range_m;
        p->confidence = 50;
    }
    S.last_perc = *p;

    if (S.faults & FBIT(FAULT_CAM_LAG)) {              /* deliver the frame from ~300 ms ago */
        S.lag[S.lag_idx % LAG_FRAMES] = *p;
        S.lag_idx++;
        if (S.lag_idx < LAG_FRAMES)
            return 0;
        *p = S.lag[S.lag_idx % LAG_FRAMES];
    } else {
        S.lag_idx = 0;
    }
    return 1;
}

static int connect_sup(void)
{
    int coid = name_open(AV_NAME_SUPERVISOR, 0);
    if (coid == -1)
        return -1;
    int err = qnx_hello(coid, AV_ROLE_SENSOR | AV_ROLE_PERCEPTION, "vehicle_simulator");
    if (err) {
        fprintf(stderr, "[SIMULATOR] supervisor refused HELLO: %s\n", strerror(err));
        name_close(coid);
        return -1;
    }
    printf("[SIMULATOR] connected to supervisor\n");
    return coid;
}

static void send_or_drop(int *coid, const void *msg, size_t len)
{
    if (*coid == -1)
        return;
    if (qnx_send_timeout(*coid, msg, len, NULL, 0, 5000) == -1 &&
        (errno == EBADF || errno == ESRCH || errno == ENOTCONN || errno == EPERM)) {
        name_close(*coid);
        *coid = -1;
    }
}

static void *physics_thread(void *arg)
{
    (void)arg;
    qnx_set_priority(AV_PRIO_SIM);

    int chid = ChannelCreate(0);
    qnx_pulse_timer(chid, AV_PRIO_SIM, AV_PULSE_TICK, AV_TICK_MS * 1000u, NULL);

    int sup = -1, nav = -1;
    uint64_t next_try = 0, last_print = 0;

    for (;;) {
        struct _pulse pulse;
        if (MsgReceivePulse(chid, &pulse, sizeof pulse, NULL) == -1)
            continue;
        uint64_t now = qnx_now_us();

        if (now >= next_try) {
            if (sup == -1)
                sup = connect_sup();
            if (nav == -1)
                nav = name_open(AV_NAME_NAV, 0);
            next_try = now + 1000000u;
        }

        av_sensor_state_t sm;
        av_perception_t pm;
        int send_perc;

        pthread_mutex_lock(&S.mtx);
        S.tick++;
        step_vehicle(now, AV_TICK_MS * 1e-3);
        make_sensor_msg(&sm, now);
        send_perc = (S.tick % AV_PERC_EVERY_TICKS) == 0 && make_perception(&pm, now);
        int link_drop = (S.faults & FBIT(FAULT_LINK_DROP)) != 0;
        if (S.verbose && now - last_print > 1000000u) {
            last_print = now;
            printf("[SIMULATOR] position x=%6.2f m y=%5.2f m | speed %5.2f m/s | ultrasonic %5.2f m%s | STM32 state %d | faults 0x%x\n", S.x, S.y,
                   0.5 * (S.vl + S.vr), S.dist, S.no_echo ? "(none)" : "", S.island, S.faults);
        }
        pthread_mutex_unlock(&S.mtx);

        if (!link_drop)                         /* pulled wire: STM32 frames stop arriving */
            send_or_drop(&sup, &sm, sizeof sm);
        if (send_perc) {
            send_or_drop(&sup, &pm, sizeof pm);
            send_or_drop(&nav, &pm, sizeof pm);
        }
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
/* server thread (main)                                                */
/* ------------------------------------------------------------------ */

typedef union {
    uint16_t          type;
    struct _pulse     pulse;
    av_hdr_t         hdr;
    av_hello_t       hello;
    av_vehicle_cmd_t cmd;
    av_sim_ctrl_t    ctrl;
} sim_msg_t;

static void handle_ctrl(int rcvid, const av_sim_ctrl_t *c)
{
    pthread_mutex_lock(&S.mtx);
    switch (c->cmd) {
    case SIM_CTRL_FAULT:
        if (c->fault < 32) {
            if (c->fault == FAULT_IMU_FROZEN) {
                S.frozen_gyro = S.gyro;
                S.frozen_ax = S.ax;
                S.frozen_ay = S.ay;
            }
            S.faults |= FBIT(c->fault);
            printf("[SIMULATOR] fault ON : %s\n", av_fault_name((int)c->fault));
        }
        break;
    case SIM_CTRL_CLEAR:
        S.faults = c->fault ? (S.faults & ~FBIT(c->fault)) : 0;
        printf("[SIMULATOR] faults cleared\n");
        break;
    case SIM_CTRL_OBSTACLE:
        S.obs_present = 1;
        S.obs_known = c->flag != 0;
        S.obs_x = S.x + c->value;
        printf("[SIMULATOR] obstacle %.2f m ahead (%s to the camera AI)\n", c->value,
               S.obs_known ? "known" : "UNKNOWN");
        break;
    case SIM_CTRL_OBSTACLE_OFF:
        S.obs_present = 0;
        printf("[SIMULATOR] obstacle removed\n");
        break;
    case SIM_CTRL_RESET:
        reset_world();
        printf("[SIMULATOR] world reset\n");
        break;
    case SIM_CTRL_ESTOP:
        S.estop = c->flag != 0;
        printf("[SIMULATOR] E-STOP button %s\n", S.estop ? "PRESSED" : "released");
        break;
    default:
        break;
    }
    av_sim_status_t st;
    memset(&st, 0, sizeof st);
    st.x = (float)S.x;
    st.y = (float)S.y;
    st.v = (float)(0.5 * (S.vl + S.vr));
    st.dist = S.no_echo ? -1.0f : S.dist;
    st.obs_x = (float)S.obs_x;
    st.faults = S.faults;
    st.collided = (uint8_t)S.collided;
    st.island_tripped = S.island == ISL_TRIPPED;
    st.estop = (uint8_t)S.estop;
    st.obs_present = (uint8_t)S.obs_present;
    st.obs_known = (uint8_t)S.obs_known;
    st.motor_on = S.island == ISL_ARMED && S.cmd.motor_enable;
    pthread_mutex_unlock(&S.mtx);
    MsgReply(rcvid, EOK, &st, sizeof st);
}

int main(int argc, char *argv[])
{
    const char *profile = "robot";
    int opt;
    while ((opt = getopt(argc, argv, "s:vh")) != -1) {
        switch (opt) {
        case 's': profile = optarg; break;
        case 'v': S.verbose = 1; break;
        default:
            fprintf(stderr, "usage: %s [-s robot|car] [-v]\n", argv[0]);
            return 1;
        }
    }
    S.P = av_profile_get(profile);
    const av_profile_t *P = S.P;
    pthread_mutex_init(&S.mtx, NULL);
    reset_world();
    S.start_us = qnx_now_us();
    S.island = ISL_ARMING;
    S.no_echo = 1;
    S.dist = P->us_max_m;

    health_init(&S.tr[SENS_ENC_L], -1.5f * P->v_max, 1.5f * P->v_max, 3.0f * P->a_phys_max, 50, P->enc_frozen_ms);
    health_init(&S.tr[SENS_ENC_R], -1.5f * P->v_max, 1.5f * P->v_max, 3.0f * P->a_phys_max, 50, P->enc_frozen_ms);
    health_init(&S.tr[SENS_US], 0.0f, P->us_max_m + 0.01f, 0.0f, 150, 1000);
    health_init(&S.tr[SENS_IMU], -P->gyro_max_dps, P->gyro_max_dps, 5000.0f, 50, P->imu_frozen_ms);

    qnx_set_priority(AV_PRIO_SIM);
    access_init(&g_roles);
    qnx_server_t srv;
    if (qnx_server_create(&srv, AV_NAME_LINK, 0) == -1)
        return 1;

    pthread_t th;
    pthread_create(&th, NULL, physics_thread, NULL);
    printf("[SIMULATOR] simulated vehicle '%s' registered as %s\n", P->name, AV_NAME_LINK);

    for (;;) {
        sim_msg_t m;
        struct _msg_info info;
        int rcvid = MsgReceive(srv.chid, &m, sizeof m, &info);
        if (rcvid == -1)
            continue;
        if (rcvid == 0) {
            if (m.pulse.code == _PULSE_CODE_DISCONNECT) {
                access_drop(&g_roles, m.pulse.scoid, NULL, NULL, 0);
                ConnectDetach(m.pulse.scoid);
            }
            continue;
        }
        if (qnx_handle_system_msg(rcvid, &m))
            continue;

        switch (m.hdr.type) {
        case AV_MSG_HELLO: {
            m.hello.name[sizeof m.hello.name - 1] = '\0';
            int err = access_hello(&g_roles, info.scoid, info.pid, m.hello.roles, m.hello.name);
            if (err)
                MsgError(rcvid, err);
            else
                MsgReply(rcvid, EOK, NULL, 0);
            break;
        }
        case AV_MSG_VEHICLE_CMD:
            /* Only the supervisor may drive the vehicle. */
            if (!access_has_role(&g_roles, info.scoid, AV_ROLE_SUPERVISOR)) {
                MsgError(rcvid, EPERM);
                break;
            }
            pthread_mutex_lock(&S.mtx);
            if (!(S.faults & FBIT(FAULT_LINK_DROP))) {    /* pulled wire: frames never arrive */
                S.cmd = m.cmd;
                S.cmd_rx_us = qnx_now_us();
                S.cmd_seen = 1;
                if (S.island != ISL_ARMED)
                    S.rearm++;
            }
            pthread_mutex_unlock(&S.mtx);
            MsgReply(rcvid, EOK, NULL, 0);
            break;
        case AV_MSG_SIM_CTRL:
            handle_ctrl(rcvid, &m.ctrl);
            break;
        default:
            MsgError(rcvid, ENOSYS);
            break;
        }
    }
    return 0;
}
