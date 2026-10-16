/*
 * unit_tests.c - host unit tests of the supervisor decision logic (no QNX needed).
 *
 * Feeds scripted sensor frames, perception packets and navigation commands into sup_core
 * and checks the verdicts and states. Build and run:  make test
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "decision_core.h"
#include "safety_deadline.h"
#include "vehicle_physics.h"
#include "hw_protocols.h"

static int g_pass, g_fail;
#define CHECK(cond, msg)                                                          \
    do {                                                                          \
        if (cond) {                                                               \
            g_pass++;                                                             \
            printf("  ok    %s\n", msg);                                          \
        } else {                                                                  \
            g_fail++;                                                             \
            printf("  FAIL  %s   [state %s]  (line %d)\n", msg,                  \
                   av_state_name(C.state), __LINE__);                            \
        }                                                                         \
    } while (0)

static sup_ctx_t C;
static const av_profile_t *P;
static uint64_t T;
static uint32_t g_sseq, g_pseq, g_nseq;

/* simulated world, edited by each test */
static float   g_v, g_dist, g_gyro_set, g_nav_speed, g_nav_steer;
static int     g_echo, g_enc_straight, g_island, g_hb, g_auto, g_sensors_on, g_ack;
static uint8_t g_health[SENS_COUNT];

static void emit(void *u, const av_event_t *e)
{
    (void)u;
    (void)e;
}

/* sensors of a vehicle that follows the supervisor's output, unless a fault is simulated */
static void send_sensors(void)
{
    av_sensor_state_t s;
    memset(&s, 0, sizeof s);
    s.seq = ++g_sseq;
    s.t_us = T;
    float l = g_v, r = g_v;
    if (!g_enc_straight)
        physics_steer_to_wheels(g_v, C.out.steer_deg, P->wheelbase_m, P->track_m, &l, &r);
    s.v_left = l;
    s.v_right = r;
    s.gyro_z_dps = isnan(g_gyro_set) ? physics_yaw_from_steer_dps(g_v, C.out.steer_deg, P->wheelbase_m)
                                     : g_gyro_set;
    s.distance_m = g_echo ? g_dist : P->us_max_m;
    s.us_no_echo = (uint8_t)!g_echo;
    memcpy(s.health, g_health, sizeof s.health);
    s.island_state = (uint8_t)g_island;
    s.ack_button = (uint8_t)g_ack;
    sup_core_on_sensor(&C, &s, T);
}

static void perception(int obstacle, int zone, float cam_dist)
{
    av_perception_t p;
    memset(&p, 0, sizeof p);
    p.seq = ++g_pseq;
    p.t_capture_us = T;
    p.obstacle = (uint8_t)obstacle;
    p.zone = (uint8_t)zone;
    p.cam_dist_m = cam_dist;
    p.odom_speed = g_v;
    sup_core_on_perception(&C, &p, T);
}

static av_verdict_t nav(float speed, float steer)
{
    av_nav_cmd_t m;
    memset(&m, 0, sizeof m);
    m.seq = ++g_nseq;
    m.t_us = T;
    m.target_speed = speed;
    m.steer_deg = steer;
    m.nav_speed_est = g_v;
    av_verdict_t v;
    sup_core_on_nav_cmd(&C, &m, T, &v);
    C.actions = 0;
    return v;
}

/* advance time in 10 ms steps: sensors, nav heartbeat, nav command every 50 ms, tick */
static void run_ms(int ms)
{
    for (int t = 0; t < ms; t += 10) {
        T += 10000;
        if (g_sensors_on)
            send_sensors();
        if (g_hb)
            sup_core_on_nav_heartbeat(&C, T);
        if (g_auto && (t % 50) == 0)
            nav(g_nav_speed, g_nav_steer);
        sup_core_tick(&C, T);
        C.actions = 0;
    }
}

static void setup(const char *name, const char *profile)
{
    printf("\n[%s]\n", name);
    P = av_profile_get(profile);
    T = 1000000;
    sup_core_init(&C, P, 0, emit, NULL, T);
    g_v = 0.0f;
    g_dist = 4.0f;
    g_echo = 0;
    g_gyro_set = NAN;
    g_enc_straight = 0;
    g_island = 0;
    g_hb = 1;
    g_auto = 1;
    g_sensors_on = 1;
    g_nav_speed = 0.0f;
    g_nav_steer = 0.0f;
    g_ack = 0;
    memset(g_health, H_OK, sizeof g_health);
    sup_core_on_nav_connect(&C, 1234, T);
    run_ms(50);
}

static void cruise(float v)
{
    g_v = v;
    g_nav_speed = v;
    run_ms(100);
}

/* ------------------------------------------------------------------ */

static void test_physics(void)
{
    printf("\n[physics]\n");
    P = av_profile_get("robot");
    CHECK(fabsf(physics_allowed_speed(0.14f, 0.03f, 1.0f) - 0.5f) < 0.01f,
          "robot: from 0.5 m/s a comfortable stop needs 14 cm + 10 cm margin");
    CHECK(fabsf(physics_delta_max_deg(0.5f, 0.2f, 0.5f, 30.0f) - 21.8f) < 0.1f,
          "robot: max safe steering at 0.5 m/s is 21.8 deg");
    CHECK(physics_allowed_speed(28.0f, 0.1f, 9.0f) < 22.2f,
          "car: 80 km/h with an obstacle at 30 m needs emergency braking");
    CHECK(physics_stopping_distance(22.2f, 0.1f, 9.0f) < 30.0f,
          "car: ...and 0.92 g stops it within 30 m");
    CHECK(fabsf(physics_delta_max_deg(22.2f, 2.7f, 4.0f, 35.0f) - 1.25f) < 0.05f,
          "car: max safe road-wheel angle at 80 km/h is 1.25 deg");
}

static void test_startup_and_pass(void)
{
    setup("startup + normal command", "robot");
    CHECK(C.state == ST_NORMAL && C.authority == AUTH_NAV, "first healthy sensor frame -> NORMAL, nav has authority");
    cruise(0.5f);
    av_verdict_t v = nav(0.5f, 0.0f);
    CHECK(v.verdict == V_PASS, "cruise 0.5 m/s straight -> PASS");
}

static void test_slew_and_corrupt(void)
{
    setup("AI hallucination: impossible steering jump + corrupted value", "robot");
    cruise(0.5f);
    g_auto = 0;
    run_ms(50);
    av_verdict_t v = nav(0.5f, 30.0f);
    CHECK(v.verdict == V_REJECT && (v.reasons & R_SLEW), "0 -> 30 deg in 50 ms -> REJECT (SLEW)");
    CHECK(C.state == ST_NORMAL && fabsf(C.out.steer_deg) < 0.01f, "last safe command kept, car keeps driving straight");
    run_ms(50);
    v = nav(0.5f, 999.0f);
    CHECK(v.verdict == V_REJECT && (v.reasons & R_CORRUPT), "steer = 999 deg -> REJECT (CORRUPT)");
}

static void test_envelope(void)
{
    setup("steering envelope (too much steering for the speed)", "robot");
    cruise(0.5f);
    g_auto = 0;
    av_verdict_t v = { 0 };
    for (float s = 2.5f; s <= 30.0f; s += 2.5f) {
        run_ms(50);
        v = nav(0.5f, s);
    }
    CHECK(v.verdict == V_CLAMP && (v.reasons & R_ENVELOPE), "30 deg at 0.5 m/s -> CLAMP (ENVELOPE)");
    CHECK(fabsf(v.out_steer - 21.8f) < 0.2f, "steering limited to 21.8 deg");
    CHECK(C.state == ST_NORMAL, "no fault: vehicle follows, state stays NORMAL");
}

static void test_camera_miss_full_cycle(void)
{
    setup("missed obstacle: MRM -> SAFE -> RECOVERY -> ack -> NORMAL", "robot");
    cruise(0.5f);
    g_auto = 0;
    run_ms(10);
    g_echo = 1;
    g_dist = 0.20f;
    run_ms(10);
    av_verdict_t v = nav(0.5f, 0.0f);
    CHECK(v.verdict == V_REJECT && (v.reasons & R_CAMERA_MISS), "nav drives at an unseen obstacle -> REJECT (CAMERA_MISS)");
    CHECK(C.state == ST_MRM && C.authority == AUTH_RECOVERY, "-> MRM, recovery manager takes authority");

    g_auto = 1;
    while (g_v > 0.0f) {                         /* vehicle decelerates at 1 m/s^2 */
        g_v = fmaxf(0.0f, g_v - 0.01f);
        run_ms(10);
    }
    run_ms(300);
    CHECK(C.state == ST_SAFE, "vehicle stopped -> SAFE (latched)");
    run_ms(2500);
    CHECK(C.state == ST_SAFE, "obstacle still 20 cm ahead -> stays SAFE (PATH_BLOCKED)");
    g_echo = 0;
    run_ms(2100);
    CHECK(C.state == ST_RECOVERY, "path clear for 2 s -> RECOVERY (waits for operator)");
    CHECK(sup_core_on_ack(&C, T) == 1 && C.state == ST_NORMAL, "operator ack -> NORMAL");
}

static void test_graded_camera_miss(void)
{
    setup("camera miss seen early by cross-check -> CAUTION (graded)", "robot");
    cruise(0.5f);
    g_echo = 1;
    g_dist = 0.9f;
    for (int i = 0; i < 4; i++) {
        perception(0, ZONE_NONE, 0.0f);
        run_ms(30);
    }
    CHECK(C.conds & CBIT(C_CAMERA_MISS), "ultrasonic sees 0.9 m, camera sees nothing -> CAMERA_MISS");
    CHECK(C.state == ST_CAUTION && C.out.speed_sp <= 0.25f + 1e-3f, "-> CAUTION, speed capped to 0.25 m/s");
}

static void test_collision_imminent(void)
{
    setup("obstacle appears very close: ESTOP", "robot");
    cruise(0.5f);
    g_echo = 1;
    g_dist = 0.12f;
    run_ms(10);
    CHECK(C.state == ST_ESTOP && C.out.brake_mode == BRAKE_MAX, "12 cm at 0.5 m/s -> ESTOP, maximum braking");
}

static void test_flicker(void)
{
    setup("camera flicker: swerve lock-out", "robot");
    cruise(0.5f);
    for (int i = 0; i < 6; i++) {
        perception(i & 1, (i & 1) ? ZONE_LEFT : ZONE_NONE, 0.8f);
        run_ms(30);
    }
    CHECK(C.conds & CBIT(C_PERC_UNSTABLE), "obstacle flag toggles within 200 ms -> PERC_UNSTABLE");
    CHECK(C.state == ST_CAUTION, "-> CAUTION");
    g_auto = 0;
    run_ms(50);
    av_verdict_t v = nav(0.5f, 2.5f);
    CHECK(v.verdict == V_CLAMP && (v.reasons & R_SWERVE_LOCK) && (v.reasons & R_STATE_CAP),
          "swerve limited to 2 deg and speed capped -> CLAMP");
}

static void test_phantom(void)
{
    setup("phantom obstacle: camera sees something the ultrasonic does not", "robot");
    cruise(0.3f);
    for (int i = 0; i < 4; i++) {
        perception(1, ZONE_CENTER, 0.4f);
        run_ms(30);
    }
    CHECK(C.conds & CBIT(C_PHANTOM), "camera 0.4 m ahead, ultrasonic clear -> PHANTOM");
    CHECK(C.state == ST_CAUTION, "-> CAUTION (no evasive swerve)");
}

static void test_nav_crash_and_hang(void)
{
    setup("navigation crash", "robot");
    cruise(0.5f);
    sup_core_on_nav_disconnect(&C, T);
    CHECK((C.actions & ACT_RESTART_NAV) && C.state == ST_MRM, "kernel disconnect pulse -> MRM + restart request");

    setup("navigation hang", "robot");
    cruise(0.5f);
    g_hb = 0;
    g_auto = 0;
    run_ms(160);
    CHECK(C.conds & CBIT(C_NAV_HB_LOST), "no heartbeat / commands for 100 ms -> NAV_HB_LOST");
    CHECK(C.state == ST_MRM || C.state == ST_ESTOP, "-> controlled stop");
    run_ms(500);
    CHECK(C.restart_requested, "silent for 500 ms -> kill + restart requested");
}

static void test_yaw_vote(void)
{
    setup("2oo3 yaw vote: IMU is the odd one out", "robot");
    cruise(0.5f);
    g_gyro_set = 20.0f;                        /* gyro says turning, model + encoders say straight */
    run_ms(100);
    CHECK(C.conds & CBIT(C_YAW_IMU_SUSPECT), "gyro disagrees with model AND encoders -> IMU suspect");
    CHECK(C.state == ST_DEGRADED, "-> DEGRADED (keep driving slowly, ignore IMU)");

    setup("2oo3 yaw vote: vehicle does not follow the steering", "robot");
    cruise(0.5f);
    g_enc_straight = 1;
    g_gyro_set = 0.0f;                         /* gyro + encoders agree: not turning */
    g_auto = 0;
    int flagged = 0;
    for (float s = 2.5f; s <= 10.0f; s += 2.5f) {
        for (int i = 0; i < 5; i++) {
            run_ms(10);
            flagged |= (C.conds & CBIT(C_ACTUATOR_FAULT)) != 0;
        }
        nav(0.5f, s);
    }
    for (int i = 0; i < 10; i++) {
        run_ms(10);
        flagged |= (C.conds & CBIT(C_ACTUATOR_FAULT)) != 0;
    }
    /* (in MRM the steering returns to 0, model and vehicle agree again: the flag clears,
     *  but the state stays latched in the controlled stop) */
    CHECK(flagged, "model disagrees with gyro AND encoders -> ACTUATOR_FAULT");
    CHECK(C.state == ST_MRM || C.state == ST_ESTOP, "-> controlled stop");
}

static void test_links_and_island(void)
{
    setup("sensor link lost (pulled wire)", "robot");
    cruise(0.5f);
    g_sensors_on = 0;
    run_ms(50);
    CHECK(C.conds & CBIT(C_SENSOR_LINK_LOST), "no sensor frame for 30 ms -> SENSOR_LINK_LOST");
    CHECK(C.state == ST_MRM, "-> MRM");

    setup("STM32 safety island tripped", "robot");
    cruise(0.5f);
    g_island = 1;
    run_ms(10);
    CHECK(C.state == ST_ESTOP, "island reports a trip -> ESTOP");
}

static void test_trust(void)
{
    setup("navigation trust: repeated bad commands", "robot");
    cruise(0.5f);
    g_auto = 0;
    for (int i = 0; i < 3; i++) {
        run_ms(50);
        nav(0.5f, 30.0f);
    }
    run_ms(10);
    CHECK(C.conds & CBIT(C_NAV_UNTRUSTED), "3 rejected commands within 1 s -> NAV_UNTRUSTED");
    CHECK(C.state == ST_MRM, "-> MRM");
}

static void test_deadline(void)
{
    setup("safe-state deadline: controlled stop in time -> PASS", "robot");
    CHECK(fabsf(dl_budget_ms(0.5f, 1.0f) - 925.0f) < 0.5f,
          "deadline from 0.5 m/s at 1 m/s^2 = 100 + 1.25 x 500 + 200 = 925 ms");
    cruise(0.5f);
    sup_core_on_nav_disconnect(&C, T);
    CHECK(C.state == ST_MRM && C.dl_active && fabsf(C.dl_budget_ms - 925.0f) < 1.0f,
          "nav crash -> MRM, deadline timer started (925 ms)");
    while (g_v > 0.0f) {                         /* vehicle decelerates at 1 m/s^2 */
        g_v = fmaxf(0.0f, g_v - 0.01f);
        run_ms(10);
    }
    run_ms(300);
    CHECK(C.state == ST_SAFE && C.dl_pass_n == 1 && C.dl_fail_n == 0 && !C.dl_active,
          "standing still in time -> SAFE, deadline PASS");
    CHECK(C.dl_last_ms < C.dl_budget_ms, "time used is below the deadline");

    setup("safe-state deadline: brakes do not work -> MISSED -> FAIL", "robot");
    cruise(0.5f);
    sup_core_on_nav_disconnect(&C, T);
    run_ms(950);                                 /* vehicle does not slow down at all */
    CHECK(C.dl_missed && (C.conds & CBIT(C_DEADLINE_MISSED)), "still moving after 925 ms -> STOP_DEADLINE_MISSED");
    CHECK(C.state == ST_ESTOP && C.out.motor_enable == 0, "-> ESTOP, motor relay cut");
    while (g_v > 0.0f) {
        g_v = fmaxf(0.0f, g_v - 0.02f);
        run_ms(10);
    }
    run_ms(300);
    CHECK(C.state == ST_SAFE && C.dl_fail_n == 1 && !(C.conds & CBIT(C_DEADLINE_MISSED)),
          "stopped late -> SAFE, deadline FAIL recorded, problem cleared");
}

static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { put16(p, (uint16_t)v); put16(p + 2, (uint16_t)(v >> 16)); }

/* a Jetson packet as the Jetson would build it */
static void jet_packet(uint8_t *b, uint32_t seq, uint8_t obstacle, uint8_t zone, float dist)
{
    uint32_t d;
    memset(b, 0, AV_JET_LEN);
    put32(b, AV_JET_MAGIC);
    put16(b + 4, AV_JET_VERSION);
    put16(b + 6, AV_JET_LEN);
    put32(b + 8, seq);
    b[12] = obstacle;
    b[13] = zone;
    b[14] = 87;
    b[15] = 0;
    memcpy(&d, &dist, 4);
    put32(b + 16, d);
    put16(b + 20, 40);
    put16(b + 22, crc16_ccitt(b, 22));
}

static void test_rover_hardware(void)
{
    printf("\n[real rover: TF-Luna, wheel encoders, Jetson packet]\n");

    /* TF-Luna frame: 123 cm, signal 500, after some garbage bytes */
    uint8_t fr[9] = { 0x59, 0x59, 123, 0, 0xF4, 0x01, 0xE8, 0x08, 0 };
    for (int i = 0; i < 8; i++)
        fr[8] = (uint8_t)(fr[8] + fr[i]);
    tfluna_parser_t tp;
    tfluna_frame_t f;
    tfluna_init(&tp);
    int got = 0;
    const uint8_t junk[] = { 0x00, 0x59, 0x13, 0x42 };
    for (unsigned i = 0; i < sizeof junk; i++)
        got += tfluna_feed(&tp, junk[i], &f);
    for (int i = 0; i < 9; i++)
        got += tfluna_feed(&tp, fr[i], &f);
    CHECK(got == 1 && fabsf(f.dist_m - 1.23f) < 1e-3f && f.amp == 500,
          "TF-Luna: frame found after garbage -> 1.23 m, signal 500");
    fr[3] = 1;                                   /* corrupted on the wire */
    got = 0;
    for (int i = 0; i < 9; i++)
        got += tfluna_feed(&tp, fr[i], &f);
    CHECK(got == 0 && tp.bad_checksum == 1, "TF-Luna: corrupted frame rejected by the checksum");

    /* encoder: 20 slots, 6.5 cm wheel = 1.02 cm per pulse; 0.30 m/s = one pulse every 34 ms */
    float mpp = 3.14159265f * 0.065f / 20.0f;
    enc_est_t e;
    enc_init(&e);
    uint64_t t = 1000000, next = t;
    float v = 0.0f;
    for (int ms = 0; ms < 2000; ms++, t += 1000) {
        if (t >= next) {
            enc_pulse(&e, t, 2000);
            next = t + (uint64_t)(mpp / 0.30f * 1e6f);
        }
        if (ms % 10 == 0)
            v = enc_speed(&e, t, mpp, 250000, 3.0f);
    }
    CHECK(fabsf(v - 0.30f) < 0.03f, "encoder: steady pulses -> 0.30 m/s");
    for (int ms = 0; ms < 400; ms += 10, t += 10000)
        v = enc_speed(&e, t, mpp, 250000, 3.0f);
    CHECK(v == 0.0f, "encoder: pulses stop -> speed falls to 0 within 0.4 s");
    uint32_t before = e.total;
    enc_pulse(&e, t, 2000);
    enc_pulse(&e, t + 500, 2000);
    CHECK(e.total == before + 1, "encoder: second edge 0.5 ms later is contact bounce, ignored");

    /* Jetson packet */
    uint8_t b[AV_JET_LEN];
    jetson_det_t d;
    jet_packet(b, 7, 1, ZONE_CENTER, 1.4f);
    CHECK(jetson_parse(b, AV_JET_LEN, &d) == JET_OK && d.seq == 7 && d.obstacle == 1 &&
          d.zone == ZONE_CENTER && fabsf(d.dist_m - 1.4f) < 1e-6f && d.age_ms == 40,
          "Jetson: valid packet accepted (seq 7, obstacle CENTER 1.4 m)");
    b[13] ^= 0x01;
    CHECK(jetson_parse(b, AV_JET_LEN, &d) == JET_BAD_CRC, "Jetson: one flipped bit -> CRC error, dropped");
    jet_packet(b, 8, 1, 7, 1.4f);
    CHECK(jetson_parse(b, AV_JET_LEN, &d) == JET_BAD_VALUE, "Jetson: zone 7 (does not exist) -> dropped");
    CHECK(jetson_parse(b, 20, &d) == JET_BAD_LEN, "Jetson: short packet -> dropped");

    /* rover profile: the stop point stays outside the TF-Luna's 0.2 m blind zone */
    const av_profile_t *R = av_profile_get("rover");
    CHECK(R->margin_m >= 0.2f && physics_stopping_distance(R->v_cruise, R->t_sys_s, R->a_comf) +
                                     R->margin_m < 1.0f,
          "rover: stops >= 0.2 m before an obstacle seen 1 m ahead at cruise speed");

    setup("rover: physical ACK button", "robot");
    cruise(0.5f);
    g_auto = 0;
    g_echo = 1;
    g_dist = 0.20f;
    run_ms(10);
    nav(0.5f, 0.0f);
    g_auto = 1;
    while (g_v > 0.0f) {
        g_v = fmaxf(0.0f, g_v - 0.01f);
        run_ms(10);
    }
    g_echo = 0;
    run_ms(2700);
    CHECK(C.state == ST_RECOVERY, "stopped, path clear for 2 s -> RECOVERY");
    g_ack = 1;
    run_ms(10);
    CHECK(C.state == ST_NORMAL, "ACK button pressed -> NORMAL (same as typing ack)");
}

static void test_car_scale(void)
{
    setup("full-size car: 80 km/h, obstacle appears at 30 m", "car");
    cruise(22.2f);
    g_echo = 1;
    g_dist = 30.0f;
    run_ms(10);
    CHECK(C.state == ST_ESTOP, "80 km/h + 30 m -> ESTOP (needs ~0.9 g)");

    setup("full-size car: 40 km/h, obstacle at 30 m", "car");
    cruise(11.1f);
    g_echo = 1;
    g_dist = 30.0f;
    run_ms(10);
    CHECK(C.state == ST_NORMAL, "40 km/h + 30 m -> no emergency, nav can brake normally");
}

int main(void)
{
    test_physics();
    test_startup_and_pass();
    test_slew_and_corrupt();
    test_envelope();
    test_camera_miss_full_cycle();
    test_graded_camera_miss();
    test_collision_imminent();
    test_flicker();
    test_phantom();
    test_nav_crash_and_hang();
    test_yaw_vote();
    test_links_and_island();
    test_trust();
    test_deadline();
    test_rover_hardware();
    test_car_scale();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
