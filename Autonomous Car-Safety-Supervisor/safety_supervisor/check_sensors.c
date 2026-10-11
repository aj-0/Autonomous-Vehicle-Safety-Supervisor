/*
 * check_sensors.c - is the sensor data trustworthy?
 *
 *  - per-sensor health flags (computed by sensor_monitor / vehicle_simulator)
 *  - true speed from the encoders (never from navigation)
 *  - 2-out-of-3 yaw vote: model vs gyro vs encoders -> identifies the faulty component
 *  - braking confirmation: when the supervisor brakes, the vehicle must actually slow down
 */
#include <math.h>
#include "checks.h"
#include "decision_core.h"
#include "vehicle_physics.h"

#define RAD2DEG 57.29577951308232f

static void yaw_vote(sup_ctx_t *c, uint64_t now, int enc_ok, int imu_ok)
{
    const av_profile_t *P = c->P;

    /* Expected yaw rate: bicycle model fed with the command actually sent to the vehicle,
     * rate-limited like the drivetrain (wheels cannot change speed instantly). */
    float dt = c->sens_prev_rx_us ? (float)(now - c->sens_prev_rx_us) * 1e-6f : 0.01f;
    if (dt > 0.1f)
        dt = 0.1f;
    float r_cmd = physics_yaw_from_steer_dps(c->v_true, c->out.steer_deg, P->wheelbase_m);
    float max_dr = 2.0f * P->accel_max / P->track_m * RAD2DEG * dt;
    float diff = r_cmd - c->yaw_model;
    if (diff > max_dr)
        diff = max_dr;
    if (diff < -max_dr)
        diff = -max_dr;
    c->yaw_model += diff;

    if (c->v_valid && c->v_true < P->standstill_v) {
        /* stopped: forget old verdicts, the vote is re-run after the restart */
        c->pers_mg = c->pers_me = c->pers_ge = 0;
        sup_cond(c, C_YAW_IMU_SUSPECT, 0);
        sup_cond(c, C_YAW_ENC_SUSPECT, 0);
        sup_cond(c, C_ACTUATOR_FAULT, 0);
        sup_cond(c, C_MULTI_FAULT, 0);
        return;
    }
    if (c->bench_mode || !c->v_valid || c->v_true < P->yaw_min_speed) {
        c->pers_mg = c->pers_me = c->pers_ge = 0;
        return;                                  /* keep previous verdict while too slow */
    }

    float rm = c->yaw_model;
    float rg = c->sens.gyro_z_dps;
    float re = physics_yaw_from_wheels_dps(c->sens.v_left, c->sens.v_right, P->track_m);
    float th = P->yaw_thresh_dps;

    c->pers_mg = (imu_ok && fabsf(rm - rg) > th) ? c->pers_mg + 1 : 0;
    c->pers_me = (enc_ok && fabsf(rm - re) > th) ? c->pers_me + 1 : 0;
    c->pers_ge = (imu_ok && enc_ok && fabsf(rg - re) > th) ? c->pers_ge + 1 : 0;

    int mg = c->pers_mg >= AV_PERSIST_SAMPLES;
    int me = c->pers_me >= AV_PERSIST_SAMPLES;
    int ge = c->pers_ge >= AV_PERSIST_SAMPLES;

    int multi    = mg && me && ge;
    int imu_bad  = !multi && mg && ge && !me;      /* gyro disagrees with both others     */
    int enc_bad  = !multi && me && ge && !mg;      /* encoders disagree with both others  */
    int actuator = !multi && mg && me && !ge;      /* vehicle does not follow the command */
    /* only two sources left: cannot tell which one is wrong -> assume the worst */
    if (!imu_ok && me)
        actuator = 1;
    if (!enc_ok && mg)
        actuator = 1;

    sup_cond(c, C_MULTI_FAULT, multi);
    sup_cond(c, C_YAW_IMU_SUSPECT, imu_bad);
    sup_cond(c, C_YAW_ENC_SUSPECT, enc_bad);
    sup_cond(c, C_ACTUATOR_FAULT, actuator);
}

static void brake_check(sup_ctx_t *c, uint64_t now)
{
    const av_profile_t *P = c->P;

    if ((c->state == ST_MRM || c->state == ST_ESTOP) && c->brake_t0 && c->v_valid &&
        now - c->brake_t0 > (uint64_t)AV_BRAKE_CHECK_MS * 1000u &&
        c->brake_v0 > 5.0f * P->standstill_v) {
        int not_slowing = c->v_true > 0.9f * c->brake_v0 && c->sens.accel_x > -0.2f * P->a_comf;
        if (not_slowing)
            sup_cond(c, C_BRAKE_INEFFECTIVE, 1);
    }
    if (c->v_valid && c->v_true < P->standstill_v)
        sup_cond(c, C_BRAKE_INEFFECTIVE, 0);
}

void chk_input_on_sensor(sup_ctx_t *c, uint64_t now)
{
    const av_profile_t *P = c->P;
    const av_sensor_state_t *s = &c->sens;

    int enc_l = s->health[SENS_ENC_L] == H_OK;
    int enc_r = s->health[SENS_ENC_R] == H_OK;
    int us    = s->health[SENS_US] == H_OK;
    int imu   = s->health[SENS_IMU] == H_OK;

    /* true speed: only from healthy encoders */
    if (enc_l && enc_r) {
        c->v_true = 0.5f * (s->v_left + s->v_right);
        c->v_valid = 1;
    } else if (enc_l || enc_r) {
        c->v_true = enc_l ? s->v_left : s->v_right;
        c->v_valid = 1;
    } else {
        c->v_valid = 0;
    }
    if (c->v_true < 0.0f)
        c->v_true = 0.0f;
    sup_cond(c, C_ENC_FAULT_ONE, enc_l != enc_r);
    sup_cond(c, C_ENC_FAULT_BOTH, !enc_l && !enc_r);

    /* obstacle distance: the supervisor's own sensor */
    c->us_ok = us;
    if (us && !s->us_no_echo) {
        c->dist_m = s->distance_m;
        c->obstacle_in_range = 1;
    } else {
        c->dist_m = P->us_max_m;
        c->obstacle_in_range = 0;
    }
    sup_cond(c, C_US_FAULT, !us);
    sup_cond(c, C_IMU_FAULT, !imu);
    sup_cond(c, C_ISLAND_TRIPPED, s->island_state != 0);
    sup_cond(c, C_ESTOP_BUTTON, s->estop != 0);

    yaw_vote(c, now, enc_l && enc_r, imu);
    brake_check(c, now);
}
