/*
 * final_command.c - output arbitration.
 *
 *  authority NAV      : last nav command that passed the checks, limited by the state cap
 *  authority RECOVERY : recovery manager's ramp, bounded by the supervisor's own ramp
 *                       (the vehicle still stops if the recovery manager dies)
 *  authority NONE     : zero speed; ESTOP brakes at maximum, SAFE also opens the motor relay
 *
 * The collision cap is applied last, to every authority, on every cycle: the world can change
 * between two navigation commands (nav runs at 20 Hz, the supervisor at 100 Hz).
 */
#include <math.h>
#include "final_command.h"
#include "checks.h"

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void arb_compute(sup_ctx_t *c, uint64_t now)
{
    const av_profile_t *P = c->P;
    float speed = 0.0f, steer = 0.0f;
    uint8_t brake = BRAKE_NORMAL, enable = 1;

    switch (c->authority) {
    case AUTH_NAV:
        speed = c->applied_speed;
        steer = c->applied_steer;
        if (c->state == ST_CAUTION)
            speed = fminf(speed, P->v_cruise * P->caution_frac);
        else if (c->state == ST_DEGRADED)
            speed = fminf(speed, P->v_cruise * P->degraded_frac);
        if (c->conds & (CBIT(C_PERC_UNSTABLE) | CBIT(C_PHANTOM)))
            steer = clampf(steer, -AV_SWERVE_LOCK_DEG, AV_SWERVE_LOCK_DEG);
        break;

    case AUTH_RECOVERY: {
        float t = (float)(now - c->mrm_t0) * 1e-6f;
        float ramp = fmaxf(0.0f, c->mrm_v0 - P->a_comf * t);
        int fresh = c->recov_cmd_us &&
                    now - c->recov_cmd_us <= (uint64_t)AV_RECOV_CMD_TIMEOUT_MS * 1000u;
        speed = fresh ? fminf(c->recov_speed, ramp) : ramp;
        steer = fresh ? c->recov_steer : 0.0f;
        brake = BRAKE_COMFORT;
        break;
    }

    default:
        speed = 0.0f;
        steer = 0.0f;
        brake = BRAKE_MAX;
        /* ESTOP keeps the motor driver powered for active braking, unless braking itself failed */
        enable = (c->state == ST_ESTOP && !(c->conds & CBIT(C_BRAKE_INEFFECTIVE))) ? 1 : 0;
        break;
    }

    if (c->obstacle_in_range)
        speed = fminf(speed, chk_allowed_speed(c, P->a_comf));

    c->out.speed_sp = clampf(speed, 0.0f, P->v_max);
    c->out.steer_deg = clampf(steer, -P->steer_max_deg, P->steer_max_deg);
    c->out.brake_mode = brake;
    c->out.motor_enable = enable;
    c->out.state = (uint8_t)c->state;
}
