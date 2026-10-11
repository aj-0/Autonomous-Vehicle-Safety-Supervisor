/*
 * check_nav_command.c - is the navigation command physically possible and sane?
 *
 * Verdicts: PASS (unchanged), CLAMP (minimal correction), REJECT (dangerous / impossible).
 * Order: authority -> corruption -> range -> slew rate -> steering envelope ->
 *        swerve lock-out -> state speed cap -> collision guard.
 */
#include <math.h>
#include <string.h>
#include "checks.h"
#include "decision_core.h"
#include "vehicle_physics.h"

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static float state_speed_cap(const sup_ctx_t *c)
{
    if (c->state == ST_CAUTION)
        return c->P->v_cruise * c->P->caution_frac;
    if (c->state == ST_DEGRADED)
        return c->P->v_cruise * c->P->degraded_frac;
    return c->P->v_max;
}

static int camera_sees_obstacle(const sup_ctx_t *c, const av_nav_cmd_t *cmd, uint64_t now)
{
    int fresh = c->perc_seen && now - c->perc_fresh_us <= (uint64_t)AV_PERC_STALE_MS * 1000u;
    return (fresh && c->perc.obstacle && c->perc.zone == ZONE_CENTER) || cmd->nav_obstacle;
}

void chk_nav_command(sup_ctx_t *c, const av_nav_cmd_t *cmd, uint64_t now, av_verdict_t *v)
{
    const av_profile_t *P = c->P;
    uint32_t rej = 0, clamp = 0;
    float speed = cmd->target_speed;
    float steer = cmd->steer_deg;

    float dt = c->nav_cmd_us ? (float)(now - c->nav_cmd_us) * 1e-6f : AV_NAV_PERIOD_MS * 1e-3f;
    dt = clampf(dt, 0.01f, 0.2f);
    c->nav_cmd_us = now;
    c->nav_last = *cmd;

    memset(v, 0, sizeof *v);
    v->seq = cmd->seq;

    /* nav's speed belief vs the encoders (independent of authority) */
    if (c->v_valid) {
        c->speed_mismatch_n = fabsf(cmd->nav_speed_est - c->v_true) > P->speed_mismatch
                                  ? c->speed_mismatch_n + 1 : 0;
        sup_cond(c, C_NAV_SPEED_MISMATCH, c->speed_mismatch_n >= AV_SPEED_MISMATCH_N);
    }

    if (c->authority != AUTH_NAV) {
        v->verdict = V_REJECT;
        v->reasons = R_NO_AUTHORITY;
        v->out_speed = c->out.speed_sp;
        v->out_steer = c->out.steer_deg;
        return;
    }

    if (!isfinite(speed) || !isfinite(steer) || speed < -0.01f || speed > 2.0f * P->v_max ||
        fabsf(steer) > 2.0f * P->steer_max_deg) {
        rej |= R_CORRUPT;                               /* garbage, e.g. steer = 999 deg */
    } else {
        /* range */
        if (speed < 0.0f)
            speed = 0.0f;
        if (speed > P->v_max) {
            speed = P->v_max;
            clamp |= R_RANGE;
        }
        if (fabsf(steer) > P->steer_max_deg) {
            steer = copysignf(P->steer_max_deg, steer);
            clamp |= R_RANGE;
        }

        /* slew rate: steering cannot jump faster than the actuator can move */
        float max_d = P->steer_rate_dps * dt;
        float d = steer - c->applied_steer;
        if (fabsf(d) > 3.0f * max_d + 1.0f) {
            rej |= R_SLEW;                              /* physically impossible: glitch */
        } else if (fabsf(d) > max_d) {
            steer = c->applied_steer + copysignf(max_d, d);
            clamp |= R_SLEW;
        }

        /* steering envelope: lateral acceleration limit at the current speed */
        float v_env = fmaxf(c->v_valid ? c->v_true : 0.0f, speed);
        float dmax = physics_delta_max_deg(v_env, P->wheelbase_m, P->a_lat_max, P->steer_max_deg);
        if (fabsf(steer) > dmax) {
            steer = copysignf(dmax, steer);
            clamp |= R_ENVELOPE;
        }

        /* unstable or phantom perception: no evasive swerves */
        if ((c->conds & (CBIT(C_PERC_UNSTABLE) | CBIT(C_PHANTOM))) &&
            fabsf(steer) > AV_SWERVE_LOCK_DEG) {
            steer = copysignf(AV_SWERVE_LOCK_DEG, steer);
            clamp |= R_SWERVE_LOCK;
        }

        /* state speed cap */
        float cap = state_speed_cap(c);
        if (speed > cap) {
            speed = cap;
            clamp |= R_STATE_CAP;
        }

        /* collision guard */
        float va = chk_allowed_speed(c, P->a_comf);
        if (speed > va + P->speed_tol) {
            if (camera_sees_obstacle(c, cmd, now)) {
                speed = va;                             /* nav brakes, just too late: help it */
                clamp |= R_COLLISION_CAP;
            } else {
                rej |= R_CAMERA_MISS;                   /* nav drives blind at an obstacle */
                sup_cond(c, C_BLIND_COLLISION_RISK, 1);
            }
        } else if (speed > va) {
            speed = va;
            clamp |= R_COLLISION_CAP;
        }
    }

    if (rej) {
        v->verdict = V_REJECT;
        v->reasons = rej | clamp;
        v->out_speed = c->applied_speed;                /* last safe command stays active */
        v->out_steer = c->applied_steer;
        c->reject_t[c->reject_idx % SUP_HIST] = now;
        c->reject_idx++;
    } else {
        v->verdict = clamp ? V_CLAMP : V_PASS;
        v->reasons = clamp;
        c->applied_speed = speed;
        c->applied_steer = steer;
        v->out_speed = speed;
        v->out_steer = steer;
    }
}

void chk_recovery_command(sup_ctx_t *c, const av_recovery_cmd_t *cmd, uint64_t now,
                          av_verdict_t *v)
{
    memset(v, 0, sizeof *v);
    v->seq = cmd->seq;
    if (c->authority != AUTH_RECOVERY) {
        v->verdict = V_REJECT;
        v->reasons = R_NO_AUTHORITY;
        return;
    }
    c->recov_speed = clampf(isfinite(cmd->speed) ? cmd->speed : 0.0f, 0.0f, c->P->v_max);
    c->recov_steer = clampf(isfinite(cmd->steer_deg) ? cmd->steer_deg : 0.0f,
                            -AV_SWERVE_LOCK_DEG, AV_SWERVE_LOCK_DEG);
    c->recov_cmd_us = now;
    v->verdict = V_PASS;
    v->out_speed = c->recov_speed;
    v->out_steer = c->recov_steer;
}
