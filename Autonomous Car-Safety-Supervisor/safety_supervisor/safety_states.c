/*
 * safety_states.c - turns active conditions into a vehicle state.
 *
 *  - escalation is immediate, de-escalation needs AV_DEESCALATE_HOLD_MS of calm (hysteresis)
 *  - MRM / ESTOP end in SAFE once the vehicle stands still; SAFE is latched
 *  - SAFE -> RECOVERY after AV_RECOVERY_CLEAR_MS without faults and with a clear path
 *  - RECOVERY -> drive again only after the operator acknowledges (CLI "ack")
 *  - authority: NAV drives in NORMAL/CAUTION/DEGRADED, RECOVERY drives in MRM, nobody otherwise
 */
#include "safety_states.h"
#include "decision_core.h"

#define MS(x) ((uint64_t)(x) * 1000u)

static const uint8_t g_sev[C__COUNT] = {
    [C_PERC_UNSTABLE]        = SEV_CAUTION,
    [C_PHANTOM]              = SEV_CAUTION,
    [C_CAMERA_MISS]          = SEV_CAUTION,
    [C_PERC_STALE]           = SEV_CAUTION,
    [C_PERC_LOST]            = SEV_MRM,
    [C_NAV_SPEED_MISMATCH]   = SEV_CAUTION,
    [C_IMU_FAULT]            = SEV_DEGRADED,
    [C_ENC_FAULT_ONE]        = SEV_DEGRADED,
    [C_ENC_FAULT_BOTH]       = SEV_MRM,
    [C_US_FAULT]             = SEV_MRM,
    [C_YAW_IMU_SUSPECT]      = SEV_DEGRADED,
    [C_YAW_ENC_SUSPECT]      = SEV_DEGRADED,
    [C_ACTUATOR_FAULT]       = SEV_MRM,
    [C_MULTI_FAULT]          = SEV_MRM,
    [C_SENSOR_LINK_LOST]     = SEV_MRM,
    [C_OUTPUT_LINK_FAIL]     = SEV_MRM,
    [C_NAV_HB_LOST]          = SEV_MRM,
    [C_NAV_CRASHED]          = SEV_MRM,
    [C_NAV_UNTRUSTED]        = SEV_MRM,
    [C_BLIND_COLLISION_RISK] = SEV_MRM,
    [C_COLLISION_IMMINENT]   = SEV_ESTOP,
    [C_BRAKE_INEFFECTIVE]    = SEV_ESTOP,
    [C_ISLAND_TRIPPED]       = SEV_ESTOP,
    [C_ESTOP_BUTTON]         = SEV_ESTOP,
    [C_PATH_BLOCKED]         = SEV_NONE,      /* only blocks recovery */
};

uint8_t sm_target_severity(uint32_t conds)
{
    uint8_t sev = SEV_NONE;
    for (int i = 0; i < C__COUNT; i++)
        if ((conds & CBIT(i)) && g_sev[i] > sev)
            sev = g_sev[i];
    return sev;
}

static av_state_t driving_state(uint8_t sev)
{
    switch (sev) {
    case SEV_CAUTION:  return ST_CAUTION;
    case SEV_DEGRADED: return ST_DEGRADED;
    default:           return ST_NORMAL;
    }
}

static uint8_t authority_for(av_state_t s)
{
    switch (s) {
    case ST_NORMAL:
    case ST_CAUTION:
    case ST_DEGRADED: return AUTH_NAV;
    case ST_MRM:      return AUTH_RECOVERY;
    default:          return AUTH_NONE;
    }
}

static void transition(sup_ctx_t *c, av_state_t to, uint64_t now)
{
    av_state_t from = c->state;
    c->state = to;
    c->state_since_us = now;
    c->lower_since_us = 0;
    c->clear_since_us = 0;
    c->authority = authority_for(to);

    switch (to) {
    case ST_MRM:
        c->actions |= ACT_RECOV_START;
        c->mrm_v0 = c->v_true;
        c->mrm_t0 = now;
        c->brake_v0 = c->v_true;
        c->brake_t0 = now;
        c->recov_cmd_us = 0;
        break;
    case ST_ESTOP:
        c->actions |= ACT_RECOV_STOP;
        c->brake_v0 = c->v_true;
        c->brake_t0 = now;
        break;
    case ST_SAFE:
        c->actions |= ACT_RECOV_STOP;
        c->brake_t0 = 0;
        break;
    case ST_NORMAL:
    case ST_CAUTION:
    case ST_DEGRADED:
        if (from == ST_RECOVERY || from == ST_INIT) {
            c->applied_speed = 0.0f;               /* nav must start again from standstill */
            c->applied_steer = 0.0f;
        }
        break;
    default:
        break;
    }

    sup_emit(c, EV_STATE, (uint8_t)from, 0, 0.0f, 0.0f);
    if (sup_state_severity(to) > sup_state_severity(from) && from != ST_INIT)
        sup_reaction(c);
}

void sm_update(sup_ctx_t *c, uint64_t now)
{
    const av_profile_t *P = c->P;
    uint8_t sev = sm_target_severity(c->conds);
    int blocked = (c->conds & CBIT(C_PATH_BLOCKED)) != 0;

    /* standstill detection (time based fallback if the encoders are gone) */
    int stopped;
    if (c->v_valid)
        stopped = c->v_true < P->standstill_v;
    else
        stopped = now - c->state_since_us >
                  (uint64_t)((c->mrm_v0 / P->a_comf + 1.0f) * 1e6f);
    if (stopped) {
        if (!c->standstill_since_us)
            c->standstill_since_us = now;
    } else {
        c->standstill_since_us = 0;
    }
    int still = c->standstill_since_us && now - c->standstill_since_us >= MS(AV_STANDSTILL_HOLD_MS);

    av_state_t s = c->state, next = s;
    switch (s) {
    case ST_INIT:
        if (c->sens_seen)
            next = sev >= SEV_MRM ? ST_SAFE : driving_state(sev);
        break;

    case ST_NORMAL:
    case ST_CAUTION:
    case ST_DEGRADED: {
        int cur = sup_state_severity(s);
        if (sev >= SEV_ESTOP) {
            next = ST_ESTOP;
        } else if (sev == SEV_MRM) {
            next = ST_MRM;
        } else if (sev > cur) {
            next = driving_state(sev);
        } else if (sev < cur) {
            if (!c->lower_since_us)
                c->lower_since_us = now;
            else if (now - c->lower_since_us >= MS(AV_DEESCALATE_HOLD_MS))
                next = driving_state(sev);
        } else {
            c->lower_since_us = 0;
        }
        break;
    }

    case ST_MRM:
        if (sev >= SEV_ESTOP)
            next = ST_ESTOP;
        else if (still)
            next = ST_SAFE;
        break;

    case ST_ESTOP:
        if (still)
            next = ST_SAFE;
        break;

    case ST_SAFE:
        if (sev <= SEV_DEGRADED && !blocked) {
            if (!c->clear_since_us)
                c->clear_since_us = now;
            else if (now - c->clear_since_us >= MS(AV_RECOVERY_CLEAR_MS))
                next = ST_RECOVERY;
        } else {
            c->clear_since_us = 0;
        }
        break;

    case ST_RECOVERY:
        if (sev >= SEV_MRM || blocked)
            next = ST_SAFE;
        else if (c->ack_pending)
            next = driving_state(sev);
        break;

    default:
        break;
    }
    c->ack_pending = 0;

    if (next != s)
        transition(c, next, now);
}
