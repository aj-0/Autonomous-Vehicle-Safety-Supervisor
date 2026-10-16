/*
 * safety_deadline.c - safe-state deadline timer.
 *
 *  start : the state machine enters MRM or ESTOP (a stop begins)
 *          deadline = REACT + MARGIN * v0 / a + STANDSTILL_HOLD   (see settings.h)
 *          MRM -> ESTOP during a stop can only make the deadline shorter, never longer
 *  check : every 10 ms timer tick; past the deadline and not yet SAFE ->
 *          condition STOP_DEADLINE_MISSED (severity ESTOP: maximum braking, motor relay cut)
 *  end   : the vehicle reaches SAFE -> PASS (in time) or FAIL (late), time used is logged
 */
#include "safety_deadline.h"
#include "decision_core.h"

float dl_budget_ms(float v0, float a)
{
    if (v0 < 0.0f)
        v0 = 0.0f;
    return (float)AV_DEADLINE_REACT_MS + AV_DEADLINE_MARGIN * 1000.0f * v0 / a +
           (float)AV_STANDSTILL_HOLD_MS;
}

static float ms_since(uint64_t t0, uint64_t now)
{
    return now > t0 ? (float)(now - t0) * 1e-3f : 0.0f;
}

void dl_on_state(sup_ctx_t *c, av_state_t from, av_state_t to, uint64_t now)
{
    (void)from;
    if (to == ST_MRM || to == ST_ESTOP) {
        float a = to == ST_ESTOP ? c->P->a_max : c->P->a_comf;
        float budget = dl_budget_ms(c->v_true, a);
        uint64_t end = now + (uint64_t)(budget * 1000.0f);
        if (!c->dl_active) {
            c->dl_active = 1;
            c->dl_missed = 0;
            c->dl_t0_us = now;
            c->dl_end_us = end;
        } else if (end < c->dl_end_us) {
            c->dl_end_us = end;                       /* harder braking: earlier deadline */
        } else {
            return;
        }
        c->dl_budget_ms = (float)(c->dl_end_us - c->dl_t0_us) * 1e-3f;
        c->dl_last_ms = ms_since(c->dl_t0_us, now);
        sup_emit(c, EV_DEADLINE, (uint8_t)from, DL_START, c->dl_budget_ms, a);
        return;
    }

    if (!c->dl_active)
        return;
    if (to == ST_SAFE) {
        float used = ms_since(c->dl_t0_us, now);
        int pass = !c->dl_missed && now <= c->dl_end_us;
        float margin = c->dl_budget_ms - used;
        if (pass)
            c->dl_pass_n++;
        else
            c->dl_fail_n++;
        if (used > c->dl_worst_ms)
            c->dl_worst_ms = used;
        if (c->dl_pass_n + c->dl_fail_n == 1 || margin < c->dl_min_margin_ms)
            c->dl_min_margin_ms = margin;
        c->dl_last_ms = used;
        sup_emit(c, EV_DEADLINE, (uint8_t)from, pass ? DL_PASS : DL_FAIL, used, c->dl_budget_ms);
    }
    /* SAFE (or any non-stopping state): the stop is over */
    c->dl_active = 0;
    c->dl_missed = 0;
    sup_cond(c, C_DEADLINE_MISSED, 0);
}

void dl_tick(sup_ctx_t *c, uint64_t now)
{
    if (!c->dl_active)
        return;
    c->dl_last_ms = ms_since(c->dl_t0_us, now);
    if (!c->dl_missed && now > c->dl_end_us) {
        c->dl_missed = 1;
        sup_cond(c, C_DEADLINE_MISSED, 1);
        sup_emit(c, EV_DEADLINE, (uint8_t)c->state, DL_MISSED, c->dl_last_ms, c->dl_budget_ms);
    }
}
