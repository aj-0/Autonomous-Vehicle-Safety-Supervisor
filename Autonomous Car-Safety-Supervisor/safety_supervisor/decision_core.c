/*
 * decision_core.c - orchestration of the supervisor decision logic (platform independent).
 *
 * Event-driven fast path : a new sensor frame / perception packet / nav command is checked
 *                          immediately; if the state escalates, ACT_OUTPUT_NOW asks supervisor_main.c
 *                          to send the vehicle command right away instead of waiting a tick.
 * Periodic path (10 ms)  : watchdogs, state machine, arbitration, output, cycle log.
 */
#include <string.h>
#include "decision_core.h"
#include "checks.h"
#include "watchdog.h"
#include "safety_states.h"
#include "safety_deadline.h"
#include "final_command.h"

/* ------------------------------------------------------------------ */
/* helpers                                                             */
/* ------------------------------------------------------------------ */

int sup_state_severity(av_state_t s)
{
    switch (s) {
    case ST_NORMAL:   return 0;
    case ST_CAUTION:  return 1;
    case ST_DEGRADED: return 2;
    case ST_MRM:      return 3;
    case ST_ESTOP:    return 4;
    default:          return 5;      /* INIT, SAFE, RECOVERY: vehicle held */
    }
}

void sup_cond(sup_ctx_t *c, int cond, int active)
{
    if (active)
        c->conds |= CBIT(cond);
    else
        c->conds &= ~CBIT(cond);
}

void sup_emit(sup_ctx_t *c, uint16_t kind, uint8_t prev_state, uint32_t aux, float x1, float x2)
{
    av_event_t e;
    memset(&e, 0, sizeof e);
    e.t_us = c->now_us;
    e.seq = ++c->ev_seq;
    e.kind = kind;
    e.state = (uint8_t)c->state;
    e.prev_state = prev_state;
    e.verdict = c->last_verdict.verdict;
    e.authority = c->authority;
    e.reasons = c->last_verdict.reasons;
    e.conds = c->conds;
    e.aux = aux;
    e.v = c->v_true;
    e.dist = c->obstacle_in_range ? c->dist_m : -1.0f;
    e.gyro = c->sens.gyro_z_dps;
    e.cmd_speed = c->nav_last.target_speed;
    e.cmd_steer = c->nav_last.steer_deg;
    e.out_speed = c->out.speed_sp;
    e.out_steer = c->out.steer_deg;
    e.x1 = x1;
    e.x2 = x2;
    if (c->emit)
        c->emit(c->emit_user, &e);
    if (kind != EV_CYCLE)
        c->actions |= ACT_LOG_KICK;
}

void sup_reaction(sup_ctx_t *c)
{
    if (!c->inject_t_us || c->now_us < c->inject_t_us)
        return;
    float ms = (float)(c->now_us - c->inject_t_us) * 1e-3f;
    stats_inject(&c->stats, ms);
    sup_emit(c, EV_REACTION, (uint8_t)c->state, c->inject_fault, ms, 0.0f);
    c->inject_t_us = 0;
}

/* log condition changes; a newly set condition counts as a reaction to an injected fault */
static void sync_conds(sup_ctx_t *c)
{
    uint32_t changed = c->conds ^ c->conds_logged;
    if (!changed)
        return;
    uint32_t set = changed & c->conds;
    uint32_t cleared = changed & ~c->conds;
    if (set) {
        sup_emit(c, EV_COND, (uint8_t)c->state, set, 1.0f, 0.0f);
        if (set & ~CBIT(C_PATH_BLOCKED))
            sup_reaction(c);
    }
    if (cleared)
        sup_emit(c, EV_COND, (uint8_t)c->state, cleared, 0.0f, 0.0f);
    c->conds_logged = c->conds;
}

static void evaluate(sup_ctx_t *c, uint64_t now, int fast_path)
{
    int sev_before = sup_state_severity(c->state);
    av_state_t before = c->state;
    sync_conds(c);
    sm_update(c, now);
    arb_compute(c, now);
    if (fast_path && c->state != before && sup_state_severity(c->state) > sev_before)
        c->actions |= ACT_OUTPUT_NOW;
}

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */

void sup_core_init(sup_ctx_t *c, const av_profile_t *P, int bench_mode,
                   sup_emit_fn emit, void *user, uint64_t now)
{
    memset(c, 0, sizeof *c);
    c->P = P;
    c->bench_mode = bench_mode;
    c->now_us = c->start_us = now;
    c->state = ST_INIT;
    c->state_since_us = now;
    c->authority = AUTH_NONE;
    c->out.hdr.type = AV_MSG_VEHICLE_CMD;
    c->out.hdr.version = AV_PROTO_VERSION;
    c->out.brake_mode = BRAKE_MAX;
    c->out.motor_enable = 0;
    c->emit = emit;
    c->emit_user = user;
    stats_init(&c->stats);
}

void sup_core_on_sensor(sup_ctx_t *c, const av_sensor_state_t *s, uint64_t now)
{
    c->now_us = now;
    c->sens_prev_rx_us = c->sens_rx_us;
    c->sens = *s;
    c->sens_rx_us = now;
    c->sens_seen = 1;
    /* physical ACK button = the operator's "ack" (only acts in RECOVERY, like the CLI) */
    if (s->ack_button && !c->ack_btn_prev && c->state == ST_RECOVERY)
        c->ack_pending = 1;
    c->ack_btn_prev = s->ack_button;
    chk_input_on_sensor(c, now);
    chk_situation_on_sensor(c);
    evaluate(c, now, 1);
}

void sup_core_on_perception(sup_ctx_t *c, const av_perception_t *p, uint64_t now)
{
    c->now_us = now;
    int fresh = !c->perc_seen || p->seq != c->perc.seq;   /* frozen camera repeats its seq */
    c->perc = *p;
    c->perc_seen = 1;
    /* capture time is only comparable when the gateway stamps it in our clock domain */
    c->perc_latency_us = (p->t_capture_us && p->t_capture_us <= now &&
                          now - p->t_capture_us < 10000000u) ? now - p->t_capture_us : 0;
    if (fresh) {
        c->perc_fresh_us = now;
        chk_perception(c, now);
    }
    evaluate(c, now, 1);
}

void sup_core_on_nav_cmd(sup_ctx_t *c, const av_nav_cmd_t *cmd, uint64_t now, av_verdict_t *v)
{
    c->now_us = now;
    chk_nav_command(c, cmd, now, v);
    c->last_verdict = *v;

    if (v->verdict == V_PASS)
        c->pass_n++;
    else if (v->verdict == V_CLAMP)
        c->clamp_n++;
    else if (v->reasons != R_NO_AUTHORITY)
        c->reject_n++;

    if (v->verdict != V_PASS && v->reasons != R_NO_AUTHORITY) {
        sup_emit(c, EV_VERDICT, (uint8_t)c->state, cmd->seq, v->out_speed, v->out_steer);
        sup_reaction(c);
    }
    evaluate(c, now, 1);
    v->state = (uint8_t)c->state;
    v->authority = c->authority;
}

void sup_core_on_recovery_cmd(sup_ctx_t *c, const av_recovery_cmd_t *cmd, uint64_t now,
                              av_verdict_t *v)
{
    c->now_us = now;
    chk_recovery_command(c, cmd, now, v);
    evaluate(c, now, 1);
    v->state = (uint8_t)c->state;
    v->authority = c->authority;
}

void sup_core_on_nav_heartbeat(sup_ctx_t *c, uint64_t now)
{
    c->nav_hb_us = now;
}

void sup_core_on_nav_connect(sup_ctx_t *c, int32_t pid, uint64_t now)
{
    c->now_us = now;
    c->nav_connected = 1;
    c->nav_ever = 1;
    c->nav_pid = pid;
    c->nav_hb_us = now;
    c->nav_cmd_us = now;
    c->restart_requested = 0;
    sup_cond(c, C_NAV_CRASHED, 0);
    sup_emit(c, EV_CLIENT, (uint8_t)c->state, (uint32_t)pid, 1.0f, 0.0f);
}

void sup_core_on_nav_disconnect(sup_ctx_t *c, uint64_t now)
{
    c->now_us = now;
    c->nav_connected = 0;
    sup_cond(c, C_NAV_CRASHED, 1);
    sup_emit(c, EV_CLIENT, (uint8_t)c->state, (uint32_t)c->nav_pid, 0.0f, 0.0f);
    if (!c->restart_requested) {
        c->actions |= ACT_RESTART_NAV;
        c->restart_requested = 1;
    }
    evaluate(c, now, 1);
}

int sup_core_on_ack(sup_ctx_t *c, uint64_t now)
{
    c->now_us = now;
    if (c->state != ST_RECOVERY)
        return 0;
    c->ack_pending = 1;
    evaluate(c, now, 0);
    return 1;
}

void sup_core_on_inject_mark(sup_ctx_t *c, uint32_t fault, uint64_t now)
{
    c->now_us = now;
    c->inject_t_us = now;
    c->inject_fault = fault;
    c->last_inject_fault = fault;
    sup_emit(c, EV_INJECT, (uint8_t)c->state, fault, 0.0f, 0.0f);
}

void sup_core_tick(sup_ctx_t *c, uint64_t now)
{
    c->now_us = now;
    wd_tick(c, now);
    dl_tick(c, now);
    if (c->inject_t_us && now - c->inject_t_us > (uint64_t)AV_INJECT_WINDOW_MS * 1000u)
        c->inject_t_us = 0;                         /* no reaction: fault had no effect */
    evaluate(c, now, 0);
    sup_emit(c, EV_CYCLE, (uint8_t)c->state, 0, 0.0f, 0.0f);
}

void sup_core_on_output_result(sup_ctx_t *c, int ok)
{
    c->output_fail_n = ok ? 0 : c->output_fail_n + 1;
}

void sup_core_fill_status(const sup_ctx_t *c, av_status_t *st)
{
    uint64_t now = c->now_us;
    memset(st, 0, sizeof *st);
    st->state = (uint8_t)c->state;
    st->authority = c->authority;
    st->last_verdict = c->last_verdict.verdict;
    st->conds = c->conds;
    st->last_reasons = c->last_verdict.reasons;
    strncpy(st->profile, c->P->name, sizeof st->profile - 1);
    st->v_true = c->v_true;
    st->dist_m = c->dist_m;
    st->gyro_dps = c->sens.gyro_z_dps;
    st->out_speed = c->out.speed_sp;
    st->out_steer = c->out.steer_deg;
    st->nav_speed = c->nav_last.target_speed;
    st->nav_steer = c->nav_last.steer_deg;
    for (int i = 0; i < SENS_COUNT; i++)
        st->health[i] = c->sens_seen ? c->sens.health[i] : H_UNKNOWN;
    st->obstacle_in_range = (uint8_t)c->obstacle_in_range;
    st->island_tripped = c->sens.island_state;
    st->estop = c->sens.estop;
    st->bench = (uint8_t)c->bench_mode;
    st->perc_seen = (uint8_t)c->perc_seen;
    st->perc_obstacle = c->perc.obstacle;
    st->perc_zone = c->perc.zone;
    st->perc_unstable = (uint8_t)c->perc_unstable;
    st->perc_age_ms = c->perc_seen ? (uint32_t)((now - c->perc_fresh_us) / 1000u) : 0;
    st->nav_connected = (uint8_t)c->nav_connected;
    st->nav_pid = c->nav_pid;
    st->nav_hb_age_ms = c->nav_ever ? (uint32_t)((now - c->nav_hb_us) / 1000u) : 0;
    st->nav_rejects_window = (uint32_t)wd_rejects_in_window(c, now);
    st->pass_count = c->pass_n;
    st->clamp_count = c->clamp_n;
    st->reject_count = c->reject_n;
    st->ticks = c->stats.ticks;
    st->overruns = c->stats.overruns;
    st->output_fails = (uint32_t)c->output_fail_n;
    st->cycle_us_avg = stats_cycle_avg(&c->stats);
    st->cycle_us_max = c->stats.cycle_max_us;
    st->jitter_us_max = c->stats.jitter_max_us;
    st->decision_us_last = c->stats.decision_last_us;
    st->decision_us_max = c->stats.decision_max_us;
    st->out_latency_us_last = c->stats.out_lat_last_us;
    st->out_latency_us_max = c->stats.out_lat_max_us;
    st->inject_ms_last = c->stats.inj_last_ms;
    st->inject_ms_min = c->stats.inj_min_ms;
    st->inject_ms_max = c->stats.inj_max_ms;
    st->inject_count = c->stats.inj_n;
    st->last_inject_fault = c->last_inject_fault;
    st->uptime_ms = (uint32_t)((now - c->start_us) / 1000u);
    st->dl_active = (uint8_t)c->dl_active;
    st->dl_budget_ms = c->dl_budget_ms;
    st->dl_elapsed_ms = c->dl_last_ms;
    st->dl_pass = c->dl_pass_n;
    st->dl_fail = c->dl_fail_n;
    st->dl_worst_ms = c->dl_worst_ms;
    st->dl_min_margin_ms = c->dl_min_margin_ms;
}
