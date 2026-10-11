/*
 * decision_core.h - platform-independent decision logic of the safety supervisor.
 *
 * supervisor_main.c (QNX) receives messages and pulses and calls these functions; they update
 * the context, decide verdicts/state and fill ctx->out + ctx->actions.
 */
#ifndef DECISION_CORE_H
#define DECISION_CORE_H

#include "supervisor_data.h"

void sup_core_init(sup_ctx_t *c, const av_profile_t *P, int bench_mode,
                   sup_emit_fn emit, void *user, uint64_t now_us);

void sup_core_on_sensor(sup_ctx_t *c, const av_sensor_state_t *s, uint64_t now_us);
void sup_core_on_perception(sup_ctx_t *c, const av_perception_t *p, uint64_t now_us);
void sup_core_on_nav_cmd(sup_ctx_t *c, const av_nav_cmd_t *cmd, uint64_t now_us,
                         av_verdict_t *verdict);
void sup_core_on_recovery_cmd(sup_ctx_t *c, const av_recovery_cmd_t *cmd, uint64_t now_us,
                              av_verdict_t *verdict);
void sup_core_on_nav_heartbeat(sup_ctx_t *c, uint64_t now_us);
void sup_core_on_nav_connect(sup_ctx_t *c, int32_t pid, uint64_t now_us);
void sup_core_on_nav_disconnect(sup_ctx_t *c, uint64_t now_us);
int  sup_core_on_ack(sup_ctx_t *c, uint64_t now_us);          /* 1 accepted, 0 not ready */
void sup_core_on_inject_mark(sup_ctx_t *c, uint32_t fault, uint64_t now_us);
void sup_core_tick(sup_ctx_t *c, uint64_t now_us);
void sup_core_on_output_result(sup_ctx_t *c, int ok);
void sup_core_fill_status(const sup_ctx_t *c, av_status_t *st);

/* ---- helpers used by the check modules ---- */
void sup_cond(sup_ctx_t *c, int cond, int active);
void sup_emit(sup_ctx_t *c, uint16_t kind, uint8_t prev_state, uint32_t aux, float x1, float x2);
void sup_reaction(sup_ctx_t *c);     /* first reaction after an injected fault */
int  sup_state_severity(av_state_t s);

#endif /* DECISION_CORE_H */
