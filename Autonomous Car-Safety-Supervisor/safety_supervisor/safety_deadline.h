/*
 * safety_deadline.h - the "defined deadline" of the problem statement: once the supervisor
 * decides to stop the vehicle, it must be standing still (state SAFE) in time.
 */
#ifndef SAFETY_DEADLINE_H
#define SAFETY_DEADLINE_H

#include "supervisor_data.h"

/* deadline in ms for a stop from v0 (m/s) with deceleration a (m/s^2) */
float dl_budget_ms(float v0, float a);

/* called by the state machine on every state change */
void dl_on_state(sup_ctx_t *c, av_state_t from, av_state_t to, uint64_t now_us);

/* called every 10 ms by the supervisor timer: detects a stop that takes too long */
void dl_tick(sup_ctx_t *c, uint64_t now_us);

#endif /* SAFETY_DEADLINE_H */
