/*
 * final_command.h - chooses the ONE command that goes to the vehicle this cycle.
 */
#ifndef SUP_ARBITER_H
#define SUP_ARBITER_H

#include "supervisor_data.h"

void arb_compute(sup_ctx_t *c, uint64_t now_us);

#endif /* SUP_ARBITER_H */
