/*
 * safety_states.h - graded response: NORMAL -> CAUTION -> DEGRADED -> MRM -> ESTOP -> SAFE -> RECOVERY
 */
#ifndef SUP_STATE_MACHINE_H
#define SUP_STATE_MACHINE_H

#include "supervisor_data.h"

enum { SEV_NONE = 0, SEV_CAUTION, SEV_DEGRADED, SEV_MRM, SEV_ESTOP };

uint8_t sm_target_severity(uint32_t conds);
void    sm_update(sup_ctx_t *c, uint64_t now_us);

#endif /* SUP_STATE_MACHINE_H */
