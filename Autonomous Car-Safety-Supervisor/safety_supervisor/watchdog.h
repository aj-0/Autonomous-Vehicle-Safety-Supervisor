/*
 * watchdog.h - time-based monitoring run on every supervisor tick.
 */
#ifndef SUP_WATCHDOG_H
#define SUP_WATCHDOG_H

#include "supervisor_data.h"

void wd_tick(sup_ctx_t *c, uint64_t now_us);
int  wd_rejects_in_window(const sup_ctx_t *c, uint64_t now_us);

#endif /* SUP_WATCHDOG_H */
