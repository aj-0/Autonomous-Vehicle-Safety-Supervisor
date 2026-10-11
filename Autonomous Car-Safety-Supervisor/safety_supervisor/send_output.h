/*
 * send_output.h - connections FROM the supervisor to other processes.
 *
 * Deadlock rule: the supervisor only MsgSend()s "downwards" to the vehicle link.
 * Everybody who sends to the supervisor (nav, recovery, sensors) is only notified
 * with non-blocking pulses, so a send-cycle (A waits for B waits for A) cannot happen.
 */
#ifndef SUP_OUTPUT_H
#define SUP_OUTPUT_H

#include <stdint.h>
#include "messages.h"

typedef struct {
    int      link_coid;       /* av_vehicle_link: vehicle_link / vehicle_simulator */
    int      recov_coid;      /* av_recovery */
    int      log_coid;        /* av_logger */
    uint64_t next_try_us;
} sup_output_t;

void out_init(sup_output_t *o);
void out_reconnect(sup_output_t *o, uint64_t now_us);          /* lazy, at most once per second */
int  out_send_vehicle(sup_output_t *o, const av_vehicle_cmd_t *cmd);   /* 1 ok, 0 failed */
void out_pulse_recovery(sup_output_t *o, int code, int value);
void out_pulse_logger(sup_output_t *o);

#endif /* SUP_OUTPUT_H */
