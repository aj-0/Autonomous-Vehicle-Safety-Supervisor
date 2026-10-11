/*
 * qnx_helpers.h - QNX real-time helpers shared by every process:
 * priorities, memory locking, CPU pinning, named channels, periodic pulse timers,
 * MsgSend with timeout and precise time stamps.
 */
#ifndef QNX_HELPERS_H
#define QNX_HELPERS_H

#include <stdint.h>
#include <stddef.h>
#include <time.h>
#include <sys/dispatch.h>

typedef struct {
    int            chid;
    int            fixed_priority;   /* 1 if _NTO_CHF_FIXED_PRIORITY is active */
    dispatch_t    *dpp;
    name_attach_t *attach;
} qnx_server_t;

uint64_t qnx_now_us(void);                 /* CLOCK_MONOTONIC, system wide            */
uint64_t qnx_cycles(void);                 /* ClockCycles(): CPU free-running counter */
double   qnx_cycles_to_us(uint64_t cycles);

int qnx_set_priority(int prio);            /* SCHED_FIFO for the calling thread       */
int qnx_lock_memory(void);                 /* mlockall: no page faults at run time     */
int qnx_pin_to_core(int core);             /* run mask, core < 0 = no pinning          */

/* Create a channel registered under `name`.
 * The channel always delivers _PULSE_CODE_DISCONNECT when a client process goes away.
 * fixed_priority = 1: messages do NOT change the receiving thread's priority
 * (no priority inheritance), so a priority-10 client cannot slow the server down. */
int  qnx_server_create(qnx_server_t *srv, const char *name, int fixed_priority);
void qnx_server_destroy(qnx_server_t *srv);

/* Periodic timer that delivers a pulse with `code` to channel `chid`. */
int qnx_pulse_timer(int chid, int prio, int code, uint32_t period_us, timer_t *out);

/* name_open with retries (delay_ms between tries). Returns coid or -1. */
int qnx_name_open_retry(const char *name, int tries, int delay_ms);

/* MsgSend that gives up after timeout_us (send + reply phase). Returns MsgSend status or -1. */
long qnx_send_timeout(int coid, const void *smsg, size_t sbytes, void *rmsg, size_t rbytes,
                     uint32_t timeout_us);

/* Send AV_MSG_HELLO claiming `roles`. Returns 0 or the errno from the server (e.g. EBUSY). */
int qnx_hello(int coid, uint32_t roles, const char *name);

/* Handle QNX system messages that arrive on a name_attach channel.
 * Returns 1 if the message was consumed (caller must continue to the next MsgReceive). */
int qnx_handle_system_msg(int rcvid, const void *msg);

#endif /* QNX_HELPERS_H */
