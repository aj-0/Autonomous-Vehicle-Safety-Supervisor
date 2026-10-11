/*
 * send_output.c - supervisor -> vehicle link (MsgSend with timeout), recovery + logger (pulses).
 */
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>

#include "send_output.h"
#include "qnx_helpers.h"
#include "settings.h"

void out_init(sup_output_t *o)
{
    o->link_coid = o->recov_coid = o->log_coid = -1;
    o->next_try_us = 0;
}

static void close_coid(int *coid)
{
    if (*coid != -1)
        name_close(*coid);
    *coid = -1;
}

void out_reconnect(sup_output_t *o, uint64_t now)
{
    if (now < o->next_try_us)
        return;
    if (o->link_coid != -1 && o->recov_coid != -1 && o->log_coid != -1)
        return;
    o->next_try_us = now + 1000000u;

    if (o->link_coid == -1) {
        o->link_coid = name_open(AV_NAME_LINK, 0);
        if (o->link_coid != -1) {
            int err = qnx_hello(o->link_coid, AV_ROLE_SUPERVISOR, "supervisor");
            if (err) {
                fprintf(stderr, "[SUPERVISOR] link refused SUPERVISOR role: %s\n", strerror(err));
                close_coid(&o->link_coid);
            } else {
                printf("[SUPERVISOR] connected to %s\n", AV_NAME_LINK);
            }
        }
    }
    if (o->recov_coid == -1 && (o->recov_coid = name_open(AV_NAME_RECOVERY, 0)) != -1)
        printf("[SUPERVISOR] connected to %s\n", AV_NAME_RECOVERY);
    if (o->log_coid == -1)
        o->log_coid = name_open(AV_NAME_LOGGER, 0);
}

int out_send_vehicle(sup_output_t *o, const av_vehicle_cmd_t *cmd)
{
    if (o->link_coid == -1)
        return 0;
    if (qnx_send_timeout(o->link_coid, cmd, sizeof *cmd, NULL, 0, AV_OUTPUT_SEND_TIMEOUT_US) == -1) {
        if (errno == EBADF || errno == ESRCH || errno == ENOTCONN || errno == EPERM)
            close_coid(&o->link_coid);
        return 0;
    }
    return 1;
}

/* The pulse carries an explicit priority: the receiver runs at its own level, not at ours. */
static void pulse(int *coid, int prio, int code, int value)
{
    if (*coid == -1)
        return;
    if (MsgSendPulse(*coid, prio, code, value) == -1 && (errno == EBADF || errno == ESRCH))
        close_coid(coid);
}

void out_pulse_recovery(sup_output_t *o, int code, int value)
{
    pulse(&o->recov_coid, AV_PRIO_RECOVERY, code, value);
}

void out_pulse_logger(sup_output_t *o)
{
    pulse(&o->log_coid, AV_PRIO_LOGGER, AV_PULSE_LOG_KICK, 0);
}
