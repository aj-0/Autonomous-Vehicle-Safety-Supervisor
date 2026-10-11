/*
 * watchdog.c - is everybody alive and on time?
 *
 *  - sensor feed       : a sensor frame must arrive every 10 ms (timeout 30 ms)
 *  - navigation        : heartbeat pulse + command stream (hang detection), crash is
 *                        reported separately by the kernel disconnect pulse
 *  - perception        : fresh camera packets (stale / lost / frozen sequence number)
 *  - navigation trust  : too many rejected commands in a short window
 *  - output link       : commands must reach the vehicle
 */
#include "watchdog.h"
#include "decision_core.h"

#define MS(x) ((uint64_t)(x) * 1000u)

int wd_rejects_in_window(const sup_ctx_t *c, uint64_t now)
{
    int n = 0;
    for (int i = 0; i < SUP_HIST; i++)
        if (c->reject_t[i] && now - c->reject_t[i] <= MS(AV_TRUST_WINDOW_MS))
            n++;
    return n;
}

void wd_tick(sup_ctx_t *c, uint64_t now)
{
    /* sensor feed: without fresh frames the last speed is meaningless */
    if (c->sens_seen) {
        int lost = now - c->sens_rx_us > MS(AV_SENSOR_TIMEOUT_MS);
        sup_cond(c, C_SENSOR_LINK_LOST, lost);
        if (lost)
            c->v_valid = 0;
    }

    /* navigation: alive but silent = hang */
    if (c->nav_ever && c->nav_connected) {
        uint64_t hb_age = now - c->nav_hb_us;
        uint64_t cmd_age = now - c->nav_cmd_us;
        int lost = hb_age > MS(AV_HB_TIMEOUT_MS) || cmd_age > MS(AV_NAV_CMD_TIMEOUT_MS);
        sup_cond(c, C_NAV_HB_LOST, lost);
        uint64_t silent = hb_age < cmd_age ? hb_age : cmd_age;
        if (lost && !c->restart_requested && silent > MS(AV_NAV_RESTART_AFTER_MS)) {
            c->actions |= ACT_RESTART_NAV;           /* recovery kills the hung process */
            c->restart_requested = 1;
        }
    } else {
        sup_cond(c, C_NAV_HB_LOST, 0);               /* a crash is covered by NAV_CRASHED */
    }

    /* perception freshness (a frozen camera repeats its seq, so it counts as stale) */
    if (c->perc_seen) {
        uint64_t age = now - c->perc_fresh_us;
        int lost = age > MS(AV_PERC_LOST_MS);
        int stale = age > MS(AV_PERC_STALE_MS) || c->perc_latency_us > MS(AV_PERC_STALE_MS);
        sup_cond(c, C_PERC_LOST, lost);
        sup_cond(c, C_PERC_STALE, stale && !lost);
        if (stale) {
            c->miss_frames = c->phantom_frames = 0;
            sup_cond(c, C_CAMERA_MISS, 0);
            sup_cond(c, C_PHANTOM, 0);
        }
    }

    /* navigation trust */
    sup_cond(c, C_NAV_UNTRUSTED, wd_rejects_in_window(c, now) >= AV_TRUST_MAX_REJECTS);

    /* output link */
    sup_cond(c, C_OUTPUT_LINK_FAIL, c->output_fail_n >= 3);
}
