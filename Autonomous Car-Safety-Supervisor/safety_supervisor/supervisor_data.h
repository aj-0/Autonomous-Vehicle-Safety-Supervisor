/*
 * supervisor_data.h - the single state structure of the safety supervisor.
 *
 * Everything the supervisor knows lives in one statically allocated sup_ctx_t.
 * No heap allocation happens after start-up. Only plain C types here, so the
 * decision logic can be compiled and unit-tested on a host PC.
 */
#ifndef SUPERVISOR_DATA_H
#define SUPERVISOR_DATA_H

#include <stdint.h>
#include "messages.h"
#include "settings.h"
#include "timing_stats.h"

/* Actions the platform layer (supervisor_main.c) must carry out after a core call. */
#define ACT_OUTPUT_NOW   (1u << 0)   /* state escalated: send vehicle command immediately */
#define ACT_RECOV_START  (1u << 1)
#define ACT_RECOV_STOP   (1u << 2)
#define ACT_RESTART_NAV  (1u << 3)
#define ACT_LOG_KICK     (1u << 4)
#define ACT_RESTART_SENSOR (1u << 5) /* sensor monitor died: recovery starts a new one */

#define SUP_HIST 8

typedef void (*sup_emit_fn)(void *user, const av_event_t *ev);

typedef struct sup_ctx {
    const av_profile_t *P;
    int      bench_mode;             /* car on a stand: yaw voting disabled */
    uint64_t now_us, start_us;

    /* ---- sensors (from sensor_monitor / vehicle_simulator) ---- */
    av_sensor_state_t sens;
    int      sens_seen;
    uint64_t sens_rx_us, sens_prev_rx_us;
    float    v_true;                 /* speed from healthy encoders            */
    int      v_valid;
    float    dist_m;                 /* ultrasonic distance                    */
    int      obstacle_in_range;
    int      us_ok;
    float    yaw_model;              /* expected yaw rate (rate-limited model) */
    int      pers_mg, pers_me, pers_ge;

    /* ---- perception (camera, untrusted) ---- */
    av_perception_t perc;
    int      perc_seen;
    uint64_t perc_fresh_us;          /* rx time of the last packet with a NEW seq */
    uint64_t perc_latency_us;        /* capture -> arrival, when the clock domain allows */
    uint8_t  perc_last_flag;
    uint64_t flick_t[SUP_HIST];
    int      flick_idx;
    int      perc_unstable;
    int      miss_frames, phantom_frames;

    /* ---- navigation (untrusted doer) ---- */
    int      nav_connected, nav_ever;
    int32_t  nav_pid;
    uint64_t nav_hb_us, nav_cmd_us;
    av_nav_cmd_t nav_last;
    float    applied_speed, applied_steer;   /* last nav command that passed the checks */
    uint64_t reject_t[SUP_HIST];
    int      reject_idx;
    int      speed_mismatch_n;
    int      restart_requested;

    /* ---- recovery (controlled stop) ---- */
    float    recov_speed, recov_steer;
    uint64_t recov_cmd_us;
    float    mrm_v0;
    uint64_t mrm_t0;
    float    brake_v0;
    uint64_t brake_t0;

    /* ---- safe-state deadline (safety_deadline.c) ---- */
    int      dl_active, dl_missed;
    uint64_t dl_t0_us, dl_end_us;    /* stop started, stop must be done by       */
    float    dl_budget_ms;
    float    dl_last_ms;             /* time used by the running / last stop     */
    uint32_t dl_pass_n, dl_fail_n;
    float    dl_worst_ms, dl_min_margin_ms;

    /* ---- conditions and state machine ---- */
    uint32_t conds, conds_logged;
    av_state_t state;
    uint64_t state_since_us, lower_since_us, clear_since_us, standstill_since_us;
    int      ack_pending;
    int      ack_btn_prev;           /* physical ACK button (rover): act on the press edge */
    uint8_t  authority;

    /* ---- output ---- */
    av_vehicle_cmd_t out;
    int      output_fail_n;

    /* ---- bookkeeping ---- */
    uint32_t actions;
    av_verdict_t last_verdict;
    uint32_t pass_n, clamp_n, reject_n;
    uint64_t inject_t_us;
    uint32_t inject_fault, last_inject_fault;
    sup_stats_t stats;
    sup_emit_fn emit;
    void    *emit_user;
    uint32_t ev_seq;
} sup_ctx_t;

#endif /* SUPERVISOR_DATA_H */
