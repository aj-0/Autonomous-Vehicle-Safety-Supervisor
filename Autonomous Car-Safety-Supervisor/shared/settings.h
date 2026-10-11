/*
 * settings.h - every tunable number in one place: periods, timeouts, priorities,
 * GPIO pins and the two vehicle profiles ("robot" = scale prototype, "car" = full-size car).
 */
#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdint.h>
#include "messages.h"

/* ---- periods ---- */
#define AV_TICK_MS                 10    /* supervisor + sensor rate (100 Hz)       */
#define AV_NAV_PERIOD_MS           50    /* navigation command rate (20 Hz)         */
#define AV_PERC_EVERY_TICKS        3     /* simulated camera: ~33 Hz                */
#define AV_US_EVERY_TICKS          4     /* ultrasonic: 25 Hz                       */
#define AV_RECOV_PERIOD_MS         20
#define AV_LOGGER_POLL_MS          100

/* ---- timeouts ----
 * Communication timeouts are multiplied by av_tscale. It is 1 on real hardware (Pi) and is set
 * by the environment variable AV_TIMESCALE (e.g. 5, "./start_all.sh robot vm") on a virtual
 * machine, where the host OS can pause the whole guest for tens of milliseconds. */
extern int av_tscale;
#define AV_SENSOR_TIMEOUT_MS       (30 * av_tscale)
#define AV_HB_TIMEOUT_MS           (100 * av_tscale)
#define AV_NAV_CMD_TIMEOUT_MS      (150 * av_tscale)
#define AV_NAV_RESTART_AFTER_MS    (500 * av_tscale)   /* hung nav is killed + restarted after this */
#define AV_PERC_STALE_MS           (100 * av_tscale)
#define AV_PERC_LOST_MS            1000
#define AV_RECOV_CMD_TIMEOUT_MS    (60 * av_tscale)
#define AV_ISLAND_TIMEOUT_MS       (30 * av_tscale)
#define AV_ISLAND_REARM_FRAMES     10
#define AV_OUTPUT_SEND_TIMEOUT_US  (2000 * av_tscale)
#define AV_INJECT_WINDOW_MS        5000  /* reaction to an injection must come within this */

/* ---- detection tuning ---- */
#define AV_PERSIST_SAMPLES         5     /* residual must persist 5 samples (50 ms)  */
#define AV_FLICKER_WINDOW_MS       200
#define AV_FLICKER_TRANSITIONS     3
#define AV_FLICKER_CLEAR_MS        500
#define AV_XCHECK_FRAMES           3
#define AV_SPEED_MISMATCH_N        5
#define AV_TRUST_WINDOW_MS         1000
#define AV_TRUST_MAX_REJECTS       3
#define AV_BRAKE_CHECK_MS          300
#define AV_SWERVE_LOCK_DEG         2.0f
#define AV_OVERRUN_US              (2000 * av_tscale)

/* ---- state machine ---- */
#define AV_DEESCALATE_HOLD_MS      500
#define AV_STANDSTILL_HOLD_MS      200
#define AV_RECOVERY_CLEAR_MS       2000

/* ---- priorities (SCHED_FIFO) ---- */
#define AV_PRIO_SUPERVISOR         50
#define AV_PRIO_LINK               48
#define AV_PRIO_RECOVERY           45
#define AV_PRIO_SIM                40
#define AV_PRIO_SENSOR             30
#define AV_PRIO_NAV                10
#define AV_PRIO_LOGGER             8
#define AV_PRIO_CLI                5

/* ---- GPIO, BCM numbering on the Raspberry Pi 40-pin header ---- */
#define AV_GPIO_RED                17
#define AV_GPIO_GREEN              27
#define AV_GPIO_BUZZER             22
#define AV_GPIO_INJECT             23    /* fault-injection marker -> STM32 stopwatch */
#define AV_GPIO_SAFE_REQ           24    /* supervisor safe request -> STM32          */

typedef struct {
    const char *name;
    /* geometry */
    float track_m;          /* distance between left and right wheel           */
    float wheelbase_m;      /* (virtual) wheelbase for the bicycle model       */
    float lane_half_m;      /* obstacle half width seen by the sensors (sim)   */
    /* speeds and accelerations */
    float v_max, v_cruise;
    float a_comf;           /* controlled-stop deceleration (MRM)              */
    float a_max;            /* emergency deceleration (ESTOP)                  */
    float a_phys_max;       /* physical braking limit of the vehicle (sim)     */
    float accel_max;
    float a_lat_max;        /* lateral acceleration limit (steering envelope)  */
    float steer_max_deg, steer_rate_dps;
    /* collision guard */
    float t_sys_s;          /* system reaction time                            */
    float margin_m;         /* distance kept to an obstacle                    */
    float speed_tol;
    /* sensors */
    float us_min_m, us_max_m;
    float cam_range_m, xcheck_max_m, phantom_tol_m, recovery_clear_m;
    float yaw_thresh_dps, yaw_min_speed, speed_mismatch, standstill_v;
    float gyro_max_dps, accel_abs_max;
    uint32_t imu_frozen_ms, enc_frozen_ms;
    /* state speed caps, fraction of v_cruise */
    float caution_frac, degraded_frac;
    /* simulation noise */
    float enc_noise, us_noise, gyro_noise_dps;
} av_profile_t;

const av_profile_t *av_profile_get(const char *name);   /* "robot" (default) or "car"; also reads AV_TIMESCALE */

const char *av_state_name(int s);
const char *av_verdict_name(int v);
const char *av_auth_name(int a);
const char *av_cond_name(int c);
const char *av_health_name(int h);
const char *av_zone_name(int z);
const char *av_fault_name(int f);
int         av_fault_from_name(const char *s);
char       *av_reasons_str(uint32_t mask, char *buf, int len);
char       *av_conds_str(uint32_t mask, char *buf, int len);
const char *av_cond_text(int c);          /* plain-English explanation of one problem      */
const char *av_state_text(int s);         /* plain-English meaning of a safety state       */
const char *av_reason_text(uint32_t mask);/* explanation of the first reason in the mask   */

#endif /* SETTINGS_H */
