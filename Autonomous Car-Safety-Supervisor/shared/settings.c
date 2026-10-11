/*
 * settings.c - vehicle profiles and name tables.
 */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "settings.h"

/* Scale prototype: 2WD robot car, ~0.5 m/s. */
static const av_profile_t g_robot = {
    .name = "robot",
    .track_m = 0.17f, .wheelbase_m = 0.20f, .lane_half_m = 0.15f,
    .v_max = 0.8f, .v_cruise = 0.5f,
    .a_comf = 1.0f, .a_max = 2.0f, .a_phys_max = 2.2f, .accel_max = 1.0f, .a_lat_max = 0.5f,
    .steer_max_deg = 30.0f, .steer_rate_dps = 60.0f,
    .t_sys_s = 0.03f, .margin_m = 0.10f, .speed_tol = 0.05f,
    .us_min_m = 0.02f, .us_max_m = 4.0f,
    .cam_range_m = 1.5f, .xcheck_max_m = 1.0f, .phantom_tol_m = 0.3f, .recovery_clear_m = 0.5f,
    .yaw_thresh_dps = 8.0f, .yaw_min_speed = 0.10f, .speed_mismatch = 0.10f, .standstill_v = 0.02f,
    .gyro_max_dps = 250.0f, .accel_abs_max = 19.6f,
    .imu_frozen_ms = 300, .enc_frozen_ms = 500,
    .caution_frac = 0.5f, .degraded_frac = 0.3f,
    .enc_noise = 0.003f, .us_noise = 0.003f, .gyro_noise_dps = 0.3f,
};

/* Full-size passenger car, used in simulation to show real-world numbers (80 km/h). */
static const av_profile_t g_car = {
    .name = "car",
    .track_m = 1.6f, .wheelbase_m = 2.7f, .lane_half_m = 1.0f,
    .v_max = 36.0f, .v_cruise = 22.2f,                 /* 22.2 m/s = 80 km/h */
    .a_comf = 3.0f, .a_max = 9.0f, .a_phys_max = 9.5f, .accel_max = 3.0f, .a_lat_max = 4.0f,
    .steer_max_deg = 35.0f, .steer_rate_dps = 30.0f,
    .t_sys_s = 0.10f, .margin_m = 2.0f, .speed_tol = 0.5f,
    .us_min_m = 0.3f, .us_max_m = 200.0f,              /* "ultrasonic" acts as radar here */
    .cam_range_m = 150.0f, .xcheck_max_m = 60.0f, .phantom_tol_m = 5.0f, .recovery_clear_m = 10.0f,
    .yaw_thresh_dps = 3.0f, .yaw_min_speed = 2.0f, .speed_mismatch = 1.0f, .standstill_v = 0.1f,
    .gyro_max_dps = 250.0f, .accel_abs_max = 19.6f,
    .imu_frozen_ms = 300, .enc_frozen_ms = 500,
    .caution_frac = 0.5f, .degraded_frac = 0.3f,
    .enc_noise = 0.05f, .us_noise = 0.05f, .gyro_noise_dps = 0.2f,
};

int av_tscale = 1;

const av_profile_t *av_profile_get(const char *name)
{
    const char *ts = getenv("AV_TIMESCALE");
    if (ts && atoi(ts) >= 1 && atoi(ts) <= 20)
        av_tscale = atoi(ts);
    if (name && strcmp(name, "car") == 0)
        return &g_car;
    return &g_robot;
}

static const char *g_state_names[ST__COUNT] = {
    "INIT", "NORMAL", "CAUTION", "DEGRADED", "MRM", "ESTOP", "SAFE", "RECOVERY"
};

/* Short names (status line) and plain-English explanations (log), same order as the C_* list. */
static const char *g_cond_names[C__COUNT] = {
    "CAMERA_FLICKER", "CAMERA_PHANTOM", "CAMERA_MISSED_OBSTACLE", "CAMERA_LATE", "CAMERA_LOST",
    "NAV_SPEED_WRONG", "IMU_FAULT", "ONE_WHEEL_SENSOR_FAULT", "BOTH_WHEEL_SENSORS_FAULT",
    "ULTRASONIC_FAULT", "IMU_DISAGREES", "WHEEL_SENSORS_DISAGREE", "CAR_NOT_FOLLOWING_STEERING",
    "MANY_SENSORS_FAULT", "SENSOR_LINK_LOST", "MOTOR_LINK_FAIL", "NAV_NOT_RESPONDING",
    "NAV_CRASHED", "NAV_NOT_TRUSTED", "DRIVING_INTO_UNSEEN_OBSTACLE", "COLLISION_VERY_CLOSE",
    "BRAKES_NOT_WORKING", "STM32_EMERGENCY_STOP", "EMERGENCY_BUTTON", "PATH_BLOCKED"
};

static const char *g_cond_text[C__COUNT] = {
    "camera result is flickering on/off (unstable AI)",
    "camera sees an obstacle that the ultrasonic does not (false alarm)",
    "ultrasonic sees an obstacle that the camera does not (camera missed it)",
    "camera results arrive too late",
    "no camera results for more than 1 s",
    "speed reported by navigation does not match the wheel sensors",
    "IMU (gyro) is dead, frozen or out of range",
    "one wheel sensor is faulty",
    "both wheel sensors are faulty",
    "ultrasonic sensor is faulty",
    "2-out-of-3 vote: IMU turn rate disagrees with steering and wheels -> IMU ignored",
    "2-out-of-3 vote: wheel sensors disagree with steering and IMU -> wheels ignored",
    "2-out-of-3 vote: car is not turning the way it was commanded (steering fault)",
    "several sensors are faulty at the same time",
    "no sensor data received in time (cable or STM32 problem)",
    "commands cannot be delivered to the motors",
    "navigation is alive but has stopped sending (frozen)",
    "navigation program crashed (QNX kernel reported it)",
    "navigation sent too many bad commands in 1 s",
    "navigation is driving towards an obstacle it does not see",
    "obstacle so close that only maximum braking avoids a collision",
    "car is not slowing down although the brakes are applied",
    "STM32 safety chip stopped the motors by itself",
    "emergency stop button pressed",
    "obstacle too close to drive again"
};

static const char *g_reason_names[] = {
    "VALUE_OUT_OF_RANGE", "INVALID_VALUE", "STEERING_TOO_FAST", "TOO_MUCH_STEERING_FOR_SPEED",
    "SWERVE_BLOCKED", "SPEED_LIMITED_BY_STATE", "SPEED_LIMITED_BY_OBSTACLE",
    "CAMERA_MISSED_OBSTACLE", "NAV_NOT_IN_CONTROL"
};

static const char *g_reason_text[] = {
    "value outside the allowed range",
    "value is not a valid number (corrupted)",
    "steering changes faster than the car can steer",
    "too much steering for this speed (car could skid or roll)",
    "evasive swerve blocked because the camera is unreliable",
    "speed reduced because of the current safety state",
    "speed reduced so the car can still stop before the obstacle",
    "navigation drives on although the ultrasonic sees an obstacle",
    "navigation is not allowed to drive right now"
};

static const char *g_state_text[ST__COUNT] = {
    "starting up",
    "all good, navigation drives",
    "something is suspicious, speed reduced to half",
    "a sensor is broken, driving slowly without it",
    "controlled stop (minimum-risk manoeuvre)",
    "emergency braking",
    "stopped and locked",
    "problem gone, waiting for operator 'ack'"
};

static const char *g_fault_names[FAULT__COUNT] = {
    "none",
    "slew", "envelope", "speedest", "hang", "crash", "corrupt",
    "flicker", "miss", "phantom", "cam-dead", "cam-frozen", "cam-lag",
    "imu-frozen", "imu-dead", "enc-dead", "us-dead", "steer-stuck", "brake-fail", "link-drop"
};

const char *av_state_name(int s)
{
    return (s >= 0 && s < ST__COUNT) ? g_state_names[s] : "?";
}

const char *av_verdict_name(int v)
{
    switch (v) {
    case V_PASS:   return "PASS";
    case V_CLAMP:  return "CLAMP";
    case V_REJECT: return "REJECT";
    default:       return "?";
    }
}

const char *av_auth_name(int a)
{
    switch (a) {
    case AUTH_NAV:      return "NAV";
    case AUTH_RECOVERY: return "RECOVERY";
    default:            return "NONE";
    }
}

const char *av_cond_name(int c)
{
    return (c >= 0 && c < C__COUNT) ? g_cond_names[c] : "?";
}

const char *av_health_name(int h)
{
    switch (h) {
    case H_OK:      return "OK";
    case H_DEAD:    return "DEAD";
    case H_FROZEN:  return "FROZEN";
    case H_RANGE:   return "RANGE";
    case H_JUMP:    return "JUMP";
    default:        return "UNKNOWN";
    }
}

const char *av_zone_name(int z)
{
    switch (z) {
    case ZONE_LEFT:   return "LEFT";
    case ZONE_CENTER: return "CENTER";
    case ZONE_RIGHT:  return "RIGHT";
    default:          return "-";
    }
}

const char *av_fault_name(int f)
{
    return (f >= 0 && f < FAULT__COUNT) ? g_fault_names[f] : "?";
}

int av_fault_from_name(const char *s)
{
    for (int i = 1; i < FAULT__COUNT; i++)
        if (strcmp(s, g_fault_names[i]) == 0)
            return i;
    return FAULT_NONE;
}

static char *mask_str(uint32_t mask, const char *const *names, int n, char *buf, int len)
{
    int pos = 0;
    buf[0] = '\0';
    for (int i = 0; i < n && pos < len - 1; i++) {
        if (mask & (1u << i))
            pos += snprintf(buf + pos, (size_t)(len - pos), "%s%s", pos ? "|" : "", names[i]);
    }
    if (pos == 0)
        snprintf(buf, (size_t)len, "-");
    return buf;
}

char *av_reasons_str(uint32_t mask, char *buf, int len)
{
    return mask_str(mask, g_reason_names, (int)(sizeof g_reason_names / sizeof g_reason_names[0]),
                    buf, len);
}

char *av_conds_str(uint32_t mask, char *buf, int len)
{
    return mask_str(mask, g_cond_names, C__COUNT, buf, len);
}

const char *av_cond_text(int c)
{
    return (c >= 0 && c < C__COUNT) ? g_cond_text[c] : "?";
}

const char *av_state_text(int s)
{
    return (s >= 0 && s < ST__COUNT) ? g_state_text[s] : "?";
}

const char *av_reason_text(uint32_t mask)
{
    int n = (int)(sizeof g_reason_text / sizeof g_reason_text[0]);
    for (int i = 0; i < n; i++)
        if (mask & (1u << i))
            return g_reason_text[i];
    return "";
}
