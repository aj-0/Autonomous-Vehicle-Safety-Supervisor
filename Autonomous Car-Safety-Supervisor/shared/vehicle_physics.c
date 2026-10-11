/*
 * vehicle_physics.c - kinematic bicycle model and stopping-distance physics.
 */
#include <math.h>
#include "vehicle_physics.h"

#define DEG2RAD 0.017453292519943295f
#define RAD2DEG 57.29577951308232f

float physics_yaw_from_steer_dps(float v, float steer_deg, float wheelbase_m)
{
    return v * tanf(steer_deg * DEG2RAD) / wheelbase_m * RAD2DEG;
}

float physics_yaw_from_wheels_dps(float v_left, float v_right, float track_m)
{
    return (v_right - v_left) / track_m * RAD2DEG;
}

float physics_delta_max_deg(float v, float wheelbase_m, float a_lat, float steer_max_deg)
{
    if (v < 0.05f)
        return steer_max_deg;
    float d = atanf(wheelbase_m * a_lat / (v * v)) * RAD2DEG;
    return d < steer_max_deg ? d : steer_max_deg;
}

float physics_stopping_distance(float v, float t_sys, float decel)
{
    return v * t_sys + v * v / (2.0f * decel);
}

float physics_allowed_speed(float d, float t_sys, float decel)
{
    if (d <= 0.0f)
        return 0.0f;
    /* solve v*t + v^2/(2a) = d for v */
    return decel * (-t_sys + sqrtf(t_sys * t_sys + 2.0f * d / decel));
}

float physics_ttc(float d, float v)
{
    return v > 0.01f ? d / v : 1e9f;
}

void physics_steer_to_wheels(float v, float steer_deg, float wheelbase_m, float track_m,
                        float *v_left, float *v_right)
{
    float omega = v * tanf(steer_deg * DEG2RAD) / wheelbase_m;     /* rad/s */
    *v_left  = v - omega * track_m * 0.5f;
    *v_right = v + omega * track_m * 0.5f;
}
