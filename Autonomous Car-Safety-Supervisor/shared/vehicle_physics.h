/*
 * vehicle_physics.h - the physics used by every safety check.
 * Kinematic bicycle model + stopping-distance equations. Pure functions, no I/O.
 */
#ifndef VEHICLE_PHYSICS_H
#define VEHICLE_PHYSICS_H

/* Yaw rate (deg/s) expected from speed v (m/s) and steering angle (deg): r = v*tan(d)/L */
float physics_yaw_from_steer_dps(float v, float steer_deg, float wheelbase_m);

/* Yaw rate (deg/s) measured by the wheel encoders: r = (vR - vL) / W */
float physics_yaw_from_wheels_dps(float v_left, float v_right, float track_m);

/* Largest steering angle that keeps lateral acceleration <= a_lat: atan(L*a_lat / v^2) */
float physics_delta_max_deg(float v, float wheelbase_m, float a_lat, float steer_max_deg);

/* Distance needed to stop: reaction distance + braking distance */
float physics_stopping_distance(float v, float t_sys, float decel);

/* Highest speed from which the vehicle can still stop within distance d (inverse of above) */
float physics_allowed_speed(float d, float t_sys, float decel);

/* Time to collision (s), large value when not closing in */
float physics_ttc(float d, float v);

/* Convert (speed, virtual steering angle) into left/right wheel speeds for a differential drive */
void physics_steer_to_wheels(float v, float steer_deg, float wheelbase_m, float track_m,
                        float *v_left, float *v_right);

#endif /* VEHICLE_PHYSICS_H */
