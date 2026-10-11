/*
 * checks.h - the three layers of safety checks.
 *
 *   input      : is the sensor data trustworthy?  (health + 2oo3 yaw vote + braking check)
 *   command    : is the nav command physically possible and sane? (range, slew, envelope, ...)
 *   situation  : is the command safe in the current world? (collision guard, camera cross-check)
 */
#ifndef SUP_CHECKS_H
#define SUP_CHECKS_H

#include "supervisor_data.h"

/* check_sensors.c */
void  chk_input_on_sensor(sup_ctx_t *c, uint64_t now_us);

/* check_obstacle_camera.c */
float chk_allowed_speed(const sup_ctx_t *c, float decel);    /* large value if path is clear */
void  chk_situation_on_sensor(sup_ctx_t *c);
void  chk_perception(sup_ctx_t *c, uint64_t now_us);

/* check_nav_command.c */
void  chk_nav_command(sup_ctx_t *c, const av_nav_cmd_t *cmd, uint64_t now_us, av_verdict_t *v);
void  chk_recovery_command(sup_ctx_t *c, const av_recovery_cmd_t *cmd, uint64_t now_us,
                           av_verdict_t *v);

#endif /* SUP_CHECKS_H */
