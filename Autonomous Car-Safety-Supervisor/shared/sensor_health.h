/*
 * sensor_health.h - per-sensor plausibility tracker: DEAD, FROZEN, out of RANGE, impossible JUMP.
 * Used by sensor_monitor (real hardware) and vehicle_simulator (Stage 0).
 */
#ifndef SENSOR_HEALTH_H
#define SENSOR_HEALTH_H

#include <stdint.h>

typedef struct {
    /* configuration */
    float    min, max;          /* valid physical range                         */
    float    max_rate;          /* max |dv/dt| per second, 0 = no jump check    */
    uint32_t dead_ms;           /* no sample for this long -> DEAD              */
    uint32_t frozen_ms;         /* identical value this long while moving       */
    /* state */
    int      seen;
    uint64_t last_us, last_change_us;
    float    last;
    uint8_t  health;
} health_tracker_t;

void    health_init(health_tracker_t *t, float min, float max, float max_rate,
                uint32_t dead_ms, uint32_t frozen_ms);

/* A new sample arrived. moving: vehicle is moving (frozen check only makes sense then). */
uint8_t health_sample(health_tracker_t *t, float v, uint64_t now_us, int moving);

/* Call every cycle: detects a sensor that stopped producing samples. */
uint8_t health_poll(health_tracker_t *t, uint64_t now_us);

#endif /* SENSOR_HEALTH_H */
