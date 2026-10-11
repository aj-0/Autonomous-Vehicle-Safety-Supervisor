/*
 * sensor_health.c - DEAD / FROZEN / RANGE / JUMP detection for one sensor channel.
 */
#include <math.h>
#include "sensor_health.h"
#include "messages.h"

void health_init(health_tracker_t *t, float min, float max, float max_rate,
             uint32_t dead_ms, uint32_t frozen_ms)
{
    t->min = min;
    t->max = max;
    t->max_rate = max_rate;
    t->dead_ms = dead_ms;
    t->frozen_ms = frozen_ms;
    t->seen = 0;
    t->last_us = t->last_change_us = 0;
    t->last = 0.0f;
    t->health = H_UNKNOWN;
}

uint8_t health_sample(health_tracker_t *t, float v, uint64_t now_us, int moving)
{
    uint8_t h = H_OK;

    if (!isfinite(v) || v < t->min || v > t->max) {
        t->last_us = now_us;
        t->seen = 1;
        t->health = H_RANGE;
        return t->health;
    }

    if (t->seen && t->max_rate > 0.0f && now_us > t->last_us) {
        float dt = (float)(now_us - t->last_us) * 1e-6f;
        if (fabsf(v - t->last) / dt > t->max_rate)
            h = H_JUMP;
    }

    /* A value that never changes while the vehicle moves is a frozen sensor.
     * Real sensors always show some noise. */
    if (!t->seen || !moving || t->frozen_ms == 0 || fabsf(v - t->last) > 1e-7f)
        t->last_change_us = now_us;
    else if (now_us - t->last_change_us > (uint64_t)t->frozen_ms * 1000u)
        h = H_FROZEN;

    t->last = v;
    t->last_us = now_us;
    t->seen = 1;
    t->health = h;
    return h;
}

uint8_t health_poll(health_tracker_t *t, uint64_t now_us)
{
    if (t->seen && now_us - t->last_us > (uint64_t)t->dead_ms * 1000u)
        t->health = H_DEAD;
    return t->health;
}
