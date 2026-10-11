/*
 * timing_stats.c - timing statistics.
 */
#include <math.h>
#include <string.h>
#include "timing_stats.h"

#define WARMUP_TICKS 20     /* ignore start-up jitter */

void stats_init(sup_stats_t *s)
{
    memset(s, 0, sizeof *s);
}

void stats_cycle(sup_stats_t *s, float cycle_us, int overrun)
{
    s->ticks++;
    s->cycle_sum_us += cycle_us;
    if (cycle_us > s->cycle_max_us)
        s->cycle_max_us = cycle_us;
    if (overrun)
        s->overruns++;
}

void stats_tick_interval(sup_stats_t *s, float interval_us, float period_us)
{
    if (s->ticks < WARMUP_TICKS)
        return;
    float j = fabsf(interval_us - period_us);
    if (j > s->jitter_max_us)
        s->jitter_max_us = j;
}

void stats_decision(sup_stats_t *s, float us)
{
    s->decision_last_us = us;
    if (us > s->decision_max_us)
        s->decision_max_us = us;
}

void stats_out_latency(sup_stats_t *s, float us)
{
    s->out_lat_last_us = us;
    if (us > s->out_lat_max_us)
        s->out_lat_max_us = us;
}

void stats_inject(sup_stats_t *s, float ms)
{
    s->inj_last_ms = ms;
    if (s->inj_n == 0 || ms < s->inj_min_ms)
        s->inj_min_ms = ms;
    if (ms > s->inj_max_ms)
        s->inj_max_ms = ms;
    s->inj_n++;
}

float stats_cycle_avg(const sup_stats_t *s)
{
    return s->ticks ? (float)(s->cycle_sum_us / s->ticks) : 0.0f;
}
