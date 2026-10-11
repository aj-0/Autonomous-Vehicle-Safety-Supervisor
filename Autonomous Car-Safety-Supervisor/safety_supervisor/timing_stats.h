/*
 * timing_stats.h - timing statistics of the supervisor (proof of determinism for the jury).
 */
#ifndef SUP_STATS_H
#define SUP_STATS_H

#include <stdint.h>

typedef struct {
    uint32_t ticks, overruns;
    double   cycle_sum_us;
    float    cycle_max_us;
    float    jitter_max_us;          /* |tick interval - period|            */
    float    decision_last_us, decision_max_us;     /* message in -> verdict out */
    float    out_lat_last_us, out_lat_max_us;       /* input in -> vehicle command sent */
    float    inj_last_ms, inj_min_ms, inj_max_ms;   /* fault injected -> first reaction  */
    uint32_t inj_n;
} sup_stats_t;

void stats_init(sup_stats_t *s);
void stats_cycle(sup_stats_t *s, float cycle_us, int overrun);
void stats_tick_interval(sup_stats_t *s, float interval_us, float period_us);
void stats_decision(sup_stats_t *s, float us);
void stats_out_latency(sup_stats_t *s, float us);
void stats_inject(sup_stats_t *s, float ms);
float stats_cycle_avg(const sup_stats_t *s);

#endif /* SUP_STATS_H */
