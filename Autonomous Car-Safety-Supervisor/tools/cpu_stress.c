/*
 * cpu_stress.c - CPU load generator for the determinism demo.
 *
 * Starts N busy threads at a chosen SCHED_FIFO priority for T seconds.
 *   default prio 9  : below navigation -> the car keeps driving, CPU is saturated,
 *                     show that supervisor timing does not change (operator_cli status)
 *   -p 20           : above navigation -> nav starves, the supervisor detects the
 *                     missing heartbeat and stops the car safely
 *
 * usage: stress [-t threads] [-p prio] [-s seconds]
 */
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

static int g_seconds = 10;

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + ts.tv_nsec * 1e-9;
}

static void *burn(void *arg)
{
    (void)arg;
    double end = now_s() + g_seconds;
    volatile unsigned long x = 0;
    while (now_s() < end)
        for (int i = 0; i < 100000; i++)
            x += (unsigned long)i * 7u;
    return NULL;
}

int main(int argc, char *argv[])
{
    int threads = 4, prio = 9, opt;
    while ((opt = getopt(argc, argv, "t:p:s:h")) != -1) {
        switch (opt) {
        case 't': threads = atoi(optarg); break;
        case 'p': prio = atoi(optarg); break;
        case 's': g_seconds = atoi(optarg); break;
        default:
            fprintf(stderr, "usage: %s [-t threads] [-p prio] [-s seconds]\n", argv[0]);
            return 1;
        }
    }
    printf("[CPU-STRESS] %d busy threads at priority %d for %d s\n", threads, prio, g_seconds);
    fflush(stdout);

    pthread_t th[64];
    if (threads > 64)
        threads = 64;
    for (int i = 0; i < threads; i++) {
        pthread_attr_t a;
        struct sched_param sp;
        memset(&sp, 0, sizeof sp);
        sp.sched_priority = prio;
        pthread_attr_init(&a);
        pthread_attr_setinheritsched(&a, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&a, SCHED_FIFO);
        pthread_attr_setschedparam(&a, &sp);
        pthread_create(&th[i], &a, burn, NULL);
    }
    for (int i = 0; i < threads; i++)
        pthread_join(th[i], NULL);
    printf("[CPU-STRESS] done\n");
    return 0;
}
