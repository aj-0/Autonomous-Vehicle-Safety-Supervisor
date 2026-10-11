/*
 * event_buffer.h - lock-free single-producer / single-consumer ring in POSIX shared memory.
 * The supervisor pushes events without ever blocking; the low-priority logger drains them.
 */
#ifndef EVENT_BUFFER_H
#define EVENT_BUFFER_H

#include <stdatomic.h>
#include <stdint.h>
#include "messages.h"

#define AV_RING_NAME  "/av_events"
#define AV_RING_CAP   2048u                 /* power of two */
#define AV_RING_MAGIC 0x53454E31u           /* "SEN1" */

typedef struct {
    uint32_t     magic;
    uint32_t     cap;
    atomic_uint  head;                       /* written by the producer only */
    atomic_uint  tail;                       /* written by the consumer only */
    atomic_uint  dropped;
    av_event_t  slot[AV_RING_CAP];
} av_ring_t;

av_ring_t *ring_open(void);                 /* creates the shared object if needed */
int         ring_push(av_ring_t *r, const av_event_t *ev);   /* 0 ok, -1 full (dropped) */
int         ring_pop(av_ring_t *r, av_event_t *ev);          /* 1 got one, 0 empty      */

#endif /* EVENT_BUFFER_H */
