/*
 * event_buffer.c - shared-memory event ring (single producer, single consumer).
 */
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>

#include "event_buffer.h"

av_ring_t *ring_open(void)
{
    int fd = shm_open(AV_RING_NAME, O_RDWR | O_CREAT, 0666);
    if (fd == -1) {
        perror("[ring] shm_open");
        return NULL;
    }
    if (ftruncate(fd, sizeof(av_ring_t)) == -1) {
        perror("[ring] ftruncate");
        close(fd);
        return NULL;
    }
    av_ring_t *r = mmap(NULL, sizeof(av_ring_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (r == MAP_FAILED) {
        perror("[ring] mmap");
        return NULL;
    }
    /* A new shared object is zero-filled; whoever opens it first initialises it. */
    if (r->magic != AV_RING_MAGIC) {
        r->cap = AV_RING_CAP;
        atomic_store(&r->head, 0);
        atomic_store(&r->tail, 0);
        atomic_store(&r->dropped, 0);
        r->magic = AV_RING_MAGIC;
    }
    return r;
}

int ring_push(av_ring_t *r, const av_event_t *ev)
{
    unsigned head = atomic_load_explicit(&r->head, memory_order_relaxed);
    unsigned tail = atomic_load_explicit(&r->tail, memory_order_acquire);
    if (head - tail >= AV_RING_CAP) {
        atomic_fetch_add_explicit(&r->dropped, 1, memory_order_relaxed);
        return -1;
    }
    r->slot[head & (AV_RING_CAP - 1)] = *ev;
    atomic_store_explicit(&r->head, head + 1, memory_order_release);
    return 0;
}

int ring_pop(av_ring_t *r, av_event_t *ev)
{
    unsigned tail = atomic_load_explicit(&r->tail, memory_order_relaxed);
    unsigned head = atomic_load_explicit(&r->head, memory_order_acquire);
    if (tail == head)
        return 0;
    *ev = r->slot[tail & (AV_RING_CAP - 1)];
    atomic_store_explicit(&r->tail, tail + 1, memory_order_release);
    return 1;
}
