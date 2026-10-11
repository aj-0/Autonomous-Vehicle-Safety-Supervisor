/*
 * qnx_helpers.c - QNX real-time helpers.
 */
#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/iomsg.h>
#include <sys/mman.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include <sys/syspage.h>

#include "qnx_helpers.h"
#include "messages.h"

uint64_t qnx_now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

uint64_t qnx_cycles(void)
{
    return ClockCycles();
}

double qnx_cycles_to_us(uint64_t cycles)
{
    static double cps = 0.0;
    if (cps == 0.0)
        cps = (double)SYSPAGE_ENTRY(qtime)->cycles_per_sec;
    return (double)cycles * 1e6 / cps;
}

int qnx_set_priority(int prio)
{
    struct sched_param sp;
    memset(&sp, 0, sizeof sp);
    sp.sched_priority = prio;
    int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (rc != EOK) {
        fprintf(stderr, "[rt] cannot set priority %d: %s\n", prio, strerror(rc));
        return -1;
    }
    return 0;
}

int qnx_lock_memory(void)
{
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        fprintf(stderr, "[rt] mlockall failed: %s (continuing)\n", strerror(errno));
        return -1;
    }
    return 0;
}

int qnx_pin_to_core(int core)
{
    if (core < 0)
        return 0;
    unsigned mask = 1u << core;
    if (ThreadCtl(_NTO_TCTL_RUNMASK, (void *)(uintptr_t)mask) == -1) {
        fprintf(stderr, "[rt] cannot pin to core %d: %s\n", core, strerror(errno));
        return -1;
    }
    return 0;
}

int qnx_server_create(qnx_server_t *srv, const char *name, int fixed_priority)
{
    memset(srv, 0, sizeof *srv);
    srv->chid = -1;

    /* Preferred path: our own channel with exactly the flags we want, then register the name on it. */
    unsigned flags = _NTO_CHF_DISCONNECT | (fixed_priority ? _NTO_CHF_FIXED_PRIORITY : 0);
    int chid = ChannelCreate(flags);
    if (chid != -1) {
        srv->dpp = dispatch_create_channel(chid, DISPATCH_FLAG_NOLOCK);
        if (srv->dpp != NULL)
            srv->attach = name_attach(srv->dpp, name, 0);
        if (srv->attach != NULL) {
            srv->chid = srv->attach->chid;
            srv->fixed_priority = fixed_priority;
            return 0;
        }
        fprintf(stderr, "[rt] name_attach on own channel failed (%s), using default channel\n",
                strerror(errno));
    }

    /* Fallback: let name_attach create the channel (priority inheritance stays active). */
    srv->attach = name_attach(NULL, name, 0);
    if (srv->attach == NULL) {
        fprintf(stderr, "[rt] name_attach(%s) failed: %s\n", name, strerror(errno));
        return -1;
    }
    srv->chid = srv->attach->chid;
    srv->fixed_priority = 0;
    return 0;
}

void qnx_server_destroy(qnx_server_t *srv)
{
    if (srv->attach)
        name_detach(srv->attach, 0);
    srv->attach = NULL;
}

int qnx_pulse_timer(int chid, int prio, int code, uint32_t period_us, timer_t *out)
{
    int coid = ConnectAttach(ND_LOCAL_NODE, 0, chid, _NTO_SIDE_CHANNEL, 0);
    if (coid == -1) {
        perror("[rt] ConnectAttach");
        return -1;
    }
    struct sigevent ev;
    SIGEV_PULSE_INIT(&ev, coid, prio, code, 0);

    timer_t tid;
    if (timer_create(CLOCK_MONOTONIC, &ev, &tid) == -1) {
        perror("[rt] timer_create");
        return -1;
    }
    struct itimerspec its;
    its.it_value.tv_sec = period_us / 1000000u;
    its.it_value.tv_nsec = (long)(period_us % 1000000u) * 1000L;
    its.it_interval = its.it_value;
    if (timer_settime(tid, 0, &its, NULL) == -1) {
        perror("[rt] timer_settime");
        return -1;
    }
    if (out)
        *out = tid;
    return 0;
}

int qnx_name_open_retry(const char *name, int tries, int delay_ms)
{
    for (int i = 0; i < tries; i++) {
        int coid = name_open(name, 0);
        if (coid != -1)
            return coid;
        if (delay_ms > 0)
            usleep((useconds_t)delay_ms * 1000u);
    }
    return -1;
}

long qnx_send_timeout(int coid, const void *smsg, size_t sbytes, void *rmsg, size_t rbytes,
                     uint32_t timeout_us)
{
    struct sigevent ev;
    SIGEV_UNBLOCK_INIT(&ev);
    uint64_t ns = (uint64_t)timeout_us * 1000u;
    if (TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_SEND | _NTO_TIMEOUT_REPLY, &ev, &ns, NULL) == -1)
        return -1;
    return MsgSend(coid, smsg, sbytes, rmsg, rbytes);
}

int qnx_hello(int coid, uint32_t roles, const char *name)
{
    av_hello_t h;
    memset(&h, 0, sizeof h);
    h.hdr.type = AV_MSG_HELLO;
    h.hdr.version = AV_PROTO_VERSION;
    h.roles = roles;
    h.pid = getpid();
    strncpy(h.name, name, sizeof h.name - 1);
    if (qnx_send_timeout(coid, &h, sizeof h, NULL, 0, 200000) == -1)
        return errno;
    return 0;
}

int qnx_handle_system_msg(int rcvid, const void *msg)
{
    uint16_t type = *(const uint16_t *)msg;
    if (type == _IO_CONNECT) {              /* sent by name_open() */
        MsgReply(rcvid, EOK, NULL, 0);
        return 1;
    }
    if (type > _IO_BASE && type <= _IO_MAX) {
        MsgError(rcvid, ENOSYS);
        return 1;
    }
    return 0;
}
