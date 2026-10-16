/*
 * perception_gateway.c - receives the Jetson's camera detections (UDP) and hands them to the
 * supervisor (camera cross-checks) and navigation (planning) as normal QNX messages.
 *
 * Nothing from the network is trusted. Every packet is checked for length, magic, version,
 * CRC-16, value ranges, sequence order and (with -j) the sender's IP address; bad packets are
 * counted and dropped. Packet layout: shared/hw_protocols.h (24 bytes, little-endian).
 * If the Jetson stops sending, nothing is forwarded and the supervisor's watchdog reports
 * CAMERA_LATE / CAMERA_LOST. Without a Jetson the rover still drives safely on the LiDAR.
 *
 * usage: perception_gateway [-p udp_port] [-j jetson_ip] [-v]     (default port 5005)
 */
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <sys/dispatch.h>

#include "qnx_helpers.h"
#include "hw_protocols.h"
#include "settings.h"

static int g_sup = -1, g_nav = -1;

static void connect_targets(void)
{
    if (g_sup == -1 && (g_sup = name_open(AV_NAME_SUPERVISOR, 0)) != -1) {
        if (qnx_hello(g_sup, AV_ROLE_PERCEPTION, "perception_gw") != 0) {
            name_close(g_sup);
            g_sup = -1;
        } else {
            printf("[CAMERA-GW] connected to supervisor as PERCEPTION\n");
        }
    }
    if (g_nav == -1 && (g_nav = name_open(AV_NAME_NAV, 0)) != -1)
        printf("[CAMERA-GW] connected to navigation\n");
}

static void forward(int *coid, const av_perception_t *p)
{
    if (*coid == -1)
        return;
    if (qnx_send_timeout(*coid, p, sizeof *p, NULL, 0, 5000) == -1 &&
        (errno == EBADF || errno == ESRCH || errno == ENOTCONN || errno == EPERM)) {
        name_close(*coid);
        *coid = -1;
    }
}

int main(int argc, char *argv[])
{
    int port = 5005, verbose = 0, opt;
    const char *jetson_ip = NULL;
    const char *env = getenv("AV_JETSON_PORT");
    if (env && atoi(env) > 0)
        port = atoi(env);
    while ((opt = getopt(argc, argv, "p:j:vh")) != -1) {
        switch (opt) {
        case 'p': port = atoi(optarg); break;
        case 'j': jetson_ip = optarg; break;
        case 'v': verbose = 1; break;
        default:
            fprintf(stderr, "usage: %s [-p udp_port] [-j jetson_ip] [-v]\n", argv[0]);
            return 1;
        }
    }

    struct in_addr allowed;
    if (jetson_ip && inet_aton(jetson_ip, &allowed) == 0) {
        fprintf(stderr, "[CAMERA-GW] bad IP address '%s'\n", jetson_ip);
        return 1;
    }

    qnx_set_priority(AV_PRIO_PERCEPTION);
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_port = htons((uint16_t)port);
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (s == -1 || bind(s, (struct sockaddr *)&a, sizeof a) == -1) {
        perror("[CAMERA-GW] UDP socket");
        return 1;
    }
    printf("[CAMERA-GW] waiting for Jetson detections on UDP port %d%s%s (prio %d)\n", port,
           jetson_ip ? " from " : "", jetson_ip ? jetson_ip : "", AV_PRIO_PERCEPTION);
    fflush(stdout);

    uint32_t last_seq = 0, n_rx = 0, n_ok = 0, n_bad = 0, n_old = 0, n_foreign = 0;
    int have_seq = 0, last_err = -1;
    uint64_t last_ok_us = 0, next_try = 0, next_stats = qnx_now_us() + 5000000u;
    float fps = 0.0f;

    for (;;) {
        struct pollfd pfd = { .fd = s, .events = POLLIN };
        int r = poll(&pfd, 1, 100);
        uint64_t now = qnx_now_us();
        if (now >= next_try) {
            connect_targets();
            next_try = now + 1000000u;
        }
        if (now >= next_stats) {
            printf("[CAMERA-GW] last 5 s: %u packets, %u ok, %u bad, %u old/repeated, %u from wrong IP, "
                   "%.1f frames/s\n", n_rx, n_ok, n_bad, n_old, n_foreign, fps);
            fflush(stdout);
            n_rx = n_ok = n_bad = n_old = n_foreign = 0;
            next_stats = now + 5000000u;
        }
        if (r <= 0 || !(pfd.revents & POLLIN))
            continue;

        uint8_t buf[64];
        struct sockaddr_in from;
        socklen_t fl = sizeof from;
        ssize_t n = recvfrom(s, buf, sizeof buf, 0, (struct sockaddr *)&from, &fl);
        if (n <= 0)
            continue;
        n_rx++;
        if (jetson_ip && from.sin_addr.s_addr != allowed.s_addr) {
            n_foreign++;
            continue;
        }
        jetson_det_t d;
        int err = jetson_parse(buf, (int)n, &d);
        if (err != JET_OK) {
            n_bad++;
            if (err != last_err)
                printf("[CAMERA-GW] dropped packet: %s\n", jetson_err_name(err));
            last_err = err;
            continue;
        }
        last_err = -1;
        /* only newer frames; after 1 s of silence any sequence is accepted (Jetson restarted) */
        int newer = (int32_t)(d.seq - last_seq) > 0;
        if (have_seq && !newer && now - last_ok_us < 1000000u) {
            n_old++;
            continue;
        }
        if (last_ok_us && now > last_ok_us) {
            float inst = 1e6f / (float)(now - last_ok_us);
            fps = fps == 0.0f ? inst : 0.9f * fps + 0.1f * inst;
        }
        have_seq = 1;
        last_seq = d.seq;
        last_ok_us = now;
        n_ok++;

        av_perception_t p;
        memset(&p, 0, sizeof p);
        p.hdr.type = AV_MSG_PERCEPTION;
        p.hdr.version = AV_PROTO_VERSION;
        p.seq = d.seq;
        p.t_capture_us = now - (uint64_t)d.age_ms * 1000u;   /* in the Pi's clock */
        p.obstacle = d.obstacle;
        p.zone = d.zone;
        p.confidence = d.confidence;
        p.class_id = d.class_id;
        p.cam_dist_m = d.dist_m >= 0.0f ? d.dist_m : 0.0f;
        p.odom_speed = -1.0f;                                 /* the camera does not measure speed */
        p.fps = fps;
        forward(&g_sup, &p);
        forward(&g_nav, &p);
        if (verbose)
            printf("[CAMERA-GW] seq %u: %s %s %.2f m conf %u%% age %u ms\n", d.seq,
                   d.obstacle ? "obstacle" : "clear", av_zone_name(d.zone), d.dist_m, d.confidence,
                   d.age_ms);
    }
    return 0;
}
