/*
 * sensor_monitor.c - Sensor Monitor task (QNX process, priority 30).
 *
 * Sits between the vehicle and the supervisor:
 *
 *   vehicle_simulator / vehicle link --RAW_SENSORS (10 ms)--> sensor_monitor --SENSOR_STATE--> supervisor
 *
 *  - every raw frame: each sensor that delivered a new sample is checked for
 *      RANGE  : value outside the physical range of the sensor
 *      JUMP   : value changed faster than physically possible
 *      FROZEN : exactly the same value while the vehicle moves (real sensors always have noise)
 *    then the readings + one health verdict per sensor are forwarded to the supervisor
 *  - own 10 ms QNX timer: a sensor that stops delivering samples becomes DEAD (even while the
 *    other sensors keep the frames coming); every health change is logged
 *
 * The monitor does not invent data: if the vehicle link goes silent, nothing is forwarded and
 * the supervisor's watchdog reports SENSOR_LINK_LOST. If this process dies, the supervisor gets
 * the kernel disconnect pulse and asks the recovery manager to start a new one.
 *
 * Real IMU: with -i /dev/i2cN (or environment AV_IMU_DEV) the yaw rate and acceleration come
 * from an MPU6050 on the Pi's I2C bus instead of the vehicle link. If the chip fails, the IMU
 * simply delivers no fresh samples -> DEAD -> the supervisor drives on without it (DEGRADED).
 *
 * usage: sensor_monitor [-s robot|car] [-i /dev/i2c1] [-a 0x68|0x69]
 */
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>

#include "qnx_helpers.h"
#include "access_control.h"
#include "sensor_health.h"
#include "settings.h"
#include "imu_mpu6050.h"

static const av_profile_t *P;
static mpu6050_t g_mpu;
static int g_use_mpu, g_mpu_calibrated_logged, g_mpu_last_errno;
static access_roles_t g_roles;
static health_tracker_t g_tr[SENS_COUNT];
static uint8_t g_reported[SENS_COUNT];
static int g_sup = -1;
static uint32_t g_seq, g_frames, g_forwarded;
static uint64_t g_last_raw_us, g_next_try_us;
static int g_link_silent;

static const char *g_sensor_name[SENS_COUNT] = { "left wheel", "right wheel", "ultrasonic", "IMU" };

static const char *health_reason(int h)
{
    switch (h) {
    case H_OK:     return "working";
    case H_DEAD:   return "no new sample in time";
    case H_FROZEN: return "same value while the car moves";
    case H_RANGE:  return "value outside the physical range";
    case H_JUMP:   return "value jumped faster than physically possible";
    default:       return "no data yet";
    }
}

static void connect_supervisor(uint64_t now)
{
    if (g_sup != -1 || now < g_next_try_us)
        return;
    g_next_try_us = now + 1000000u;
    g_sup = name_open(AV_NAME_SUPERVISOR, 0);
    if (g_sup == -1)
        return;
    int err = qnx_hello(g_sup, AV_ROLE_SENSOR, "sensor_monitor");
    if (err) {
        fprintf(stderr, "[SENSOR-MONITOR] supervisor refused SENSOR role: %s\n", strerror(err));
        name_close(g_sup);
        g_sup = -1;
        return;
    }
    printf("[SENSOR-MONITOR] connected to supervisor as SENSOR\n");
}

static void report_changes(void)
{
    for (int i = 0; i < SENS_COUNT; i++) {
        uint8_t h = g_tr[i].health;
        if (h == g_reported[i] || h == H_UNKNOWN)
            continue;
        printf("[SENSOR-MONITOR] %-11s %-7s -> %-7s (%s)\n", g_sensor_name[i],
               av_health_name(g_reported[i]), av_health_name(h), health_reason(h));
        g_reported[i] = h;
    }
    fflush(stdout);
}

/* real IMU: replace the vehicle link's IMU values with the MPU6050 reading */
static void read_real_imu(av_raw_sensors_t *r, uint64_t now)
{
    float gz, ax, ay;
    r->fresh &= (uint8_t)~(1u << SENS_IMU);
    if (mpu_read(&g_mpu, now, &gz, &ax, &ay) == 0) {
        r->gyro_z_dps = gz;
        r->accel_x = ax;
        r->accel_y = ay;
        r->fresh |= 1u << SENS_IMU;
        g_mpu_last_errno = 0;
        if (!g_mpu_calibrated_logged) {
            g_mpu_calibrated_logged = 1;
            printf("[SENSOR-MONITOR] MPU6050 calibrated, gyro offset z %.2f deg/s\n", g_mpu.gyro_bias[2]);
        }
        return;
    }
    if (errno != g_mpu_last_errno) {                      /* log each new kind of problem once */
        g_mpu_last_errno = errno;
        printf("[SENSOR-MONITOR] MPU6050: %s\n",
               errno == EAGAIN ? "calibrating - keep the car still and level" :
               errno == ESTALE ? "chip stuck (identical data for 2 s)" :
               errno == ERANGE ? "value saturated or gravity not 1 g" : strerror(errno));
    }
    r->gyro_z_dps = 0.0f;                                 /* never pass on simulated IMU data */
    r->accel_x = r->accel_y = 0.0f;
}

static void on_raw(const av_raw_sensors_t *in, uint64_t now)
{
    av_raw_sensors_t frame = *in;
    const av_raw_sensors_t *r = &frame;
    if (g_use_mpu)
        read_real_imu(&frame, now);
    g_frames++;
    g_last_raw_us = now;
    if (g_link_silent) {
        printf("[SENSOR-MONITOR] vehicle link back\n");
        g_link_silent = 0;
    }

    /* frozen check only makes sense while the car moves: judged from the wheel sensors */
    float v = 0.5f * (r->v_left + r->v_right);
    int moving = fabsf(v) > 2.0f * P->standstill_v;

    if (r->fresh & (1u << SENS_ENC_L))
        health_sample(&g_tr[SENS_ENC_L], r->v_left, now, moving);
    if (r->fresh & (1u << SENS_ENC_R))
        health_sample(&g_tr[SENS_ENC_R], r->v_right, now, moving);
    if (r->fresh & (1u << SENS_IMU))
        health_sample(&g_tr[SENS_IMU], r->gyro_z_dps, now, moving);
    if (r->fresh & (1u << SENS_US))
        health_sample(&g_tr[SENS_US], r->distance_m, now, moving && !r->us_no_echo);
    for (int i = 0; i < SENS_COUNT; i++)
        health_poll(&g_tr[i], now);

    av_sensor_state_t s;
    memset(&s, 0, sizeof s);
    s.hdr.type = AV_MSG_SENSOR_STATE;
    s.hdr.version = AV_PROTO_VERSION;
    s.seq = ++g_seq;
    s.t_us = r->t_us;
    s.v_left = r->v_left;
    s.v_right = r->v_right;
    s.distance_m = r->distance_m;
    s.gyro_z_dps = r->gyro_z_dps;
    s.accel_x = r->accel_x;
    s.accel_y = r->accel_y;
    for (int i = 0; i < SENS_COUNT; i++)
        s.health[i] = g_tr[i].health;
    s.us_no_echo = r->us_no_echo;
    s.island_state = r->island_state;
    s.estop = r->estop;
    s.relay_closed = r->relay_closed;
    s.ack_button = r->ack_button;

    if (g_sup == -1)
        return;
    if (qnx_send_timeout(g_sup, &s, sizeof s, NULL, 0, 5000) == -1) {
        if (errno == EBADF || errno == ESRCH || errno == ENOTCONN || errno == EPERM) {
            printf("[SENSOR-MONITOR] supervisor gone, reconnecting\n");
            name_close(g_sup);
            g_sup = -1;
        }
        return;
    }
    g_forwarded++;
}

static void on_tick(uint64_t now)
{
    connect_supervisor(now);
    for (int i = 0; i < SENS_COUNT; i++)
        health_poll(&g_tr[i], now);
    report_changes();
    if (g_last_raw_us && !g_link_silent &&
        now - g_last_raw_us > (uint64_t)AV_SENSOR_TIMEOUT_MS * 1000u) {
        g_link_silent = 1;
        printf("[SENSOR-MONITOR] no frames from the vehicle link for %d ms (nothing forwarded)\n",
               AV_SENSOR_TIMEOUT_MS);
    }
}

typedef union {
    uint16_t          type;
    struct _pulse     pulse;
    av_hdr_t          hdr;
    av_hello_t        hello;
    av_raw_sensors_t  raw;
} mon_msg_t;

int main(int argc, char *argv[])
{
    const char *profile = "robot";
    const char *imu_dev = getenv("AV_IMU_DEV");           /* inherited by a restarted monitor */
    const char *imu_addr = getenv("AV_IMU_ADDR");
    int opt;
    while ((opt = getopt(argc, argv, "s:i:a:h")) != -1) {
        switch (opt) {
        case 's': profile = optarg; break;
        case 'i': imu_dev = optarg; break;
        case 'a': imu_addr = optarg; break;
        default:
            fprintf(stderr, "usage: %s [-s robot|car] [-i /dev/i2cN] [-a 0x68|0x69]\n", argv[0]);
            return 1;
        }
    }
    P = av_profile_get(profile);

    if (imu_dev && *imu_dev) {
        uint32_t addr = imu_addr ? (uint32_t)strtoul(imu_addr, NULL, 0) : 0x68u;
        g_use_mpu = 1;                                    /* even if it fails: no fake IMU data */
        if (mpu_open(&g_mpu, imu_dev, addr) == 0)
            printf("[SENSOR-MONITOR] real IMU: MPU6050 at 0x%02x on %s (WHO_AM_I ok), calibrating\n",
                   addr, imu_dev);
        else
            printf("[SENSOR-MONITOR] real IMU: MPU6050 at 0x%02x on %s FAILED: %s -> IMU will be DEAD\n",
                   addr, imu_dev, strerror(errno));
    }

    /* physical limits of each sensor: range, max rate of change, dead after, frozen after */
    health_init(&g_tr[SENS_ENC_L], -1.5f * P->v_max, 1.5f * P->v_max, 3.0f * P->a_phys_max, 50, P->enc_frozen_ms);
    health_init(&g_tr[SENS_ENC_R], -1.5f * P->v_max, 1.5f * P->v_max, 3.0f * P->a_phys_max, 50, P->enc_frozen_ms);
    health_init(&g_tr[SENS_US], 0.0f, P->us_max_m + 0.01f, 0.0f, 150, 1000);
    health_init(&g_tr[SENS_IMU], -P->gyro_max_dps, P->gyro_max_dps, 5000.0f, 50, P->imu_frozen_ms);
    for (int i = 0; i < SENS_COUNT; i++)
        g_reported[i] = H_UNKNOWN;

    qnx_set_priority(AV_PRIO_SENSOR);
    qnx_lock_memory();
    access_init(&g_roles);

    qnx_server_t srv;
    if (qnx_server_create(&srv, AV_NAME_SENSOR, 1) == -1)
        return 1;
    if (qnx_pulse_timer(srv.chid, AV_PRIO_SENSOR, AV_PULSE_TICK, AV_TICK_MS * 1000u, NULL) == -1)
        return 1;
    printf("[SENSOR-MONITOR] sensor monitor ready  profile=%s  prio=%d  fixed-priority channel=%s\n",
           P->name, AV_PRIO_SENSOR, srv.fixed_priority ? "yes" : "NO");
    fflush(stdout);

    for (;;) {
        mon_msg_t m;
        struct _msg_info info;
        int rcvid = MsgReceive(srv.chid, &m, sizeof m, &info);
        if (rcvid == -1)
            continue;
        uint64_t now = qnx_now_us();
        if (rcvid == 0) {
            if (m.pulse.code == AV_PULSE_TICK) {
                on_tick(now);
            } else if (m.pulse.code == _PULSE_CODE_DISCONNECT) {
                if (access_drop(&g_roles, m.pulse.scoid, NULL, NULL, 0))
                    printf("[SENSOR-MONITOR] vehicle link disconnected\n");
                ConnectDetach(m.pulse.scoid);
            }
            continue;
        }
        if (qnx_handle_system_msg(rcvid, &m))
            continue;

        switch (m.hdr.type) {
        case AV_MSG_HELLO: {
            m.hello.name[sizeof m.hello.name - 1] = '\0';
            int err = access_hello(&g_roles, info.scoid, info.pid, m.hello.roles, m.hello.name);
            if (err) {
                MsgError(rcvid, err);
            } else {
                MsgReply(rcvid, EOK, NULL, 0);
                printf("[SENSOR-MONITOR] vehicle link '%s' (pid %d) connected\n", m.hello.name, info.pid);
            }
            break;
        }
        case AV_MSG_RAW_SENSORS:
            /* only the registered vehicle link may deliver sensor data */
            if (!access_has_role(&g_roles, info.scoid, AV_ROLE_SENSOR)) {
                MsgError(rcvid, EPERM);
                break;
            }
            if (info.srcmsglen < (int)sizeof m.raw) {
                MsgError(rcvid, EBADMSG);
                break;
            }
            MsgReply(rcvid, EOK, NULL, 0);                 /* release the vehicle link first */
            on_raw(&m.raw, now);
            break;
        default:
            MsgError(rcvid, ENOSYS);
            break;
        }
    }
    return 0;
}
