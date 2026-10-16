/*
 * imu_mpu6050.c - MPU6050 over the QNX I2C resource manager.
 *
 * Every register access is one devctl(DCMD_I2C_SEND / DCMD_I2C_SENDRECV) on the i2c driver,
 * bounded by a kernel timeout (TimerTimeout), so a hung bus can never block the sensor
 * monitor for more than AV_IMU_I2C_TIMEOUT_US.
 *
 * I2C register code based on the Q-SAFE project (team-mate's prototype, credited in README).
 */
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>
#include <devctl.h>
#include <hw/i2c.h>
#include <sys/neutrino.h>

#include "imu_mpu6050.h"

#define AV_IMU_I2C_TIMEOUT_US 5000u     /* one register transfer, 100 kHz bus needs ~1.5 ms */
#define AV_IMU_CAL_SAMPLES    50u       /* 0.5 s of standing still at start-up              */
#define AV_IMU_STUCK_US       2000000u  /* identical raw frame (incl. temperature) for 2 s  */
#define G_MPS2                9.80665f

typedef struct { i2c_sendrecv_t h; uint8_t data[14]; } read_frame_t;
typedef struct { i2c_send_t h; uint8_t data[2]; } write_frame_t;
_Static_assert(offsetof(read_frame_t, data) == sizeof(i2c_sendrecv_t), "i2c read layout");
_Static_assert(offsetof(write_frame_t, data) == sizeof(i2c_send_t), "i2c write layout");

static int arm_timeout(void)
{
    uint64_t ns = (uint64_t)AV_IMU_I2C_TIMEOUT_US * 1000u;
    return TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_SEND | _NTO_TIMEOUT_REPLY, NULL, &ns, NULL);
}

static int read_regs(mpu6050_t *m, uint8_t reg, uint8_t *out, uint32_t n)
{
    read_frame_t f;
    if (n == 0 || n > sizeof f.data) {
        errno = EINVAL;
        return -1;
    }
    memset(&f, 0, sizeof f);
    f.h.slave.addr = m->addr;
    f.h.slave.fmt = I2C_ADDRFMT_7BIT;
    f.h.send_len = 1;
    f.h.recv_len = n;
    f.h.stop = 1;
    f.data[0] = reg;
    if (arm_timeout() == -1)
        return -1;
    int rc = devctl(m->fd, DCMD_I2C_SENDRECV, &f, sizeof f.h + n, NULL);
    if (rc != EOK) {
        errno = rc;
        return -1;
    }
    memcpy(out, f.data, n);
    return 0;
}

static int write_reg(mpu6050_t *m, uint8_t reg, uint8_t value)
{
    write_frame_t f;
    memset(&f, 0, sizeof f);
    f.h.slave.addr = m->addr;
    f.h.slave.fmt = I2C_ADDRFMT_7BIT;
    f.h.len = 2;
    f.h.stop = 1;
    f.data[0] = reg;
    f.data[1] = value;
    if (arm_timeout() == -1)
        return -1;
    int rc = devctl(m->fd, DCMD_I2C_SEND, &f, sizeof f.h + sizeof f.data, NULL);
    if (rc != EOK) {
        errno = rc;
        return -1;
    }
    return 0;
}

void mpu_close(mpu6050_t *m)
{
    if (m->fd >= 0)
        close(m->fd);
    m->fd = -1;
}

int mpu_open(mpu6050_t *m, const char *device, uint32_t addr)
{
    /* register, value: clock = gyro X PLL, all axes on, 1 kHz / (1 + 9) = 100 Hz,
     * low-pass 44 Hz, gyro +-250 deg/s, accel +-2 g */
    static const uint8_t cfg[][2] = {
        { 0x6B, 0x01 }, { 0x6C, 0x00 }, { 0x19, 9 }, { 0x1A, 3 }, { 0x1B, 0x00 }, { 0x1C, 0x00 },
    };
    uint8_t who, back;

    memset(m, 0, sizeof *m);
    m->fd = -1;
    m->addr = addr;
    if (addr != 0x68 && addr != 0x69) {
        errno = EINVAL;
        return -1;
    }
    m->fd = open(device, O_RDWR);
    if (m->fd == -1)
        return -1;
    if (read_regs(m, 0x75, &who, 1) == -1)
        goto fail;
    if (who != 0x68) {                                  /* WHO_AM_I */
        errno = ENODEV;
        goto fail;
    }
    if (write_reg(m, 0x6B, 0x80) == -1)                 /* device reset */
        goto fail;
    usleep(100000);
    for (size_t i = 0; i < sizeof cfg / sizeof cfg[0]; i++) {
        if (write_reg(m, cfg[i][0], cfg[i][1]) == -1 || read_regs(m, cfg[i][0], &back, 1) == -1)
            goto fail;
        if (back != cfg[i][1]) {                        /* read back every setting */
            errno = EIO;
            goto fail;
        }
    }
    return 0;
fail: {
        int saved = errno;
        mpu_close(m);
        errno = saved;
        return -1;
    }
}

static int16_t be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

int mpu_read(mpu6050_t *m, uint64_t now_us, float *gyro_z_dps, float *accel_x, float *accel_y)
{
    uint8_t raw[14];                                    /* accel x y z, temp, gyro x y z */
    float a[3], g[3];

    if (m->fd < 0) {
        errno = ENODEV;
        return -1;
    }
    if (read_regs(m, 0x3B, raw, sizeof raw) == -1) {
        m->errors++;
        return -1;
    }

    /* a live sensor always has noise: an identical frame for 2 s means the chip is stuck */
    if (!m->raw_seen || memcmp(raw, m->last_raw, sizeof raw) != 0) {
        memcpy(m->last_raw, raw, sizeof raw);
        m->raw_seen = 1;
        m->raw_since_us = now_us;
    } else if (now_us - m->raw_since_us > AV_IMU_STUCK_US) {
        m->errors++;
        errno = ESTALE;
        return -1;
    }

    for (int i = 0; i < 3; i++) {
        int16_t ar = be16(raw + 2 * i), gr = be16(raw + 8 + 2 * i);
        if (ar == INT16_MIN || ar == INT16_MAX || gr == INT16_MIN || gr == INT16_MAX) {
            m->errors++;
            errno = ERANGE;                             /* saturated: value unknown */
            return -1;
        }
        a[i] = (float)ar / 16384.0f;                    /* g      (+-2 g range)       */
        g[i] = (float)gr / 131.0f;                      /* deg/s  (+-250 deg/s range) */
    }

    /* gravity check: the total acceleration of a working sensor is about 1 g */
    float norm = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (!isfinite(norm) || norm < 0.7f || norm > 1.3f) {
        m->errors++;
        errno = ERANGE;
        return -1;
    }

    /* start-up: average the gyro while standing level and still -> zero offset */
    if (!m->calibrated) {
        if (fabsf(a[0]) > 0.15f || fabsf(a[1]) > 0.15f || a[2] < 0.85f ||
            fabsf(g[0]) > 10.0f || fabsf(g[1]) > 10.0f || fabsf(g[2]) > 10.0f) {
            m->cal_n = 0;                               /* moved or tilted: start again */
            memset(m->gyro_sum, 0, sizeof m->gyro_sum);
            errno = EAGAIN;
            return -1;
        }
        for (int i = 0; i < 3; i++)
            m->gyro_sum[i] += g[i];
        if (++m->cal_n < AV_IMU_CAL_SAMPLES) {
            errno = EAGAIN;
            return -1;
        }
        for (int i = 0; i < 3; i++)
            m->gyro_bias[i] = (float)(m->gyro_sum[i] / m->cal_n);
        m->calibrated = 1;
    }

    *gyro_z_dps = g[2] - m->gyro_bias[2];
    *accel_x = a[0] * G_MPS2;
    *accel_y = a[1] * G_MPS2;
    m->samples++;
    return 0;
}
