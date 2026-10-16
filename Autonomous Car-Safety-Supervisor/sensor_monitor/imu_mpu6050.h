/*
 * imu_mpu6050.h - real MPU6050 IMU on the Raspberry Pi I2C bus (QNX i2c driver, devctl).
 *
 * Based on the MPU6050 I2C code of the Q-SAFE project (team-mate's prototype), adapted to the
 * AV Safety Supervisor: yaw rate + forward/side acceleration instead of pitch/roll.
 * Mounting: +X forward, +Y left, +Z up  ->  positive yaw rate = turning left.
 */
#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H

#include <stdint.h>

typedef struct {
    int      fd;
    uint32_t addr;
    int      calibrated;
    uint32_t cal_n;
    double   gyro_sum[3];
    float    gyro_bias[3];          /* deg/s, measured at start-up while standing still */
    uint8_t  last_raw[14];
    int      raw_seen;
    uint64_t raw_since_us;          /* when the raw frame last changed */
    uint32_t samples, errors;
} mpu6050_t;

/* open the I2C bus (e.g. /dev/i2c1), check WHO_AM_I, configure +-250 deg/s, +-2 g, 100 Hz.
 * Returns 0, or -1 with errno set. */
int  mpu_open(mpu6050_t *m, const char *device, uint32_t addr);

/* read one sample. Returns 0 with new values, or -1 with errno:
 *   EAGAIN   still calibrating (keep the car still for 0.5 s after start)
 *   ESTALE   chip returns exactly the same raw frame for 2 s (stuck)
 *   ERANGE   saturated value or gravity not about 1 g (broken / loose sensor)
 *   other    I2C error or timeout */
int  mpu_read(mpu6050_t *m, uint64_t now_us, float *gyro_z_dps, float *accel_x, float *accel_y);

void mpu_close(mpu6050_t *m);

#endif /* IMU_MPU6050_H */
