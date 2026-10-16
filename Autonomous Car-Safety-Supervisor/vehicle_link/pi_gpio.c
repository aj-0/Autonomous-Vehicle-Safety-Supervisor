/*
 * pi_gpio.c - thin wrapper around the QNX Raspberry Pi GPIO resource manager messages
 * (message layout: <sys/rpi_gpio.h>, usage as in QNX's hardware-component-samples).
 */
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <sys/iomsg.h>
#include <sys/neutrino.h>
#include <sys/rpi_gpio.h>

#include "pi_gpio.h"

static int g_fd = -1;

int pig_open(void)
{
    if (g_fd == -1)
        g_fd = open("/dev/gpio/msg", O_RDWR);
    return g_fd == -1 ? -1 : 0;
}

static int send_msg(int subtype, int pin, unsigned value, unsigned *reply)
{
    if (g_fd == -1) {
        errno = ENODEV;
        return -1;
    }
    rpi_gpio_msg_t msg = {
        .hdr.type = _IO_MSG,
        .hdr.subtype = subtype,
        .hdr.mgrid = RPI_GPIO_IOMGR,
        .gpio = (unsigned)pin,
        .value = value,
    };
    if (MsgSend(g_fd, &msg, sizeof msg, reply ? &msg : NULL, reply ? sizeof msg : 0) == -1)
        return -1;
    if (reply)
        *reply = msg.value;
    return 0;
}

int pig_write(int pin, int level)
{
    return send_msg(RPI_GPIO_WRITE, pin, level ? 1u : 0u, NULL);
}

int pig_output(int pin, int level)
{
    pig_write(pin, level);                       /* level first, then switch to output */
    if (send_msg(RPI_GPIO_SET_SELECT, pin, RPI_GPIO_FUNC_OUT, NULL) == -1)
        return -1;
    return pig_write(pin, level);
}

int pig_input_pullup(int pin)
{
    if (send_msg(RPI_GPIO_SET_SELECT, pin, RPI_GPIO_FUNC_IN, NULL) == -1)
        return -1;
    return send_msg(RPI_GPIO_PUD, pin, RPI_GPIO_PUD_UP, NULL);
}

int pig_read(int pin)
{
    unsigned v = 0;
    if (send_msg(RPI_GPIO_READ, pin, 1u, &v) == -1)
        return -1;
    return v ? 1 : 0;
}

int pig_pwm_setup(int pin, unsigned hz, unsigned range)
{
    if (g_fd == -1) {
        errno = ENODEV;
        return -1;
    }
    if (send_msg(RPI_GPIO_SET_SELECT, pin, RPI_GPIO_FUNC_OUT, NULL) == -1)
        return -1;
    rpi_gpio_pwm_t msg = {
        .hdr.type = _IO_MSG,
        .hdr.subtype = RPI_GPIO_PWM_SETUP,
        .hdr.mgrid = RPI_GPIO_IOMGR,
        .gpio = (unsigned)pin,
        .frequency = hz,
        .range = range,
        .mode = RPI_PWM_MODE_MS,                 /* mark-space: fixed frequency, like QNX's motor sample */
    };
    if (MsgSend(g_fd, &msg, sizeof msg, NULL, 0) == -1)
        return -1;
    return pig_pwm_duty(pin, 0);
}

int pig_pwm_duty(int pin, unsigned value)
{
    return send_msg(RPI_GPIO_PWM_DUTY, pin, value, NULL);
}

int pig_rising_edge_pulse(int pin, int coid, int prio, int code)
{
    if (g_fd == -1) {
        errno = ENODEV;
        return -1;
    }
    rpi_gpio_event_t msg = {
        .hdr.type = _IO_MSG,
        .hdr.subtype = RPI_GPIO_ADD_EVENT,
        .hdr.mgrid = RPI_GPIO_IOMGR,
        .gpio = (unsigned)pin,
        .detect = RPI_EVENT_EDGE_RISING,
    };
    SIGEV_PULSE_INIT(&msg.event, coid, prio, code, pin);
    /* QNX 8: an event delivered by a server must be registered with that server first */
    if (MsgRegisterEvent(&msg.event, g_fd) == -1)
        return -1;
    return MsgSend(g_fd, &msg, sizeof msg, NULL, 0) == -1 ? -1 : 0;
}
