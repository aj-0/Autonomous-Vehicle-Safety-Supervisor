/*
 * leds_buzzer.c - GPIO safety outputs (LEDs, buzzer, inject marker, motor relay).
 *
 * Default build (QNX VM): the pin state is kept in memory and printed with -v, so the
 * supervisor runs without any hardware.
 *
 * Raspberry Pi (build with EXTRA_CFLAGS=-DAV_GPIO_RPI): the pins are driven through the QNX
 * Raspberry Pi GPIO resource manager. Its "msg" file is opened once, then every change is one
 * QNX message (MsgSend) - the same message passing the rest of the system uses:
 *     fd = open("/dev/gpio/msg", O_RDWR)
 *     RPI_GPIO_SET_SELECT + RPI_GPIO_FUNC_OUT   -> make the pin an output
 *     RPI_GPIO_WRITE value 0/1                  -> drive it
 * <sys/rpi_gpio.h> comes with the QNX Raspberry Pi image / BSP (not with the plain SDP). If the
 * resource manager is not running, the supervisor keeps working and prints a warning.
 * Only the five pins listed in settings.h are touched; UART / I2C pins are left alone.
 */
#include <stdio.h>
#include "leds_buzzer.h"
#include "settings.h"

#define MAX_PIN 64

static int g_state[MAX_PIN];
static int g_verbose;

#ifdef AV_GPIO_RPI
#if !__has_include(<sys/rpi_gpio.h>)
#error "AV_GPIO_RPI needs <sys/rpi_gpio.h> (gitlab.com/qnx/projects/hardware-component-samples, common/system/gpio/sys/rpi_gpio.h): copy it to qnx800/target/qnx/usr/include/sys/"
#endif
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <sys/iomsg.h>
#include <sys/neutrino.h>
#include <sys/rpi_gpio.h>

static int g_fd = -1;

static int rpi_msg(int subtype, int pin, int value)
{
    if (g_fd == -1)
        return -1;
    rpi_gpio_msg_t msg = {
        .hdr.type = _IO_MSG,
        .hdr.subtype = subtype,
        .hdr.mgrid = RPI_GPIO_IOMGR,
        .gpio = pin,
        .value = value,
    };
    return MsgSend(g_fd, &msg, sizeof msg, NULL, 0) == -1 ? -1 : 0;
}

static void rpi_open(int relay_external)
{
    static const int pins[] = { AV_GPIO_RED, AV_GPIO_GREEN, AV_GPIO_BUZZER, AV_GPIO_INJECT,
                                AV_GPIO_MOTOR_RELAY };
    unsigned n_pins = sizeof pins / sizeof pins[0] - (relay_external ? 1u : 0u);
    g_fd = open("/dev/gpio/msg", O_RDWR);
    if (g_fd == -1) {
        printf("[LEDS] WARNING: /dev/gpio/msg not available (%s) - GPIO outputs disabled\n",
               strerror(errno));
        return;
    }
    for (unsigned i = 0; i < n_pins; i++) {
        rpi_msg(RPI_GPIO_WRITE, pins[i], 0);             /* low first: motor relay stays open */
        if (rpi_msg(RPI_GPIO_SET_SELECT, pins[i], RPI_GPIO_FUNC_OUT) == -1)
            printf("[LEDS] WARNING: GPIO%d cannot be set as output: %s\n", pins[i], strerror(errno));
        rpi_msg(RPI_GPIO_WRITE, pins[i], 0);
    }
    printf("[LEDS] Raspberry Pi GPIO ready (red %d, green %d, buzzer %d, inject %d, motor relay %d)\n",
           AV_GPIO_RED, AV_GPIO_GREEN, AV_GPIO_BUZZER, AV_GPIO_INJECT, AV_GPIO_MOTOR_RELAY);
}
#endif

int gpio_init(int verbose, int relay_external)
{
    g_verbose = verbose;
    for (int i = 0; i < MAX_PIN; i++)
        g_state[i] = -1;
#ifdef AV_GPIO_RPI
    rpi_open(relay_external);
#else
    (void)relay_external;
#endif
    return 0;
}

void gpio_force(int pin, int value)
{
    if (pin < 0 || pin >= MAX_PIN)
        return;
    g_state[pin] = value ? 1 : 0;
#ifdef AV_GPIO_RPI
    rpi_msg(RPI_GPIO_WRITE, pin, value ? 1 : 0);
#endif
    if (g_verbose)
        printf("[LEDS] GPIO%-2d -> %d (forced)\n", pin, value ? 1 : 0);
}

void gpio_set(int pin, int value)
{
    if (pin < 0 || pin >= MAX_PIN)
        return;
    value = value ? 1 : 0;
    if (g_state[pin] == value)
        return;
    g_state[pin] = value;
#ifdef AV_GPIO_RPI
    rpi_msg(RPI_GPIO_WRITE, pin, value);
#endif
    if (g_verbose)
        printf("[LEDS] GPIO%-2d -> %d\n", pin, value);
}

int gpio_get(int pin)
{
    return (pin >= 0 && pin < MAX_PIN) ? g_state[pin] : -1;
}
