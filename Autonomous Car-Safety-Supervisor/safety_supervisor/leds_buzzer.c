/*
 * leds_buzzer.c - GPIO back-ends.
 *
 * Default (Stage 0, QNX VM): the pin state is kept in memory and printed with -v, so the
 * supervisor runs without any hardware.
 *
 * Raspberry Pi 5: GPIO is behind the RP1 I/O controller and is accessed through the GPIO
 * service of your QNX Raspberry Pi 5 image. Build with -DAV_GPIO_RPI and implement
 * rpi_gpio_output() / rpi_gpio_write() below with that service's API once you have checked
 * it on the board (ls /dev, BSP documentation). Nothing else in the supervisor changes.
 */
#include <stdio.h>
#include "leds_buzzer.h"

#define MAX_PIN 64

static int g_state[MAX_PIN];
static int g_verbose;

#ifdef AV_GPIO_RPI
#error "Implement rpi_gpio_output()/rpi_gpio_write() with your Pi 5 QNX GPIO service, then remove this line"
static int rpi_gpio_output(int pin) { (void)pin; return -1; }
static int rpi_gpio_write(int pin, int value) { (void)pin; (void)value; return -1; }
#endif

int gpio_init(int verbose)
{
    g_verbose = verbose;
    for (int i = 0; i < MAX_PIN; i++)
        g_state[i] = -1;
#ifdef AV_GPIO_RPI
    for (int pin = 0; pin < 28; pin++)
        rpi_gpio_output(pin);
#endif
    return 0;
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
    rpi_gpio_write(pin, value);
#endif
    if (g_verbose)
        printf("[LEDS] GPIO%-2d -> %d\n", pin, value);
}

int gpio_get(int pin)
{
    return (pin >= 0 && pin < MAX_PIN) ? g_state[pin] : -1;
}
