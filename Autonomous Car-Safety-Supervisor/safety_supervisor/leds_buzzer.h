/*
 * leds_buzzer.h - GPIO abstraction (the ONLY file that changes between QNX VM, Pi 4 and Pi 5).
 */
#ifndef LEDS_BUZZER_H
#define LEDS_BUZZER_H

/* relay_external: the rover's vehicle_link owns the motor relay pin; the supervisor only
 * forces it low in an emergency (gpio_force) */
int  gpio_init(int verbose, int relay_external);
void gpio_set(int pin, int value);    /* writes only when the value changes */
void gpio_force(int pin, int value);  /* writes always */
int  gpio_get(int pin);

#endif /* LEDS_BUZZER_H */
