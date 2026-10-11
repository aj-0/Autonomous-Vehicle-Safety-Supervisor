/*
 * leds_buzzer.h - GPIO abstraction (the ONLY file that changes between QNX VM, Pi 4 and Pi 5).
 */
#ifndef LEDS_BUZZER_H
#define LEDS_BUZZER_H

int  gpio_init(int verbose);
void gpio_set(int pin, int value);    /* writes only when the value changes */
int  gpio_get(int pin);

#endif /* LEDS_BUZZER_H */
