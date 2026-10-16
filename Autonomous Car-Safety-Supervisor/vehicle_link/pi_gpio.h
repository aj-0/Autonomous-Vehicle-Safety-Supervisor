/*
 * pi_gpio.h - Raspberry Pi GPIO through the QNX rpi_gpio resource manager (/dev/gpio/msg).
 * Every call is one QNX message (MsgSend) to the resource manager.
 */
#ifndef PI_GPIO_H
#define PI_GPIO_H

int pig_open(void);                                    /* 0 ok, -1 errno            */
int pig_output(int pin, int level);                    /* make output, set level    */
int pig_input_pullup(int pin);                         /* input with pull-up        */
int pig_write(int pin, int level);
int pig_read(int pin);                                 /* 0 / 1, -1 on error        */
int pig_pwm_setup(int pin, unsigned hz, unsigned range);
int pig_pwm_duty(int pin, unsigned value);             /* 0 .. range                */
/* deliver a pulse (code, value = pin) to coid on every rising edge of pin */
int pig_rising_edge_pulse(int pin, int coid, int prio, int code);

#endif /* PI_GPIO_H */
