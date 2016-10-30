#ifndef __POWER_LED_H__
#define __POWER_LED_H__

#include <stdint.h>

void power_led_init(void);

void power_led_set(uint8_t);
void power_led_reset(uint8_t);
void power_led_invert(uint8_t);

#endif /* __POWER_LED_H__ */


