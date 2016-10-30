#ifndef __POWER_ADC_H__
#define __POWER_ADC_H__

#include <stdint.h>

void power_adc_init(void);

uint16_t power_adc_get_sensor(uint8_t num);
uint16_t power_adc_get_voltage_1(void);
uint16_t power_adc_get_voltage_2(void);

#endif /* __POWER_ADC_H__ */
