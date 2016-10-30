#include "power.h"

#include "debugger/debug_switch.h"

#include "power_adc.h"
#include "power_led.h"

void power_init(void)
{
    power_adc_init();
    power_led_init();
}
