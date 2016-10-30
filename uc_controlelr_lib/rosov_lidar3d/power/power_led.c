#include "power_led.h"

#include "pin/pin.h"

pin_t pwr_led_3 =   {GPIOD, 8, GPIO_Speed_2MHz, GPIO_Mode_Out_PP};
pin_t pwr_led_4 =   {GPIOD, 9, GPIO_Speed_2MHz, GPIO_Mode_Out_PP};
pin_t pwr_led_5 =   {GPIOD, 10, GPIO_Speed_2MHz, GPIO_Mode_Out_PP};
pin_t pwr_led_6 =   {GPIOD, 11, GPIO_Speed_2MHz, GPIO_Mode_Out_PP};

pin_t *pwr_led[5];

void power_led_init(void)
{
    pin_init(&pwr_led_3);
    pin_init(&pwr_led_4);
    pin_init(&pwr_led_5);
    pin_init(&pwr_led_6);
    pwr_led[1] = &pwr_led_3;
    pwr_led[2] = &pwr_led_4;
    pwr_led[3] = &pwr_led_5;
    pwr_led[4] = &pwr_led_6;
    power_led_reset(1);
    power_led_reset(2);
    power_led_reset(3);
    power_led_reset(4);
}

void power_led_set(uint8_t i)
{
    if(i<1 || i>4) { debug("Wrong pwrled number\n"); return; }
    pin_set(pwr_led[i]);
}

void power_led_reset(uint8_t i)
{
    if(i<1 || i>4) { debug("Wrong pwrled number\n"); return; }
    pin_reset(pwr_led[i]);
}

void power_led_invert(uint8_t i)
{
    if(i<1 || i>4) { debug("Wrong pwrled number\n"); return; }
    pin_invert(pwr_led[i]);
}

