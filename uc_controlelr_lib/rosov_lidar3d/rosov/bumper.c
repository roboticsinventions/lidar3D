#include "bumper.h"

#include "pin/pin.h"

pin_t bumper_pin[10] = {
    { GPIOE, 12, GPIO_Speed_50MHz, GPIO_Mode_IN_FLOATING },
    { GPIOE, 15, GPIO_Speed_50MHz, GPIO_Mode_IN_FLOATING },
    { GPIOA, 6, GPIO_Speed_50MHz, GPIO_Mode_IN_FLOATING },
    { GPIOE, 13, GPIO_Speed_50MHz, GPIO_Mode_IN_FLOATING },
    { GPIOE, 14, GPIO_Speed_50MHz, GPIO_Mode_IN_FLOATING },
    { GPIOD, 13, GPIO_Speed_50MHz, GPIO_Mode_IN_FLOATING },
    { GPIOD, 14, GPIO_Speed_50MHz, GPIO_Mode_IN_FLOATING },
    { GPIOA, 12, GPIO_Speed_50MHz, GPIO_Mode_IN_FLOATING },
    { GPIOE, 8, GPIO_Speed_50MHz, GPIO_Mode_IN_FLOATING },
    { GPIOB, 0, GPIO_Speed_50MHz, GPIO_Mode_IN_FLOATING }
};

bool lidar_initiated = 0;

void bumper_init(void)
{
    uint32_t i;
    for(i=0;i<10;++i)
        pin_init(&bumper_pin[i]);
    lidar_initiated=true;
}

uint16_t bumper_read(void)
{
    if(!lidar_initiated) return 0;
    uint16_t bumper=0;
    uint32_t i;
    for(i=0;i<10;++i)
    {
        //debug("%d ", pin_read(&bumper_pin[i]));
        bumper |= (!pin_read(&bumper_pin[i])<<i);
    }
    //debug("\n");
    return bumper;
}
