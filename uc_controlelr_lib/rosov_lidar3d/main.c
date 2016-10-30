
#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#include "debugger/debug_leds.h"
#include "debugger/debug_switch.h"
#include "debugger/debug_uart.h"
#include "pin/pin.h"
#include "pin/pins_standby.h"
#include "system_clock/system_clock.h"
#include "systick/delay_ms.h"
#include "systick/systick.h"
#include "uart/uart2.h"
#include "uart/printf.h"
#include "watchdog/watchdog.h"

#include "engines/encoders.h"
#include "engines/engines.h"
#include "engines/engines_defs.h"
#include "lights/lights.h"
#include "power/power.h"
#include "power/power_adc.h"
#include "power/power_led.h"
#include "video/video.h"

#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "pin/pin.h"

int32_t lidar_pose;
bool lidar_dir;

pin_t brush_left_dir_pin = { GPIOE, 12, GPIO_Speed_50MHz, GPIO_Mode_Out_PP };
pin_t brush_left_pwm_pin = { GPIOE, 14, GPIO_Speed_50MHz, GPIO_Mode_Out_PP };
pin_t brush_right_dir_pin = { GPIOE, 15, GPIO_Speed_50MHz, GPIO_Mode_Out_PP };
pin_t brush_right_pwm_pin = { GPIOE, 13, GPIO_Speed_50MHz, GPIO_Mode_Out_PP };

int main(void)
{
    debug_leds_init();
    debug_led_on(0);

    pins_standby();
    system_clock_init();

    debug_leds_init();
    debug_switch_init();
    debug_uart_init(57600);
    debug_led_on(0);

    debug("\nBooting up.");

    //bumper_init();
    debug(".");

    //brush_init();
    pin_init(&brush_left_dir_pin);
    pin_init(&brush_left_pwm_pin);
    pin_init(&brush_right_dir_pin);
    pin_init(&brush_right_pwm_pin);
    pin_reset(&brush_right_dir_pin);
    lidar_dir = 0;
    lidar_pose = 0;
    debug('.');

    comm_init();
    debug(".");

    engines_init();
    debug(".");

    lights_init();
    lights_set_mode(LightsModeBoth);

    watchdog_init_custom_time(20000);
    systick_init();
    debug(".\n");

    delay_ms(200);

    while(true)
    {
        /*int i;
        for(i=0;i<450;++i)
        {
            delay_ms(1);
            pin_set(&brush_right_pwm_pin);
            delay_ms(1);
        }
        delay_ms(100);
        lidar_dir = 1 - lidar_dir;
        pin_invert(&brush_right_dir_pin);*/
        comm_pull_msg();
        watchdog_reload();
    }

    while(true);
    return 0;
}

uint32_t systick_counter;

uint32_t czas_machniecia=3;

void systick_1ms(void)
{
    if( systick_counter%(czas_machniecia*1000) < (czas_machniecia*900) )
    {
        if( systick_counter%czas_machniecia == 0 )
        {
            if(lidar_dir == 0) lidar_pose--; else lidar_pose++;
            pin_invert(&brush_right_pwm_pin);
        }
    }

    if( systick_counter%(czas_machniecia*1000) == 0 )
    {
        lidar_dir = 1 - lidar_dir;
        pin_invert(&brush_right_dir_pin);
    }

    ++systick_counter;
    if( systick_counter%5 == 0 )
    {
        engine_pid_update(EngineLeft);
    }
    if( (systick_counter+1)%5 == 0 )
    {
        engine_pid_update(EngineRight);
    }
    if( (systick_counter+2)%100 == 0 )
    {
        //comm_pull_msg();
    }
    if( systick_counter%1000 == 0 )
    {
        debug_led_invert(1);
    }
}
