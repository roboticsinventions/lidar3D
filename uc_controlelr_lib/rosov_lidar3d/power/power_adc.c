#include "power_adc.h"

#include "misc.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#include "pin/pin.h"

pin_t power_adc_pin_1 = { GPIOB, 0, GPIO_Speed_50MHz, GPIO_Mode_AIN };    // VAC_CURR
pin_t power_adc_pin_2 = { GPIOA, 0, GPIO_Speed_50MHz, GPIO_Mode_AIN };    // BRUSH_LEFT_CURR
pin_t power_adc_pin_3 = { GPIOB, 1, GPIO_Speed_50MHz, GPIO_Mode_AIN };    // VAC_TEMP           // ADC3 - presure

pin_t multi_1_A = { GPIOD, 5, GPIO_Speed_50MHz, GPIO_Mode_Out_PP };         // led0
pin_t multi_1_B = { GPIOD, 6, GPIO_Speed_50MHz, GPIO_Mode_Out_PP };         // led1
pin_t multi_1_C = { GPIOD, 7, GPIO_Speed_50MHz, GPIO_Mode_Out_PP };         // led2

pin_t multi_2_A = { GPIOA, 12, GPIO_Speed_50MHz, GPIO_Mode_Out_PP };     // VAC_EMPTY2
pin_t multi_2_B = { GPIOE, 8, GPIO_Speed_50MHz, GPIO_Mode_Out_PP };      // VAC_EMPTY3
pin_t multi_2_C = { GPIOB, 9, GPIO_Speed_50MHz, GPIO_Mode_Out_PP };      // i2c1_sda

uint32_t multi_1_state, multi_2_state;
uint16_t multi_1_adc_val[8],multi_2_adc_val[8];

void multi_1_refresh(void)
{
    if(multi_1_state>=8) multi_1_state=0;
    if(multi_1_state&0b001) pin_set(&multi_1_A); else pin_reset(&multi_1_A);
    if(multi_1_state&0b010) pin_set(&multi_1_B); else pin_reset(&multi_1_B);
    if(multi_1_state&0b100) pin_set(&multi_1_C); else pin_reset(&multi_1_C);
}

void multi_2_refresh(void)
{
    if(multi_2_state>=8) multi_2_state=0;                                                   // multi2 tylko na 2 kanaly
    if(multi_2_state&0b001) pin_set(&multi_2_A); else pin_reset(&multi_2_A);
    if(multi_2_state&0b010) pin_set(&multi_2_B); else pin_reset(&multi_2_B);
    if(multi_2_state&0b100) pin_set(&multi_2_C); else pin_reset(&multi_2_C);
}

void power_adc_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

    // ADC1
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5);
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    // ADC2
    ADC_Init(ADC2, &ADC_InitStructure);
    ADC_RegularChannelConfig(ADC2, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
    ADC_Cmd(ADC2, ENABLE);
    ADC_ResetCalibration(ADC2);
    while(ADC_GetResetCalibrationStatus(ADC2));
    ADC_StartCalibration(ADC2);
    while(ADC_GetCalibrationStatus(ADC2));
    ADC_SoftwareStartConvCmd(ADC2, ENABLE);

    // ADC3
    ADC_Init(ADC3, &ADC_InitStructure);
    ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 1, ADC_SampleTime_239Cycles5);
    ADC_Cmd(ADC3, ENABLE);
    ADC_ResetCalibration(ADC3);
    while(ADC_GetResetCalibrationStatus(ADC3));
    ADC_StartCalibration(ADC3);
    while(ADC_GetCalibrationStatus(ADC3));
    ADC_SoftwareStartConvCmd(ADC3, ENABLE);

    // PINY OD ADC
    pin_init(&power_adc_pin_1);
    pin_init(&power_adc_pin_2);
    pin_init(&power_adc_pin_3);

    // PINY OD MULTIPLEXOW
    pin_init(&multi_1_A);
    pin_init(&multi_1_B);
    pin_init(&multi_1_C);
    pin_init(&multi_2_A);
    pin_init(&multi_2_B);
    pin_init(&multi_2_C);

    multi_1_state = 0;
    multi_2_state = 0;

    multi_1_refresh();
    multi_2_refresh();

    //timer do wyzwalania
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);        // pewnie zbedne

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 500;                      // 100 to 10000 pomiarow na sekunde
    TIM_TimeBaseStructure.TIM_Prescaler = 71;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 100;                        // to nie ma znaczenia bo nie ejst zerowane w przerwaniu
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

//uint32_t adc_counter;

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
    {
        //++adc_counter;

        multi_1_adc_val[multi_1_state] = ADC_GetConversionValue(ADC1);
        ++multi_1_state;
        multi_1_refresh();

        multi_2_adc_val[multi_2_state] = ADC_GetConversionValue(ADC2);
        ++multi_2_state;
        multi_2_refresh();
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
    }
}

uint16_t power_adc_get_sensor(uint8_t num)
{
    return 2500-multi_1_adc_val[num]*3300./4095.;
}

#define VOLTAGE_FACTOR (3300./4095.*10.1)     // SPRAWDZ WSP 10.1 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

uint16_t power_adc_get_voltage_1(void)
{
    return multi_2_adc_val[0]*VOLTAGE_FACTOR;
}

uint16_t power_adc_get_voltage_2(void)
{
    return multi_2_adc_val[1]*VOLTAGE_FACTOR;
}

uint16_t power_adc_get_multi_2(uint8_t num)
{
    return multi_2_adc_val[num];//*VOLTAGE_FACTOR;
}

uint8_t power_adc_get_presure_255(void)
{
    return (ADC_GetConversionValue(ADC3)>>4);
}
