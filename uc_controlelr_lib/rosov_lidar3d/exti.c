

#include "exti.h"

#include "stm32f10x_exti.h"
#include "engines/encoders.h"
#include "engines/engines_defs.h"

void EXTI0_IRQHandler()
{
    encoder_Handler_A(EngineLeft);
    EXTI_ClearITPendingBit(EXTI_Line0);
}

void EXTI1_IRQHandler()
{
    encoder_Handler_B(EngineLeft);
    EXTI_ClearITPendingBit(EXTI_Line1);
}

void EXTI2_IRQHandler()
{

    EXTI_ClearITPendingBit(EXTI_Line2);
}

void EXTI3_IRQHandler()
{
    encoder_Handler_A(EngineRight);
    EXTI_ClearITPendingBit(EXTI_Line3);
}

void EXTI4_IRQHandler()
{
    encoder_Handler_B(EngineRight);
    EXTI_ClearITPendingBit(EXTI_Line4);
}

void EXTI9_5_IRQHandler()
{
    if(EXTI_GetITStatus(EXTI_Line5) != RESET)
    {

        EXTI_ClearITPendingBit(EXTI_Line5);
    }
    else if(EXTI_GetITStatus(EXTI_Line6) != RESET)
    {

        EXTI_ClearITPendingBit(EXTI_Line6);
    }
    else if(EXTI_GetITStatus(EXTI_Line7) != RESET)
    {

        EXTI_ClearITPendingBit(EXTI_Line7);
    }
    else if(EXTI_GetITStatus(EXTI_Line8) != RESET)
    {
        //battery_Handler(0);
        EXTI_ClearITPendingBit(EXTI_Line8);
    }
    else if(EXTI_GetITStatus(EXTI_Line9) != RESET)
    {
        //battery_Handler(0);
        EXTI_ClearITPendingBit(EXTI_Line9);
    }
}

void EXTI15_10_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line10) != RESET)
    {

        EXTI_ClearITPendingBit(EXTI_Line10);
    }
    else if(EXTI_GetITStatus(EXTI_Line11) != RESET)
    {

        EXTI_ClearITPendingBit(EXTI_Line11);
    }
    else if(EXTI_GetITStatus(EXTI_Line12) != RESET)
    {

        EXTI_ClearITPendingBit(EXTI_Line12);
    }
    else if(EXTI_GetITStatus(EXTI_Line13) != RESET)
    {

        EXTI_ClearITPendingBit(EXTI_Line13);
    }
    else if(EXTI_GetITStatus(EXTI_Line14) != RESET)
    {

        EXTI_ClearITPendingBit(EXTI_Line14);
    }
    else if(EXTI_GetITStatus(EXTI_Line15) != RESET)
    {

        EXTI_ClearITPendingBit(EXTI_Line15);
    }
}
