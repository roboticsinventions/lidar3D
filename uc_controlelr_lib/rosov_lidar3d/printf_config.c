#include "printf_config.h"

#include "uart/uart2.h"

void printf_uart_putch(uint8_t ch)
{
    uart1_putch(ch);
}
