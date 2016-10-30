#include "comm.h"

#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "misc.h"

#include "pin/pin.h"

#define TX_SIZE (comm_tx_in - comm_tx_out)
#define RX_SIZE (comm_rx_in - comm_rx_out)

uint8_t comm_rx_buf[255], comm_tx_buf[255];
uint8_t comm_rx_in, comm_rx_out;
uint8_t comm_tx_in, comm_tx_out;

bool comm_new_message_waiting;

void comm_init(void)
{
    uart3_init_custom_baud(57600);

    comm_rx_in=0;
    comm_rx_out=0;
    comm_tx_in=0;
    comm_tx_out=0;
    comm_new_message_waiting = false;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

int int2str(char *ptr,int32_t num)
{
    int i=0;
    int temp=0;
    int32_t cmp = 1000000000;
    if(num < 0)
    {
        ptr[i++]='-';
        num=-num;
    }
    while(cmp>0)
    {
        if (num>=cmp)
        {
            ptr[i] = num/cmp + '0' - temp;
            temp=num/cmp*10;
            ++i;
        }
        cmp/=10;
    }
    if(i==0)
    {
        ptr[i++]='0';
    }
    ptr[i] = 0;
    return i;
}

extern int32_t lidar_pose;

void comm_push_msg(void)
{
    if(TX_SIZE != 0)
    {
        return;
    }
    char msg[100];
    char *ptr = msg;
    int i;
    *ptr++ = '{';
    i = int2str(ptr,encoder_get_ticks(0));
    ptr+=i;
    *ptr = ',';
    ++ptr;
    i = int2str(ptr,encoder_get_ticks(1));
    ptr+=i;
    *ptr = ',';
    ++ptr;
    *ptr = '0'; ++ptr; // zamiast zderzakow
    //i = int2str(ptr,bumper_read());
    *ptr = ',';
    ++ptr;
    i = int2str(ptr,lidar_pose);
    ptr+=i;
    *ptr++ = '}';
    *ptr = 0;
    debug("re=%s\n",msg);
    //debug(" l=%d r=%d b=%d",encoder_get_ticks(0),encoder_get_ticks(1),bumper_read());
    comm_put(msg);
}

void comm_pull_msg(void)
{
    if(comm_new_message_waiting == 0)
    {
        return;
    }
    while(true)
    {
        uint8_t c = comm_get();
        if(RX_SIZE == 0)
        {
            comm_new_message_waiting = 0;
            return;
        }
        if(c=='{') break;
    }
    --comm_new_message_waiting;
    int cmd_num = comm_get();
    if(comm_get() != ',') return;
    uint8_t msg[255];
    uint8_t msg_it=0;
    while(true)
    {
        uint8_t c = comm_get();
        if(c=='}')
        {
            msg[msg_it++] = 0;
            break;
        }
        msg[msg_it++] = c;
        if(RX_SIZE == 0) return;
        if(c=='{')
        {
            msg_it = 0;
        }
    }
    debug("in=%s\t",msg);
    switch(cmd_num)
    {
        case '1':
            comm_parse_velocity(msg);
            break;
    }
    comm_push_msg();
}

void comm_put(uint8_t *str)
{
    while(*str)
    {
        comm_tx_buf[comm_tx_in++] = *str;
        str++;
    }
    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
}

uint8_t comm_get(void)
{
    return comm_rx_buf[comm_rx_out++];
}

bool isNumeric(uint8_t c) { if(c>='0' && c<='9') {return true;} else {return false;} }

int parse_double(uint8_t *ptr, double *num)
{
    int sign = 1;
    uint8_t* ptr_start = ptr;
    if(*ptr == '-')
    {
        ++ptr;
        sign = -1;
    }
    while(isNumeric(*ptr))
    {
        *num = *num*10 + *ptr-'0';
        ++ptr;
    }
    if(*ptr=='.')
    {
        ++ptr;
        double devider = 10;
        while(isNumeric(*ptr))
        {
            *num += (*ptr-'0')/devider;
            devider *= 10;
            ++ptr;
        }
    }
    *num *= sign;
    return ptr - ptr_start;
}


int32_t parse_int32(uint8_t *ptr)
{
    int num = 0, sign = 1;
    if(*ptr == '-')
    {
        ++ptr;
        sign = -1;
    }
    while(isNumeric(*ptr))
    {
        num = num*10 + *ptr-'0';
        ++ptr;
    }
    num*=sign;
    //debug("num=%d\t",num);
    return num;
}

void comm_parse_velocity(uint8_t *msg)
{
    uint8_t *ptr = msg;
    double linear,angular;
    while ( *ptr != ',') ++ptr;
    if(parse_double(msg,&linear)    < 8) return;
    if(parse_double(++ptr,&angular) < 8 ) return;

    int32_t left,right;
    #define WHEEL_RADIUS (175/2/1000.)
    #define WHEELS_DISTANCE (3040/1000.)
    left =  (linear - angular * (WHEELS_DISTANCE/2.)) * engines_get_max_speed();
    right = (linear + angular * (WHEELS_DISTANCE/2.)) * engines_get_max_speed();

    debug("in=%s\t",msg);
    debug("l=%d\ta=%d\t", (int)(linear*1000), (int)(angular*1000));
    debug("L=%d\tR=%d\t", left, right);
    debug("\n");
    engines_set_speed((int)left,(int)right);

    /*uint8_t *ptr = msg;
    while ( *ptr != ',') ++ptr;
    int32_t left,right;
    left = parse_int32(msg);
    right = parse_int32(++ptr);
    debug("L=%d\tR=%d\t", left, right);
    engines_set_speed(left,right);*/
}

void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        uint8_t c = USART_ReceiveData(USART3);
        if(RX_SIZE < 255)
        {
            comm_rx_buf[comm_rx_in++] = c;
            if(c=='}')
            {
                ++comm_new_message_waiting;
            }
        }
    }
    if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
    {
        USART_SendData(USART3, comm_tx_buf[comm_tx_out++]);
        if(TX_SIZE == 0)
        {
            USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
        }
    }
}
