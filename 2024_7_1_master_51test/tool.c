#include "51test.h"


#ifdef UART_FUNC

void uart_init(uint8_t baud)
{
    TMOD| = 0X20;   //计数器工作方式
    SCOM = 0X50;  //设置工作方式1
    PCON = 0X80;  //波特率加倍
    TH1 = baud;   //计数器初始值设置
    TL1 = baud;
    ES = 1;     //接受中断开启
    EA = 1;     //打开总中断
    TR1 = 1;    //打开计数器
}

void send_buffer(uint8_t byte)
{
    SUBF = byte;   //数据存入缓存区
    while(!TI);   //发送置1 等待发送完成 发送完成置0
    TI = 0;
} 

void uart_print(const uint8_t *format, ...)
{
    
}

#endif