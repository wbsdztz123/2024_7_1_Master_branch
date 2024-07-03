#include "51test.h"


#ifdef UART_FUNC
/* 函数名: uart_init(uint8_t baud)
 * 描述：串口初始化
 * 返回值：NA
 */
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

    return; //
}

/* 函数名: send_buffer(uint8_t byte)
 * 描述：数据发送单个byte
 * 返回值：NA
 */

void send_buffer(uint8_t byte)
{
    SUBF = byte;   //数据存入缓存区
    while(!TI);   //发送置1 等待发送完成 发送完成置0
    TI = 0;
    return; 
} 

/* 函数名: uart_print(const uint8_t *format, ...)
 * 描述：串口输出 支持%c，%d，%f，%s
 * 返回值：NA
 */

void uart_print(const uint8_t *format, ...)
{
    //va_list args;  //va_list个指向参数列表的指针,用于访问和存储参数列表
    va_start(args,format);  //初始化可变参数列表
    while (*format !='\0')
    {
        if(*format == '%')
        {
            format++;  //跳过%
            switch (*format)
            {
            case 'c':
                    int8_t c = va_arg(args,int8_t); //参数列表获取char类型数据
                    send_buffer(c);
                break;
            case 'd':
                    int32_t d = va_arg(args,int32_t); //从参数列表获取int类型数据
                    int8_t buffer[20] = {0};
                    sprintf(buffer, "%d",d);
                    for (int8_t i = 0; i != '\0'; i++)
                    {
                        send_buffer(buffer[i]);
                    }
                    
                break;
            case 's':
                    int8_t *str = va_arg(args,int8_t*);
                    while (*str)   //while('\0')
                    {
                        send_buffer(*str++);
                    }
                break;
            case 'f':
                    float32_t f = va_arg(args,float32_t);
                    int8_t buffer[20] = {0};
                    sprintf(buffer, "%f",f);
                    for(int8_t i = 0; i != '\0'; i++)
                    {
                        send_buffer(buffer[i]);
                    }
                break;
            default:
                    send_buffer(*format);
                break;
            }
        }
        else
        {
            send_buffer(*format);
        }
        format++;
    }
    va_end(args);
    return;
}


/* 函数名: delay_ms(uint32_t ms)
 * 描述：延时函数 ms = 1；1ms
 * 返回值：NA
 */
void delay_ms(uint16_t ms)
{
    uint16_t i,j;
    for(i = ms,i > 0; i--)
    {
        for(j = 110; j > 0 ; j--);
    }
}
#endif

#ifdef EPPROM_FUNC

#endif