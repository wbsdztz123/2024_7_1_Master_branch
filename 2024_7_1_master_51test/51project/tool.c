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
    for(i = ms;i > 0; i--)
    {
        for(j = 110; j > 0 ; j--);
    }
}

void delay_10us(uint16_t us)
{
    while (us--)
}
#endif

#ifdef EPPROM_FUNC
/* 函数名: I2C起始信号产生
 * 描述：I2C产生起始信号  时钟线SCL处于高电平时，SDA数据线从高电平切换到低电频
 * 返回值：NA
 */
void I2C_start()
{
    IIC_SDA = 1;
    delay_10us(1);
    IIC_SCL = 1;
    delay_10us(1);
    IIC_SDA = 0;  //当SCL为高电平时，SDA由高变为低   起始信号产生
    delay_10us(1);
    IIC_SCL = 0;  //钳住I2C,为接下来的数据传输或其他操作做好准备。
}

/* 函数名: I2C 停止信号产生
 * 描述：I2C 产生停止信号。
 * 返回值：NA
 */
void I2C_stop()
{
    IIC_SDA = 0;
    delay_10us(1);
    IIC_SCL = 1;
    delay_10us(1);
    IIC_SDA = 1;
}
/* 函数名: I2C_Write_onebyte(uint8_t data)
 * 描述：读取一个字节的数据   0xA0 = 1010 0000 1010 000  设备地址  0写操作
 * 返回值：NA
 */
void I2C_Write_onebyte(uint8_t data)
{
    uint8_t i = 0;
    IIC_SCL = 0;
    for(i = 0;i < 8;i++)
    {
        if((data&0x80) > 0)
        {
            IIC_SDA = 1;
        }
        else
        {
            IIC_SDA = 0;
        }
        data<<=1; //左移1位
        delay_10us(10);
        IIC_SCL = 1;   //通知接受数据
        delay_10us(10);
        IIC_SCL = 0;
        delay_10us(10);
    }
}
/* 函数名: I2C_Read_onebyte(uint8_t Ack)
 * 描述：读取一个字节的数据   0xA0 = 1010 0000 1010 000  设备地址  0写操作
          ack == 1
         return ACK
         else
         return NACK
 * 返回值：receive
 */
uint8_t I2C_Read_onebyte(uint8_t ack)
{
    uint8_t i = 0,receive = 0;
    for (i = 0; i < 8; i++)
    {
        IIC_SCL = 0;
        delay_10us(1);
        IIC_SCL = 1;
        receive <<= 1;
        if(IIC_SDA)
        {
            receive++;
        }
    }
    if (ack)
    {
        I2C_ack();
    }else{
        I2C_NACK();
    }
    return receive;
}


/* 函数名: void I2C_ack(void)
 * 描述：I2C_应答信号
 * 返回值：NA
 */
void I2C_Ack(void)
{
	IIC_SCL=0;
	IIC_SDA=0;	//SDA为低电平   成功接受数据  SDA为高电平形成NACK信号
	delay_10us(1);
   	IIC_SCL=1; //读取数据线的状态  SDA为低电平ACK 高电频NACK
	delay_10us(1);
	IIC_SCL=0;  
}
/* 函数名: void I2C_Nack(void)
 * 描述：I2C_非应答信号产生
 * 返回值：NA
 */
void I2C_Nack(void)
{
	IIC_SCL=0;
	IIC_SDA=1;	//SDA为低电平   成功接受数据  SDA为高电平形成NACK信号
	delay_10us(1);
   	IIC_SCL=1; //读取数据线的状态  SDA为低电平ACK 高电频NACK
	delay_10us(1);
	IIC_SCL=0;  
}

/* 函数名: void I2C_Nack(void)
 * 描述：I2C等待应答信号
 * 返回值：NA
 */

#define I2C_STOP 1
#define I2C_SUCCESS 0
uint8_t I2C_Wait_ack(void)
{
    uint8_t time = 0;
    IIC_SCL = 1;
    delay_10us(1);
    while (IIC_SDA)  //1-0
    {
        time++;
        if(time > 100)
        {
            I2C_stop();
            return I2C_STOP;
        }
    }
    IIC_SCL = 0;
    return I2C_SUCCESS;
    

}


/* 函数名: Write_EPPROM(uint8_t address, uint8_t data)
 * 描述：EPPROM写入函数，起始地址0x00，终止地址0xFF  单次写入一个byte 
 * 容量：255个byte
 * 返回值：NA
 */
void Write_EPPROM(uint8_t address, uint8_t data)
{

}

/* 函数名: Read_EPPROM(uint8_t address, uint8_t data)
 * 描述：EPPROM读取函数，起始地址0x00，终止地址0xFF  单次读取一个byte 
 * 容量：255个byte
 * 返回值：NA
 */
void Read_EPPROM(uint8_t address, uint8_t data)
{

}


#endif