#include "51test.h"

void  main(int32_t argc , int8_t **argv)
{
}

#include <REGX52.H>
#include "DELAY.h"
#define uchar unsigned char
#define uint unsigned int
uchar code seg[11]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x40};//分别对应 0~9和“-”的段码
uchar code dis[]={0x1c,0x18,0x14,0x10,0x0c,0x08,0x04,0x00};//位码
uchar hour=23,min=59,second=55;con=0;
static work_mode=0;

void T0_init()   //定时器初始化函数
{
	TMOD &= 0xF0;		//设置定时器模式
	TMOD |= 0x01;		//设置定时器模式
	TL0 = 0x18;		//设置定时初始值
	TH0 = 0xFC;		//设置定时初始值
	TF0 = 0;		//清除TF0标志
	TR0 = 1;		//定时器0开始计时
	ET0 = 1;        //定时器0中断打开
	EA = 1;         //总中断打开
}
void T0_ET0()interrupt 1  //中断
{
	TH0=0XFC;
	TL0=0X18;
	if(1000==++con)//1000次一毫秒为一秒
	{
		 con=0;
		 if(60==++second)   //秒
		 {
				 second=0;     
				 if(60==++min)  //分
				 {
					   min=0;
						 if(24==++hour) //时
							 hour=0;
				 }
		 }
	}
}
void clock()//时钟显示函数
{   
		P2=dis[0];P0=seg[hour/10];Delay(2);//时高位
		P2=dis[1];P0=seg[hour%10];Delay(2);//时低位
		P2=dis[2];P0=seg[10];Delay(2);//显示“-”
			
		P2=dis[3];P0=seg[min/10];Delay(2);//分高位
		P2=dis[4];P0=seg[min%10];Delay(2);//分低位
		P2=dis[5];P0=seg[10];Delay(2);//显示“-”
			
		P2=dis[6];P0=seg[second/10];Delay(2);//秒高位
		P2=dis[7];P0=seg[second%10];Delay(2);//秒低位
}

void alarm_clock()//闹钟显示“11-11-11”
{
		P2=dis[0];P0=seg[1];Delay(2);
		P2=dis[1];P0=seg[1];Delay(2);
		P2=dis[2];P0=seg[10];Delay(2);
			
		P2=dis[3];P0=seg[1];Delay(2);
		P2=dis[4];P0=seg[1];Delay(2);
		P2=dis[5];P0=seg[10];Delay(2);
			
		P2=dis[6];P0=seg[1];Delay(2);
		P2=dis[7];P0=seg[1];Delay(2);
}

void key_down()//按键按下 获取键值
{
		uchar key=0;
		P1=0xFF;
		P1_3=0;
		if(P1_7==0)
		{
				Delay(10);
            if(0==P1_7 && 0 ==work_mode)
            {
                    work_mode=1;
            }else if(1==work_mode && 0==P1_7)
            {
                    work_mode=0;
            }
        }
}
//void key_scan()
//{

//P1=0xFF;
//		P1_3=0;
//		if(P1_7==0)
//		{
//				Delay(10);if(P1_7=0);mode=~mode;
//		}


//}


void main()
{

	T0_init();   //初始化定时计数器
	while(1)
	{
			key_down();
				switch(work_mode)
					{
						case 0:
							clock();
							break;
						case 1:
							alarm_clock();
						  break;
						default:
							clock();
						   
					}
	}
}

//			if(!mode)   //模式0显示时钟
//			{
//					clock();
//			}
//			if(mode)   //模式1显示闹钟
//			{
//				alarm_clock();
//			}	
//void key_scan()//扫描函数
//{
////	r1=0;r2=r3=r4=1;
////	c1=1;
//	  P1=0xFF;
//		P1_3=0;
//		if(P1_7==0)
//		{
//				Delay(10);if(P1_7=0);mode=~mode;
//		}
////	if(c1==0)//持续扫描矩阵按键一
////	{
////		Delay(10);//消抖
////		if(c1==0)
////		{
////			mode=~mode;
////		}
////	}	
//}
