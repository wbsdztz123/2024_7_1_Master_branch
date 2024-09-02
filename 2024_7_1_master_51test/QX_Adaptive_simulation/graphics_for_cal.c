#include "graphics_for_cal.h"
void graphics()
{
    initgraph(800, 700);  //初始化窗口，也就是程序框大小
	setbkcolor(WHITE);      //设置背景颜色
	setlinecolor(RED);    //设置坐标轴的颜色
	cleardevice();       //清除屏幕内容
	setorigin(400, 350);    //设定坐标原点
	line(-400, 00, 400, 00);    //绘制X轴
	line(0, 350, 0, -350);  //绘制Y轴

		for (int i = -500; i <= 500; i++)     //绘制坐标
	{
		line(Amp_ratio  * i, 0, Amp_ratio * i, -10);
		if (i % 5 == 0)
			line(Amp_ratio * i, 0, Amp_ratio * i, -15);
		line(0, Amp_ratio * i, 10, Amp_ratio * i);
		if (i % 5 == 0)
			line(0, Amp_ratio * i, 15, Amp_ratio * i);
	}
	double x,y;
	for(x=-100;x<=100;x=x+0.001)
	{
		y = x * x + x + 2;        //计算出每个X对应的Y值
		putpixel(Amp_ratio * x, -Amp_ratio * y, BLACK);  //画出每个点
	}
	system("pause");
	return 0;

}