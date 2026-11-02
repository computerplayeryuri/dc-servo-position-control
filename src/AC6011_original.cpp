// AC6011.cpp 
#include "stdafx.h" 
#include "conio.h"
#include "windows.h" 
#include "stdio.h"
#include "io_nt.h"
#include "math.h"
#include "AC6011.h"
HINSTANCE hDLL; //DLL句柄
HANDLE NTportio=NULL;  //NT IO驱动 句柄--->暂定为全局变量，自定义时，可以自己传递参数
unsigned long idd[100];

void DA_PMW(double u);	//声明输出函数
double lb();	//声明滤波函数

void main()
{
unsigned long cardc;	
char *dll_name="acpci.dll";//动态连接库名
hDLL=LoadLibrary(dll_name); //调入DLL ，动态调用
#include "ac6011_func.h"
#include "io_nt_func.h"
NTportio=AC_OpenDriver(); //获取IO驱动句柄			
cardc=AC_6011_INIT(NTportio,idd);//初始化AC6011卡

//主函数编辑区域

int a=0,n=0,m=0;	//定义参数
double Kp,Ki,Kd;
double error=0,last_error=0,pre_error=0;
double x0=0,x1=0,output_pmw=0,Volt=0,goal=0,s=0,T=0,t=0;

printf("请选择输入信号:1.阶跃2.正弦3.方波\n");
scanf("%d",&a);
FILE *control_data;
//打开并写入文档
	if ((control_data = fopen("d:\\control_data.txt", "w")) == NULL) {
		printf("\nERROR:Can't open file,press any key to return!");
		exit(0);
	}
	printf("请您输入位移");
	scanf("%lf", &s);
	if (a != 1) {
		printf("请您输入周期");
		scanf("%lf", &T);
	}
	printf("请您输入比例参数");
	scanf("%lf", &Kp);
	printf("请您输入积分参数");
	scanf("%lf", &Ki);
	printf("请您输入微分参数");
	scanf("%lf", &Kd);
	x0 = lb();//初始位置
	do {
		//阶跃?
		if (a==1){
			if (t>1){
				goal=s;
			}
			t=t+0.1;
		}
		//正弦
		else if (a==2){
			goal=s+10+s*sin(2*3.1415926*t/T);
			t=t+0.1;
		}
		//方波
		else if(a==3){
			n++;
			if (n==10*T){
				m++;
				n=0;
			}
			if (m%2==0){
				goal=s+20;
			}
			else {
				goal=20;
			}
		}	
		x1=lb();	//当前位置
		error=goal-(x1-x0);	//需要运动的位置
		output_pmw+=Kp*(error-last_error)+Ki*error+Kd*(error-2*last_error+pre_error);	//增量式pid算法
		pre_error=last_error;	//将第k-1次的误差赋给第k-2次
		last_error=error;		//将第k次的误差赋给第k-1次
		Volt=(output_pmw/400.0)*10;	//输出电压
		//限幅
		if (Volt>=9.9){
			Volt=9.9;
		}
		if (Volt<=-9.9){
			Volt=-9.9;
		}
		//死区
		if (Volt>=0 && Volt<=0.88){
			Volt=Volt+1.0;
		}
		if (Volt<0 && Volt>-0.38){
			Volt=Volt-1.0;
		}
		DA_PMW(Volt);	//将电压信息输出给电机
		fprintf(control_data, "%lf %lf\n", x1 - x0, goal);	//写入文件
		printf("\n当前位移%lf 给定位移%lf output_pmw=%lf", x1 - x0, goal, output_pmw);
	} while (kbhit() == 0);
	DA_PMW(-8.0);	//电机复位
	fclose(control_data);	//关闭文件
//主函数编辑区域
AC_CloseDriver(NTportio);//关闭AC6011卡驱动
FreeLibrary(hDLL);//清DLL
getch();
}
//子函数编辑区域

//采样子程序
double AD(){
	double ADDATA,U,POS;
	ADDATA=AC_6011_AD(NTportio,idd[0],14,1);
	U=ADDATA*5/4095.0;
	POS=U*400/5.01;
	return POS;
}
//滤波子程序
double lb(){	//采用平均滤波算法
	double i = 0, sum = 0, max=-100000, min=100000;
	double wy, POS_2;
	for (i = 0; i < 258 ; i++){
		wy = AD();
        if (max<wy)
		{
			max=wy;
		}
		if (min>wy)
		{
			min=wy;
		}
		sum = wy + sum;
	}
	POS_2 = (sum-min-max) / 256 ;
	return(POS_2);
}


//输出子程序
void DA_PMW(double Volt){
int DADATA;
DADATA=int(2048*(Volt/10)+2048);
AC_6011_DA(NTportio,idd[0],1,DADATA<<4);
}

