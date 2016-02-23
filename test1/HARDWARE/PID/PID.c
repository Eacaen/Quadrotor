#include "pid.h"
#include "timer.h"
#include "usart.h"
#include "suanfa.h"
#include "motor.h"
/*********************************************************
The code
Thanks for @ 陈映冰
All rights Reserve
*********************************************************/
unsigned char FLY=1;		//可以起飞的指令
extern float cord[3];	

float KP=4.5f;
float TD=0.15f;				

float Target_x=0;			    //目标值
float Target_y=0;
float Target_yaw=0;			   

static float X_Sum_Out;				//Sum_Out	X
static float Y_Sum_Out;				//Sum_Out	Y
static float YAW_Sum_Out;				//Sum_Out	YAW

void PID_Deal(float *cord)
{
	//定义变量
	static float ERROR_X[2]={0,0};		 	 	 //N个时刻数据记录
	static float ERROR_Y[2]={0,0};		 		 //N个时刻数据记录
	static float ERROR_YAW[2]={0,0};			 //N个时刻数据记录
	static float PID_X_Data[2]={0,0};		 	 //2个状态数据
	static float PID_Y_Data[2]={0,0};			 //2个状态数据
	static float PID_YAW_Data[2]={0,0};			 //两个状态数据
	/*------------------------------------MOTOR  PREPARE-----------------------------------*/	
	ERROR_X[1]=ERROR_X[0];						//堆栈记录	error X
	ERROR_X[0]=(-cord[0]-Target_y);	
	ERROR_Y[1]=ERROR_Y[0];						//堆栈记录	error Y
	ERROR_Y[0]=(cord[1]-Target_x);

	ERROR_YAW[1]=ERROR_YAW[0];					//堆栈记录	error YAW	  
	ERROR_YAW[0]=(cord[2]-Target_yaw);						

	if(ERROR_YAW[0]>180)	ERROR_YAW[0]-=360;	//校正防 0°->360°跳变		逆时针减小 顺时针增大
	if(ERROR_YAW[0]<-180)	ERROR_YAW[0]+=360;
	//////////////////////////////////////////////////////////////////////////////////X
	PID_X_Data[0]=ERROR_X[0];									//p							   					   
	PID_X_Data[1]=(ERROR_X[0]-ERROR_X[1])/tine;					//d
	//////////////////////////////////////////////////////////////////////////////////Y
	PID_Y_Data[0]=ERROR_Y[0];									//p					   
	PID_Y_Data[1]=(ERROR_Y[0]-ERROR_Y[1])/tine;					//d
	//////////////////////////////////////////////////////////////////////////////////YAW
	PID_YAW_Data[0]=ERROR_YAW[0];								//p
	//PID_YAW_Data[1]=(ERROR_YAW[0]-ERROR_YAW[1])/tine;			//d
	
	//数据处理
	X_Sum_Out = KP*(PID_X_Data[0]+PID_X_Data[1]*TD); 			// 
	Y_Sum_Out  =KP*(PID_Y_Data[0]+PID_Y_Data[1]*TD);			// 
	YAW_Sum_Out=KPYAW * PID_YAW_Data[0];
		
	/*------------------------------------MOTOR DDDEEAL-----------------------------------*/	
	//方向符号 剪头方向为负   
	if(FLY){
		MM_Drive(X_Sum_Out,Y_Sum_Out,YAW_Sum_Out);
	}else{											   
		MM_Set(0);
	}
	//printf(" %f | %f;",Target_x,Target_y);  前：y- 右：x-	 	cord[0]:x 箭头方向是-
	//printf(" %f ;",ERROR_YAW[0]);
	
}

