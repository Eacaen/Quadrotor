#include "pid.h"
#include "timer.h"
#include "usart.h"
#include "suanfa.h"
#include "motor.h"
/*********************************************************
The code
Thanks for @ ��ӳ��
All rights Reserve
*********************************************************/
unsigned char FLY=1;		//������ɵ�ָ��
extern float cord[3];	

float KP=4.5f;
float TD=0.15f;				

float Target_x=0;			    //Ŀ��ֵ
float Target_y=0;
float Target_yaw=0;			   

static float X_Sum_Out;				//Sum_Out	X
static float Y_Sum_Out;				//Sum_Out	Y
static float YAW_Sum_Out;				//Sum_Out	YAW

void PID_Deal(float *cord)
{
	//�������
	static float ERROR_X[2]={0,0};		 	 	 //N��ʱ�����ݼ�¼
	static float ERROR_Y[2]={0,0};		 		 //N��ʱ�����ݼ�¼
	static float ERROR_YAW[2]={0,0};			 //N��ʱ�����ݼ�¼
	static float PID_X_Data[2]={0,0};		 	 //2��״̬����
	static float PID_Y_Data[2]={0,0};			 //2��״̬����
	static float PID_YAW_Data[2]={0,0};			 //����״̬����
	/*------------------------------------MOTOR  PREPARE-----------------------------------*/	
	ERROR_X[1]=ERROR_X[0];						//��ջ��¼	error X
	ERROR_X[0]=(-cord[0]-Target_y);	
	ERROR_Y[1]=ERROR_Y[0];						//��ջ��¼	error Y
	ERROR_Y[0]=(cord[1]-Target_x);

	ERROR_YAW[1]=ERROR_YAW[0];					//��ջ��¼	error YAW	  
	ERROR_YAW[0]=(cord[2]-Target_yaw);						

	if(ERROR_YAW[0]>180)	ERROR_YAW[0]-=360;	//У���� 0��->360������		��ʱ���С ˳ʱ������
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
	
	//���ݴ���
	X_Sum_Out = KP*(PID_X_Data[0]+PID_X_Data[1]*TD); 			// 
	Y_Sum_Out  =KP*(PID_Y_Data[0]+PID_Y_Data[1]*TD);			// 
	YAW_Sum_Out=KPYAW * PID_YAW_Data[0];
		
	/*------------------------------------MOTOR DDDEEAL-----------------------------------*/	
	//������� ��ͷ����Ϊ��   
	if(FLY){
		MM_Drive(X_Sum_Out,Y_Sum_Out,YAW_Sum_Out);
	}else{											   
		MM_Set(0);
	}
	//printf(" %f | %f;",Target_x,Target_y);  ǰ��y- �ң�x-	 	cord[0]:x ��ͷ������-
	//printf(" %f ;",ERROR_YAW[0]);
	
}

