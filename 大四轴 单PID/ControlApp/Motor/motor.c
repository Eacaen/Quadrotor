#include "motor.h"
#include "timer.h"
#include "usart.h"
#include "pwm.h"
#include "led.h"
/*********************************************************
The code
Thanks for  @	��ӳ��
All rights Reserve
*********************************************************/

int Power= 0;			//����

//MOTOR Drive

static float M_deal1,M_deal2,M_deal3,M_deal4;
void MM_Drive(float x_data,float y_data,float z_data)
{
	
	x_data = -x_data;
		

	M_deal1 = Power + x_data - y_data + z_data;			//1 �ŵ��
	M_deal2 = Power + x_data + y_data - z_data*1.5f;	//2 �ŵ�� 
	M_deal3 = Power - x_data + y_data + z_data;			//3 �ŵ��
	M_deal4 = Power - x_data - y_data - z_data*1.5f;	//4 �ŵ��
 
	
	
	if(M_deal1<=0)M_deal1=0;
	if(M_deal1>=999)M_deal1=999;
	if(M_deal2<=0)M_deal2=0;
	if(M_deal2>=999)M_deal2=999;
	if(M_deal3<=0)M_deal3=0;
	if(M_deal3>=999)M_deal3=999;
	if(M_deal4<=0)M_deal4=0;
	if(M_deal4>=999)M_deal4=999;	  
//M1
	TIM_SetCompare4(TIM2,M_deal4*1.1 );

//M2
	TIM_SetCompare2(TIM2,M_deal1 );

//M3
	TIM_SetCompare1(TIM2,M_deal2 );
	
//M4
	TIM_SetCompare3(TIM2,M_deal3);

}

