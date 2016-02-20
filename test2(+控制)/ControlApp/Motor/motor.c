#include "motor.h"

#define Moto_PwmMax 999

int16_t MOTO1_PWM = 0;
int16_t MOTO2_PWM = 0;
int16_t MOTO3_PWM = 0;
int16_t MOTO4_PWM = 0;

void Moto_PwmRflash(int16_t MOTO1_PWM,int16_t MOTO2_PWM,int16_t MOTO3_PWM,int16_t MOTO4_PWM)
{		
	if(MOTO1_PWM>Moto_PwmMax)	MOTO1_PWM = Moto_PwmMax;
	if(MOTO2_PWM>Moto_PwmMax)	MOTO2_PWM = Moto_PwmMax;
	if(MOTO3_PWM>Moto_PwmMax)	MOTO3_PWM = Moto_PwmMax;
	if(MOTO4_PWM>Moto_PwmMax)	MOTO4_PWM = Moto_PwmMax;
	if(MOTO1_PWM<0)	MOTO1_PWM = 0;
	if(MOTO2_PWM<0)	MOTO2_PWM = 0;
	if(MOTO3_PWM<0)	MOTO3_PWM = 0;
	if(MOTO4_PWM<0)	MOTO4_PWM = 0;
	
//M1
	TIM_SetCompare4(TIM2,MOTO4_PWM );//LED0 =!LED0;
// 	 printf("1--%.1f\r\n",M_deal4);

//M2
	TIM_SetCompare2(TIM2,MOTO1_PWM );//LED1 =!LED1;
// 	printf("2--%.1f\r\n",M_deal2);

//M3
	TIM_SetCompare1(TIM2,MOTO2_PWM );//LED2 =!LED2;
// 	printf("3--%.1f\r\n",M_deal1);

//M4
	TIM_SetCompare3(TIM2,MOTO3_PWM   );//LED3 =!LED3;
// 	printf("4--%.1f\r\n",M_deal3);
}


