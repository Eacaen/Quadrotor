#include "allinclude.h"

/*********************************************************
The code
Thanks for 匿名
All rights Reserve
*********************************************************/
unsigned char FLY=1;		//可以起飞的指令
extern EularAngle EA;
extern T_RC_Data Rc_D;
extern Gyro gyro;
PID PID_ROL,PID_PIT,PID_YAW,PID_ALT,PID_POS;
 
int16_t getlast_roll=0,geilast_pitch=0;


float rol_i=0,pit_i=0,yaw_p=0;
vs16 Moto_PWM_1=0,Moto_PWM_2=0,Moto_PWM_3=0,Moto_PWM_4=0;

 
void Control()
{
	EularAngle angle;
	angle.Roll = EA.Roll - (Rc_D.ROLL-1500)/12;
	angle.Pitch = EA.Pitch + (Rc_D.PITCH-1500)/12;
	
	rol_i += angle.Roll;
	if(rol_i>2000)
	rol_i=2000;
	if(rol_i<-2000)
	rol_i=-2000;

	PID_ROL.pout = PID_ROL.P * angle.Roll;
	PID_ROL.dout = -PID_ROL.D * gyro.y;
	PID_ROL.iout = PID_ROL.I * PID_ROL.dout;

	pit_i += angle.Pitch;
	if(pit_i>2000)
	pit_i=2000;
	if(pit_i<-2000)
	pit_i=-2000;

	PID_PIT.pout = PID_PIT.P * angle.Pitch;
	PID_PIT.dout = PID_PIT.D * gyro.x;
	PID_PIT.iout = PID_PIT.I * pit_i;
	
	if(Rc_D.YAW<1400||Rc_D.YAW>1600)
	{gyro.z=gyro.z+(Rc_D.YAW-1500)*2;}
	
	yaw_p+=gyro.z*0.0609756f*0.002f;// +(Rc_Get.YAW-1500)*30
	
	if(yaw_p>20)
		yaw_p=20;
	if(yaw_p<-20)
		yaw_p=-20;


	PID_YAW.pout=PID_YAW.P*yaw_p;
	PID_YAW.dout = PID_YAW.D * gyro.z;				   
	
	if(Rc_D.THROTTLE<1200)
	{		
		pit_i=0;
		rol_i=0;
		yaw_p=0;
	}

	PID_ROL.OUT =  (-PID_ROL.pout)-PID_ROL.iout +PID_ROL.dout;//
	PID_PIT.OUT = PID_PIT.pout + PID_PIT.iout + PID_PIT.dout;
	PID_YAW.OUT = PID_YAW.pout + PID_YAW.iout + PID_YAW.dout;
 
	if(Rc_D.THROTTLE>1200)
	{
		Moto_PWM_1 = Rc_D.THROTTLE - 1000 + PID_ROL.OUT - PID_PIT.OUT - PID_YAW.OUT;
		Moto_PWM_2 = Rc_D.THROTTLE - 1000 + PID_ROL.OUT + PID_PIT.OUT + PID_YAW.OUT;
		Moto_PWM_3 = Rc_D.THROTTLE - 1000 - PID_ROL.OUT + PID_PIT.OUT - PID_YAW.OUT;
		Moto_PWM_4 = Rc_D.THROTTLE - 1000 - PID_ROL.OUT - PID_PIT.OUT + PID_YAW.OUT;
	}
	else
	{
		Moto_PWM_1 = 0;
		Moto_PWM_2 = 0;
		Moto_PWM_3 = 0;
		Moto_PWM_4 = 0;
	}
 	Moto_PwmRflash(Moto_PWM_1,Moto_PWM_2,Moto_PWM_3,Moto_PWM_4);
}


