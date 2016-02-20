#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"

void MM_Drive(float x_data,float y_data,float z_data);
void Moto_PwmRflash(int16_t MOTO1_PWM,int16_t MOTO2_PWM,int16_t MOTO3_PWM,int16_t MOTO4_PWM);


#endif

