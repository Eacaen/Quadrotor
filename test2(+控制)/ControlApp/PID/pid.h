#ifndef __PID_H
#define __PID_H
#include "sys.h"
// #include "motor.h"


#define tine 	    0.005f  //ʱ���
#define KPYAW 		2.55f 	//KP YAW 
#define KDYAW 		0.345f	//KD YAW	  buyong
#define I_Top		3.2	  //pid_i range	 top
#define I_Bottom	-3.2  //pid_i range	 top	
#define I_M_Top		0.5  //�ÿ���
#define I_M_Bottom -0.5  //�ÿ���


void PID_Deal(float *cord);


#endif

