#ifndef __PID_H
#define __PID_H
#include "sys.h"
// #include "motor.h"

typedef struct PID
{
	float P,pout,I,iout,D,dout,IMAX,OUT;
}PID;

void PID_Deal(float *cord);


#endif

