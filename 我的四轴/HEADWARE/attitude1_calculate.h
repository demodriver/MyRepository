#ifndef _ATTITUDE_CACULATE_H
#define _ATTITUDE_CACULATE_H

#include "spi.h"
#include "stm32f10x.h"
#include "math.h"

typedef struct
{
	float ROL;
	float PIT;
	float YAW;
}Out_Angel;

extern Out_Angel Q_ANGLE; //вНжувкл╛╫г
void attitude_calculate(MPU_value current_value);

#endif
