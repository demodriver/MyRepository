#ifndef _ATTITUDE_CACULATE_H
#define _ATTITUDE_CACULATE_H

#include "spi.h"
#include "stm32f10x.h"
#include "math.h"

//extern MPU_value current_value;//µ±Ç°Öµ

void attitude_calculate(MPU_value current_value);

#endif
