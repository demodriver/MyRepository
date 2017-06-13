#ifndef _RC_H
#define _RC_H
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "Control.h"

#define RC_FUN_MIN	800
#define RC_FUN_MAX	1500

void Rc_GetValue(T_RC_Data *temp);
void Rc_Fun(T_RC_Data *rc_in,T_RC_Control *rc_ct);
#endif

