/// @file MotorCtl.h
/// @brief Driver PWM generation definition
#ifndef _MOTORCTRL_H_
#define _MOTORCTRL_H_
#include "CommonType.h"

tBOOL motorI_Init(void);
void  motorI_SetPwmOutput(tSI16 set_duty_val);

#endif /* _MOTORCTRL_H_ */
