/// @file MotorCtl.c
/// @brief Driver PWM generation manipulate
#include "MotorCtl.h"

#include "tim.h"
#include "gpio.h"

/// @brief Initialize motor pwm generation
tBOOL motorI_Init(void) {
  tSI8 tim_start_status = 0;
  // Set CCW for direction pin
  HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_SET);
  // Initialize motor PWM
  tim_start_status += HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  // Status return
  if (tim_start_status != 0) return eFALSE;
  // Return true if no error
  return eTRUE;
}

/// @brief Motor control PWM out calculation
/* PWM duty cycle manipulate configuration
 * TIM2 belong to APB1 group, clock config is 36MHz
 * -> Frequency expected is 10kHz
 * We have (Fexp = Fclk(36Mhz) / ((ARR+1)_(900)*(PSC+1)_(4)))
 * ARR and PSC configured in the cubemx
 * Setting duty from 0->900 mean 0%->100%
 */
void motorI_SetPwmOutput(tSI16 set_duty_val) {
  tSI16 actual_duty_val = 0;
  // Limit input
  if (set_duty_val >= 500) actual_duty_val = 500;
  else actual_duty_val = set_duty_val;
  // Pwm setting
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, actual_duty_val);
}
