/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2021 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "MotorCtl.h"
#include "adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
tUI32 volumeAdcValue_gdu32               = 0;
tUI8  volumeAdcStartupCounter            = 0;
tUI8  volumeAdcStartupErrorStatus_gdu8 = 0;

tUI32 motorPwmDutySet_gdu32 = 0;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId motorCtrlTaskHandle;
osThreadId ledDispTaskHandle;
osThreadId volumeReadTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void startupErrorHandler(void);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void startMotorCtrlTask(void const * argument);
void startLedDispTask(void const * argument);
void startVolumeReadTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of motorCtrlTask */
  osThreadDef(motorCtrlTask, startMotorCtrlTask, osPriorityHigh, 0, 128);
  motorCtrlTaskHandle = osThreadCreate(osThread(motorCtrlTask), NULL);

  /* definition and creation of ledDispTask */
  osThreadDef(ledDispTask, startLedDispTask, osPriorityNormal, 0, 128);
  ledDispTaskHandle = osThreadCreate(osThread(ledDispTask), NULL);

  /* definition and creation of volumeReadTask */
  osThreadDef(volumeReadTask, startVolumeReadTask, osPriorityAboveNormal, 0, 128);
  volumeReadTaskHandle = osThreadCreate(osThread(volumeReadTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  TickType_t xNextWakeTime;
  const TickType_t xFrequency = 100u; // 100ms
  /* Infinite loop */
  for(;;)
  {
    // Only active once the error detected at starting up
    if (9 == volumeAdcStartupErrorStatus_gdu8) {
      HAL_GPIO_TogglePin(LED_DISP_GPIO_Port, LED_DISP_Pin);
    }
    vTaskDelayUntil(&xNextWakeTime, xFrequency);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_startMotorCtrlTask */
/**
* @brief Function implementing the motorCtrlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startMotorCtrlTask */
void startMotorCtrlTask(void const * argument)
{
  /* USER CODE BEGIN startMotorCtrlTask */
  TickType_t xNextWakeTime;
  const TickType_t xFrequency = 10u; // 10ms
  
  
  // Motor control initialize
  motorI_Init();
  /* Infinite loop */
  for(;;)
  {
    // Check for error
    if (9 == volumeAdcStartupErrorStatus_gdu8) break;
    else if (0 == volumeAdcStartupErrorStatus_gdu8) { /* Start up check, waiting for valid */ }
    else {
      // Calculate duty according to adc value read
      motorPwmDutySet_gdu32 = (volumeAdcValue_gdu32 * 500) / 4096;
      // Direct motor export
      motorI_SetPwmOutput((tUI16)motorPwmDutySet_gdu32);
    }
    vTaskDelayUntil(&xNextWakeTime, xFrequency);
  }
  startupErrorHandler();
  /* USER CODE END startMotorCtrlTask */
}

/* USER CODE BEGIN Header_startLedDispTask */
/**
* @brief Function implementing the ledDispTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startLedDispTask */
void startLedDispTask(void const * argument)
{
  /* USER CODE BEGIN startLedDispTask */
  TickType_t xNextWakeTime;
  const TickType_t xFrequency = 5u; // 5ms
  
  GPIO_PinState pin_status = GPIO_PIN_SET;
  tUI16 led_disp_counter = 0;
  tUI8  led_on_max_count = motorPwmDutySet_gdu32 / 5;
  tUI8  led_on_counter   = 0;
  /* Infinite loop */
  for(;;)
  {
    // Check for error
    if (9 == volumeAdcStartupErrorStatus_gdu8) break;
    else if (0 == volumeAdcStartupErrorStatus_gdu8) { /* Start up check, waiting for valid */ }
    else {
      // Pin state handle update every 0.5s
      if (led_disp_counter++ >= 100) {
        led_disp_counter = 0;
        // Update duty recalcualte display
        led_on_max_count = motorPwmDutySet_gdu32 / 5;
      } else {
        // Keep duty and display
        led_on_counter++;
        if (led_on_counter >= (led_disp_counter)) {
          led_on_counter = 0;
        }
        else if (led_on_counter >= led_on_max_count) {
          // Off led -> positive low
          pin_status = GPIO_PIN_SET;
        }
        else {
          // On led  -> positive low
          pin_status = GPIO_PIN_RESET;
        }
      }
      // Export led state handle
      HAL_GPIO_WritePin(LED_DISP_GPIO_Port, LED_DISP_Pin, pin_status);
    }
    vTaskDelayUntil(&xNextWakeTime, xFrequency);
  }
  startupErrorHandler();
  /* USER CODE END startLedDispTask */
}

/* USER CODE BEGIN Header_startVolumeReadTask */
/**
* @brief Function implementing the volumeReadTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startVolumeReadTask */
void startVolumeReadTask(void const * argument)
{
  /* USER CODE BEGIN startVolumeReadTask */
  TickType_t xNextWakeTime;
  const TickType_t xFrequency = 10u; // 10ms

  // Calib
  HAL_ADCEx_Calibration_Start(&hadc1);

  /* Infinite loop */
  for(;;)
  {
    // Start ADC Conversion
    HAL_ADC_Start_IT(&hadc1);
    // Check for error start up
    if (9 == volumeAdcStartupErrorStatus_gdu8) {
      // No longer request adc conversion
      HAL_ADC_Stop_IT(&hadc1);
      break;
    }

    vTaskDelayUntil(&xNextWakeTime, xFrequency);
  }
  startupErrorHandler();
  /* USER CODE END startVolumeReadTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  static tUI32 sumupAdcRaw = 0;
  // Read & Update The ADC Result
  volumeAdcValue_gdu32 = HAL_ADC_GetValue(&hadc1);
  // Checking first 5 cycle
  if (volumeAdcStartupCounter < 5) {
    volumeAdcStartupCounter++;
    sumupAdcRaw += volumeAdcValue_gdu32;
    // Checking each sample, 100% equal to 4096, let the valid data lower than 50%
    // In first five cycles, each sampling data should not greater than 2000
    // Valid some noise, and first five sum up  should not greater than 2000x5 = 10000
    if (sumupAdcRaw >= 10000) volumeAdcStartupErrorStatus_gdu8 = 9;
    else volumeAdcStartupErrorStatus_gdu8 = 0;
    
  } else if ((volumeAdcStartupCounter >= 5) && (0 == volumeAdcStartupErrorStatus_gdu8)) {
    /* Valid first 5 sample -> start normal control */
    volumeAdcStartupErrorStatus_gdu8 = 1;
  } else { /* Do no thing */ }
}

void startupErrorHandler (void) {
  while (1) { 
    /* Infinite loop n do nothing but need release resource */
    osDelay(1000);
  }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
