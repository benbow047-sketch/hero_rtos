/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rm_task.h"
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

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* USER CODE BEGIN RTOS_THREADS */

#ifdef BSP_USING_INS_TASK
  osThreadId_t InsTaskHandle;
  const osThreadAttr_t InsTask_attr = {
      .name = "InsTask", .stack_size = 2048*4, .priority = osPriorityHigh,
  };
  InsTaskHandle = osThreadNew(InsTask_Entry, NULL, &InsTask_attr);
#endif

#ifdef BSP_USING_MOTOR_TASK
  osThreadId_t MotorTaskHandle;
  const osThreadAttr_t MotorTask_attr = {
      .name = "MotorTask", .stack_size = 2048*4, .priority = osPriorityIdle,
  };
  MotorTaskHandle = osThreadNew(MotorTask_Entry, NULL, &MotorTask_attr);
#endif

#ifdef BSP_USING_CMD_TASK
  osThreadId_t CmdTaskHandle;
  const osThreadAttr_t CmdTask_attr = {
      .name = "CmdTask", .stack_size = 2048*4, .priority = osPriorityHigh,
  };
  CmdTaskHandle = osThreadNew(CmdTask_Entry, NULL, &CmdTask_attr);
#endif

#ifdef BSP_USING_CHASSIS_TASK
  osThreadId_t ChassisTaskHandle;
  const osThreadAttr_t ChassisTask_attr = {
      .name = "ChassisTask", .stack_size = 1024*4, .priority = osPriorityHigh,
  };
  ChassisTaskHandle = osThreadNew(ChassisTask_Entry, NULL, &ChassisTask_attr);
#endif

#ifdef BSP_USING_GIMBAL_TASK
  osThreadId_t GimbalTaskHandle;
  const osThreadAttr_t GimbalTask_attr = {
      .name = "GimbalTask", .stack_size = 1024*4, .priority = osPriorityHigh,
  };
  GimbalTaskHandle = osThreadNew(GimbalTask_Entry, NULL, &GimbalTask_attr);
#endif

#ifdef BSP_USING_SHOOT_TASK
  osThreadId_t ShootTaskHandle;
  const osThreadAttr_t ShootTask_attr = {
      .name = "ShootTask", .stack_size = 1024*4, .priority = osPriorityHigh,
  };
  ShootTaskHandle = osThreadNew(ShootTask_Entry, NULL, &ShootTask_attr);
#endif

#ifdef BSP_USING_TRANSMISSION_TASK
  osThreadId_t TransmissionTaskHandle;
  const osThreadAttr_t TransmissionTask_attr = {
      .name = "TransTask", .stack_size = 1024*4, .priority = osPriorityHigh,
  };
  TransmissionTaskHandle = osThreadNew(TransmissionTask_Entry, NULL, &TransmissionTask_attr);
#endif

#ifdef BSP_USING_USARTREC_TASK
  osThreadId_t USARTRecTaskHandle;
  const osThreadAttr_t USARTRecTask_attr = {
      .name = "USARTRecTask", .stack_size = 2048*4, .priority = osPriorityHigh,
  };
  USARTRecTaskHandle = osThreadNew(USARTRecTask_Entry, NULL, &USARTRecTask_attr);
#endif

#ifdef BSP_USING_REFEREE_TASK
  osThreadId_t RefereeTaskHandle;
  const osThreadAttr_t RefereeTask_attr = {
      .name = "RefereeTask", .stack_size = 2048*4, .priority = osPriorityHigh,
  };
  RefereeTaskHandle = osThreadNew(RefereeTask_Entry, NULL, &RefereeTask_attr);
#endif

#ifdef BSP_USING_SUPERCAP_TASK
  osThreadId_t SupercapTaskHandle;
  const osThreadAttr_t SupercapTask_attr = {
      .name = "Supercap", .stack_size = 768*4, .priority = osPriorityIdle,
  };
  SupercapTaskHandle = osThreadNew(Supercap_Entry, NULL, &SupercapTask_attr);
#endif

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Application */

/* USER CODE END Application */
