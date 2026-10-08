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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "i2c.h"
#include "usart.h"
#include "imu_filter.h"
#include <stdio.h>
#include <string.h>
#include "OLED.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUF_SIZE  256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
MPU6050_Raw_t raw;
Attitude_t att;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;
uint8_t rx_buf[RX_BUF_SIZE];
uint16_t g_rx_len = 0;
osSemaphoreId UartRxSemHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId UartTaskHandle;
osThreadId ImuTaskHandle;
osThreadId OledTaskHandle;
osMessageQId AttitudeQueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartUartTask(void const * argument);
void StartImuTask(void const * argument);
void StartOledTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  osSemaphoreDef(UartRxSem);
  UartRxSemHandle = osSemaphoreCreate(osSemaphore(UartRxSem), 1);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* 队列存放 Attitude_t 指针（4字节），深度10 */
  osMessageQDef(AttitudeQueue, 10, 4);
  AttitudeQueueHandle = osMessageCreate(osMessageQ(AttitudeQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of UartTask */
  osThreadDef(UartTask, StartUartTask, osPriorityNormal, 0, 256);
  UartTaskHandle = osThreadCreate(osThread(UartTask), NULL);

  /* definition and creation of ImuTask */
  osThreadDef(ImuTask, StartImuTask, osPriorityAboveNormal, 0, 512);
  ImuTaskHandle = osThreadCreate(osThread(ImuTask), NULL);

  /* definition and creation of OledTask */
  osThreadDef(OledTask, StartOledTask, osPriorityBelowNormal, 0, 256);
  OledTaskHandle = osThreadCreate(osThread(OledTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  初始化任务：完成 MPU6050、OLED、Mahony 滤波器初始化及陀螺仪零偏校准
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  Mahony_Init(100.0f);          /* 与 ImuTask 的 osDelay(10) 对应，100Hz */
  OLED_Init();
  OLED_ShowString(1, 1, "IMU Attitude");
  OLED_ShowString(2, 1, "Roll: 0.0   ");
  OLED_ShowString(3, 1, "Pitch: 0.0  ");
  OLED_ShowString(4, 1, "Yaw: 0.0    ");

  MPU6050_Init(&hi2c1);

  /* 陀螺仪零偏校准：静止采样 200 次取平均 */
  for (int i = 0; i < 200; i++)
  {
    MPU6050_ReadRaw(&hi2c1, &raw);
    gx_offset += raw.gx;
    gy_offset += raw.gy;
    gz_offset += raw.gz;
    osDelay(5);
  }
  gx_offset /= 200.0f;
  gy_offset /= 200.0f;
  gz_offset /= 200.0f;

  for (;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartUartTask */
/**
  * @brief  串口任务：启动 DMA 不定长接收，IDLE 中断通知后原样回传
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUartTask */
void StartUartTask(void const * argument)
{
  /* USER CODE BEGIN StartUartTask */
  /* 启动 DMA 循环接收 + 使能 IDLE 中断 */
  HAL_UART_Receive_DMA(&huart1, rx_buf, RX_BUF_SIZE);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

  for (;;)
  {
    /* 等待 IDLE 中断释放信号量 */
    osSemaphoreWait(UartRxSemHandle, osWaitForever);

    if (g_rx_len > 0)
    {
      /* 停止当前 DMA，准备回传 */
      HAL_UART_DMAStop(&huart1);

      /* DMA 发送收到的数据 */
      HAL_UART_Transmit_DMA(&huart1, rx_buf, g_rx_len);
      /* 等待发送完成 */
      while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET);

      /* 重新启动 DMA 接收 */
      HAL_UART_Receive_DMA(&huart1, rx_buf, RX_BUF_SIZE);
    }
  }
  /* USER CODE END StartUartTask */
}

/* USER CODE BEGIN Header_StartImuTask */
/**
  * @brief  IMU 采样与姿态解算任务：100Hz 读 MPU6050，Mahony 滤波后发队列
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartImuTask */
void StartImuTask(void const * argument)
{
  /* USER CODE BEGIN StartImuTask */
  for (;;)
  {
    MPU6050_ReadRaw(&hi2c1, &raw);
    raw.gx -= gx_offset;
    raw.gy -= gy_offset;
    raw.gz -= gz_offset;
    /* 陀螺仪从 °/s 转成 rad/s 再给 Mahony */
    Mahony_Update(&att, raw.ax, raw.ay, raw.az,
                  raw.gx * 0.0174533f, raw.gy * 0.0174533f, raw.gz * 0.0174533f);

    /* 把姿态数据指针发给 OLED 任务 */
    osMessagePut(AttitudeQueueHandle, (uint32_t)&att, 0);

    osDelay(10);   /* 100 Hz，与 Mahony_Init(100.0f) 一致 */
  }
  /* USER CODE END StartImuTask */
}

/* USER CODE BEGIN Header_StartOledTask */
/**
  * @brief  OLED 显示任务：从队列取姿态数据，刷新 Roll/Pitch/Yaw
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartOledTask */
void StartOledTask(void const * argument)
{
  /* USER CODE BEGIN StartOledTask */
  char buf[24];
  for (;;)
  {
    osEvent evt = osMessageGet(AttitudeQueueHandle, osWaitForever);
    if (evt.status == osEventMessage)
    {
      Attitude_t *patt = (Attitude_t *)evt.value.v;

      sprintf(buf, "Roll: %6.1f\x7F ", patt->roll);
      OLED_ShowString(2, 1, buf);

      sprintf(buf, "Pitch:%6.1f\x7F ", patt->pitch);
      OLED_ShowString(3, 1, buf);

      sprintf(buf, "Yaw:  %6.1f\x7F ", patt->yaw);
      OLED_ShowString(4, 1, buf);
    }
  }
  /* USER CODE END StartOledTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

