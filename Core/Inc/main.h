/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
#include "ventilator_types.h"
#include "math.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern osSemaphoreId_t Receive_BufferHandle;
extern osSemaphoreId_t ADC_BufferHandle;
extern uint16_t ADC_RESULT_BUF[3];
extern char Rx_UART[];
extern uint8_t UART_RX_BUF[17];
extern uint32_t UART_RX_BUF_LENGTH;
extern osThreadId_t E_VALUE_CLOSEHandle;
extern uint16_t  ADS1115_raw;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern void COMMAND_HANDLER(SET_PARAM_CMD_PACKET *RX_PARAM_CMD_PACKET);
extern void ALERT_COMMAND_HANDLER(ALERT_RANGE_PACKET *RX_ALERT_RANGE_PACKET);
extern void SERVICE_COMMAND_HANDLER( REQUEST_SERVICE_PACKET_tst *RX_SERVICE_RANGE_PACKET);
extern void SEND_ALERT_PACKET();
extern HAL_StatusTypeDef HAL_UART_DMA_Tx_Stop(UART_HandleTypeDef *huart);
extern void Switch_TASK_I_CYCLE(void);
extern void Switch_TASK_E_CYCLE(void);
extern void SEND_REPORT_PACKET();
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RX_FLAG_Pin GPIO_PIN_2
#define RX_FLAG_GPIO_Port GPIOE
#define MODE_FLAG_Pin GPIO_PIN_3
#define MODE_FLAG_GPIO_Port GPIOE
#define Blower_Pin GPIO_PIN_5
#define Blower_GPIO_Port GPIOE
#define InspValve_Pin GPIO_PIN_2
#define InspValve_GPIO_Port GPIOC
#define ExpValve_Pin GPIO_PIN_3
#define ExpValve_GPIO_Port GPIOC
#define Wave_Pin GPIO_PIN_1
#define Wave_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

#define InspValve_Close() 	(GPIOC->ODR|=((1<<3)))
#define InspValve_Open()  	(GPIOC->ODR&=(~(1<<3)))

#define ADS1115_ADDRESS 0x49

#define ExpValve_Open()  (GPIOA->ODR&=(~(1<<1)))
#define ExpValve_Close() (GPIOA->ODR|=((1<<1)))

#define BLOWER_ON()  GPIOE->ODR|=(1<<5)
#define BLOWER_OFF()  GPIOE->ODR&=(~(1<<5))


extern uint16_t ADS1115_Getdata(uint8_t channel);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
