/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void set_baudrate(uint32_t baudrate);
HAL_StatusTypeDef wire_reset(void);
void write_bit(int value);
void write_byte(uint8_t byte);
int read_bit(void);
uint8_t read_byte(void);

HAL_StatusTypeDef _1wire_reset(void);
HAL_StatusTypeDef _1wire_init(void);
HAL_StatusTypeDef _1wire_ROMID(uint8_t* rom_code_buf);
HAL_StatusTypeDef _1wire_SPI_config();
HAL_StatusTypeDef _1wire_SPI_BYTE(uint8_t mode, uint8_t* value);
HAL_StatusTypeDef _1wire_SENS_VDD_ON();
HAL_StatusTypeDef _1wire_SENS_VDD_OFF();
HAL_StatusTypeDef _1wire_SPI_SS_LOW();
HAL_StatusTypeDef _1wire_SPI_SS_HIGH();
HAL_StatusTypeDef _1wire_status();

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

