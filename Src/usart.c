/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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


  /*
  0000b Volatile Wiper 0          //  00b   Write Data  //  00 0000 0000        1 R3HW R3A R3W R3B R2HW R2A R2W R2B
  0001b Volatile Wiper 1          //  01b   INCR        //        |
  0100b Volatile TCON 0 Register  //  10b   DECR        //        |
  0110b Volatile Wiper 2          //  11b   READ DATA   //        |         OR  
  0111b Volatile Wiper 3          //                    //        |
  1010b Volatile TCON 1 Register  //                    //  11 1000 0000

  0000 00 1 0 0001 1001 = 0219h
  0001 00 1 0 0011 0010 = 1232h
  0110 00 1 0 0100 1011 = 6248h
  0111 00 1 0 0110 0100 = 7264h

  0100 00 1 1 1110 1110 = 43EE
  1010 00 1 1 1110 1110 = A3EE

  0000 00 X n nnnn nnnn
  
*/
#include "usart.h"

UART_HandleTypeDef huart1;
//#define _1WIRE_READ 0
//#define _1WIRE_WRITE 1

void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(uartHandle->Instance==USART1)
  {
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  }
}
void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
    __HAL_RCC_USART1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4);
  }
}
/*void set_baudrate(uint32_t baudrate)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = baudrate;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}
HAL_StatusTypeDef _1wire_reset(void)
{
  uint8_t data_out = 0xF0;
  uint8_t data_in = 0;
  set_baudrate(9600);
  HAL_UART_Transmit(&huart1, &data_out, 1, 10);
  HAL_UART_Receive(&huart1, &data_in, 1, 10);
  set_baudrate(115200);
  if (data_in != (0xF0 || 0xFF || 0x00))
    return HAL_OK;
  else
    return HAL_ERROR;
}
HAL_StatusTypeDef _1wire_init(void)
{
  if (_1wire_reset() != HAL_OK) return HAL_ERROR;
  write_byte(0xCC); // Skip ROM
  write_byte(0x66); // COMMAND START
  write_byte(0x05); // Number of Bytes
  write_byte(0x83); // Write GPIO Configuration
  write_byte(0x0B); // Sets Access to the GPIO control Register
  write_byte(0x03); // Only value allowed
  write_byte(0xA5); // GPIO_CTRL_HI Value
  write_byte(0x0F); // GPIO_CTRL_LO Value
  int CRC16_1 = read_byte(); //75
  int CRC16_2 = read_byte(); //02
  write_byte(0xAA); // Release Byte
  HAL_Delay(1);
  int DUMMY_BYTE = read_byte();
  int LENGHT = read_byte();
  int RESULT = read_byte();
  int CRC16_3 = read_byte();  // XX: CRC16
  int CRC16_4 = read_byte();  // XX: CRC16
  _1wire_reset();
  return HAL_OK;
}
HAL_StatusTypeDef _1wire_ROMID(uint8_t* rom_code_buf)
{
  if (_1wire_reset() != HAL_OK) return HAL_ERROR;
  write_byte(0x33);
  for (int i = 0; i < 8; i++) rom_code_buf[i] = read_byte();

  return HAL_OK;
}
HAL_StatusTypeDef _1wire_SPI_config()
{
  if (_1wire_reset() != HAL_OK) return HAL_ERROR;
  write_byte(0xCC); // Skip ROM
  write_byte(0x66); // COMMAND START
  write_byte(0x02); // Number of Bytes
  write_byte(0x55); // Write Configuration Command
  write_byte(0x08); // Configuration Register
                    //  PROT: SPI
                    //  SPI MODE: 0
                    //  SPD: 100 kHz
  int CRC16_1 = read_byte();  // XX: CRC16
  int CRC16_2 = read_byte();  // XX: CRC16
  write_byte(0xAA); // Release Byte
  HAL_Delay(1);
  int DUMMY_BYTE = read_byte();
  int LENGHT = read_byte();
  int RESULT = read_byte();
  int CRC16_3 = read_byte();  // XX: CRC16
  int CRC16_4 = read_byte();  // XX: CRC16
  if (_1wire_reset() != HAL_OK) return HAL_ERROR;
  if (RESULT == 0xAA) return HAL_OK;
  else return HAL_ERROR;
}
HAL_StatusTypeDef _1wire_SPI_SEQUENCER_WRITE()
{
  if (_1wire_reset() != HAL_OK) return HAL_ERROR;
  write_byte(0xCC); // Skip ROM
  write_byte(0x66); // COMMAND START
  write_byte(0x39); // Number of Bytes
  write_byte(0x11); // Write Sequencer Command
  write_byte(0x00); // ADDR_LO
  write_byte(0x00); // ADDR_HI

  write_byte(0xDD); // DELAY
  write_byte(0x03); // few ms
  write_byte(0x80); // SS low
  write_byte(0xC0); // SPI Write
  write_byte(0x02); // SPI write Lenght
  write_byte(0x00); // SPI read Lenght
    write_byte(0x43); // SPI BYTE TO SEND
    write_byte(0xEE); // SPI BYTE TO SEND
  write_byte(0x01); // SS high

  write_byte(0xDD); // DELAY
  write_byte(0x03); // few ms
  write_byte(0x80); // SS low
  write_byte(0xC0); // SPI Write
  write_byte(0x02); // SPI write Lenght
  write_byte(0x00); // SPI read Lenght
    write_byte(0xA3); // SPI BYTE TO SEND
    write_byte(0xEE); // SPI BYTE TO SEND
  write_byte(0x01); // SS high
/////////////////////////////////////////////////////////////////////////////////////////
    write_byte(0xDD); // DELAY
  write_byte(0x03); // few ms
  write_byte(0x80); // SS low
  write_byte(0xC0); // SPI Write
  write_byte(0x02); // SPI write Lenght
  write_byte(0x00); // SPI read Lenght
    write_byte(0x02); // SPI BYTE TO SEND
    write_byte(0x20); // SPI BYTE TO SEND
  write_byte(0x01); // SS high

    write_byte(0xDD); // DELAY
  write_byte(0x03); // few ms
  write_byte(0x80); // SS low
  write_byte(0xC0); // SPI Write
  write_byte(0x02); // SPI write Lenght
  write_byte(0x00); // SPI read Lenght
    write_byte(0x12); // SPI BYTE TO SEND
    write_byte(0x20); // SPI BYTE TO SEND
  write_byte(0x01); // SS high

    write_byte(0xDD); // DELAY
  write_byte(0x03); // few ms
  write_byte(0x80); // SS low
  write_byte(0xC0); // SPI Write
  write_byte(0x02); // SPI write Lenght
  write_byte(0x00); // SPI read Lenght
    write_byte(0x62); // SPI BYTE TO SEND
    write_byte(0x20); // SPI BYTE TO SEND
  write_byte(0x01); // SS high

    write_byte(0xDD); // DELAY
  write_byte(0x03); // few ms
  write_byte(0x80); // SS low
  write_byte(0xC0); // SPI Write
  write_byte(0x02); // SPI write Lenght
  write_byte(0x00); // SPI read Lenght
    write_byte(0x72); // SPI BYTE TO SEND
    write_byte(0x20); // SPI BYTE TO SEND
  write_byte(0x01); // SS high

  //write_byte(0x80); // SS low
  int CRC16_1 = read_byte();  // XX: CRC16
  int CRC16_2 = read_byte();  // XX: CRC16
  write_byte(0xAA); // Release Byte
  HAL_Delay(1);
  int DUMMY_BYTE = read_byte();
  int LENGHT_BYTE = read_byte();
  int RESULT = read_byte();
  int CRC16_3 = read_byte();  // XX: CRC16
  int CRC16_4 = read_byte();  // XX: CRC16
  return HAL_OK;
}
HAL_StatusTypeDef _1wire_SPI_SEQUENCER_READ()
{
  if (_1wire_reset() != HAL_OK) return HAL_ERROR;
  write_byte(0xCC); // Skip ROM
  write_byte(0x66); // COMMAND START
  write_byte(0x03); // Number of Bytes
  write_byte(0x22); // Read Sequencer Command
    write_byte(0x00); // ADDR_LO
    write_byte(0x6C); // LEN + ADDR_HI (WAZME)
  int CRC16_1 = read_byte();  // XX: CRC16
  int CRC16_2 = read_byte();  // XX: CRC16
  write_byte(0xAA); // Release Byte
  HAL_Delay(1);
  int DUMMY_BYTE = read_byte();
  int LENGHT_BYTE = read_byte();
  int RESULT = read_byte();
  uint8_t RX_DATA[LENGHT_BYTE];
  for(int i = 0; i<(LENGHT_BYTE-1); i++) RX_DATA[i] = read_byte();
  int CRC16_3 = read_byte();  // XX: CRC16
  int CRC16_4 = read_byte();  // XX: CRC16
  return HAL_OK;
}
HAL_StatusTypeDef _1wire_SPI_SEQUENCER_RUN()
{
  if (_1wire_reset() != HAL_OK) return HAL_ERROR;
  write_byte(0xCC); // Skip ROM
  write_byte(0x66); // COMMAND START
  write_byte(0x04); // Number of Bytes
  write_byte(0x33); // RUN SEQUENCER
  write_byte(0x00); // ADDR_LO
  write_byte(0x6C); // SLEN_LO:ADDR_HI
  write_byte(0x00); // SLEN_HI
  int CRC16_0 = read_byte();  // XX: CRC16
  int CRC16_1 = read_byte();  // XX: CRC16
  write_byte(0xAA); // Release Byte
  HAL_Delay(1000);
  int DUMMY_BYTE = read_byte();
  int LENGHT = read_byte();
  int RESULT = read_byte();
  if (RESULT == 0x88)
  {
    int RX_1 = read_byte();  // XX: CRC16
    int RX_2 = read_byte();  // XX: CRC16
  }
  int CRC16_2 = read_byte();  // XX: CRC16
  int CRC16_3 = read_byte();  // XX: CRC16
  if (_1wire_reset() != HAL_OK) return HAL_ERROR;
  if (RESULT == 0xAA) return HAL_OK;
  else return HAL_ERROR;
}
HAL_StatusTypeDef _1wire_SPI_SS_LOW()
{
  //WRITE
  if (_1wire_reset() != HAL_OK) return HAL_ERROR;
  write_byte(0xCC); // Skip ROM
  write_byte(0x66); // COMMAND START
  write_byte(0x04); // Number of Bytes
  write_byte(0x11); // Write Sequencer Command
  write_byte(0x00); // ADDR_LO
  write_byte(0x00); // ADDR_HI
  write_byte(0x80); // SS LOW
  int CRC16_1 = read_byte();  // 7E: CRC16
  int CRC16_2 = read_byte();  // 7E: CRC16
  write_byte(0xAA); // Release Byte
  HAL_Delay(1);
  int DUMMY_BYTE1 = read_byte();
  int LENGHT1 = read_byte();
  int RESULT1 = read_byte();
  int CRC16_3 = read_byte();  // XX: CRC16
  int CRC16_4 = read_byte();  // XX: CRC16
  if (RESULT1 != 0xAA) return HAL_ERROR;
  //RUN
  if (_1wire_reset() != HAL_OK) return HAL_ERROR;
  write_byte(0xCC); // Skip ROM
  write_byte(0x66); // COMMAND START
  write_byte(0x04); // Number of Bytes
  write_byte(0x33); // RUN SEQUENCER
  write_byte(0x00); // ADDR_LO
  write_byte(0x02); // SLEN_LO:ADDR_HI
  write_byte(0x00); // SLEN_HI
  int CRC16_5 = read_byte();  // XX: CRC16
  int CRC16_6 = read_byte();  // XX: CRC16
  write_byte(0xAA); // Release Byte
  HAL_Delay(1);
  int DUMMY_BYTE2 = read_byte();
  int LENGHT2 = read_byte();
  int RESULT2 = read_byte();
  if (RESULT2 == 0x88)
  {
    int RESULT2_1 = read_byte();  // XX: CRC16
    int RESULT2_2 = read_byte();  // XX: CRC16
  }
  int CRC16_7 = read_byte();  // XX: CRC16
  int CRC16_8 = read_byte();  // XX: CRC16
  if (_1wire_reset() != HAL_OK) return HAL_ERROR;
  if (RESULT2 == 0xAA) return HAL_OK;
  else return HAL_ERROR;
}
HAL_StatusTypeDef _1wire_SPI_SS_HIGH()
{
  //WRITE
  if (_1wire_reset() != HAL_OK) return HAL_ERROR;
  write_byte(0xCC); // Skip ROM
  write_byte(0x66); // COMMAND START
  write_byte(0x04); // Number of Bytes
  write_byte(0x11); // Write Sequencer Command
  write_byte(0x00); // ADDR_LO
  write_byte(0x00); // ADDR_HI
  write_byte(0x01); // SS HIGH
  int CRC16_1 = read_byte();  // 7E: CRC16
  int CRC16_2 = read_byte();  // 7E: CRC16
  write_byte(0xAA); // Release Byte
  HAL_Delay(1);
  int DUMMY_BYTE1 = read_byte();
  int LENGHT1 = read_byte();
  int RESULT1 = read_byte();
  int CRC16_3 = read_byte();  // XX: CRC16
  int CRC16_4 = read_byte();  // XX: CRC16
  if (RESULT1 != 0xAA) return HAL_ERROR;
  //RUN
  if (_1wire_reset() != HAL_OK) return HAL_ERROR;
  write_byte(0xCC); // Skip ROM
  write_byte(0x66); // COMMAND START
  write_byte(0x04); // Number of Bytes
  write_byte(0x33); // RUN SEQUENCER
  write_byte(0x00); // ADDR_LO
  write_byte(0x02); // SLEN_LO:ADDR_HI
  write_byte(0x00); // SLEN_HI
  int CRC16_5 = read_byte();  // XX: CRC16
  int CRC16_6 = read_byte();  // XX: CRC16
  write_byte(0xAA); // Release Byte
  HAL_Delay(1);
  int DUMMY_BYTE2 = read_byte();
  int LENGHT2 = read_byte();
  int RESULT2 = read_byte();
  if (RESULT2 == 0x88)
  {
    int RESULT2_1 = read_byte();  // XX: CRC16
    int RESULT2_2 = read_byte();  // XX: CRC16
  }
  int CRC16_7 = read_byte();  // XX: CRC16
  int CRC16_8 = read_byte();  // XX: CRC16
  if (_1wire_reset() != HAL_OK) return HAL_ERROR;
  if (RESULT2 == 0xAA) return HAL_OK;
  else return HAL_ERROR;
}
HAL_StatusTypeDef _1wire_SENS_VDD_ON()
{
  //WRITE
  if (_1wire_reset() != HAL_OK) return HAL_ERROR;
  write_byte(0xCC); // Skip ROM
  write_byte(0x66); // COMMAND START
  write_byte(0x04); // Number of Bytes
  write_byte(0x11); // Write Sequencer Command
  write_byte(0x00); // ADDR_LO
  write_byte(0x00); // ADDR_HI
  write_byte(0xCC); // SENS_VDD ON
  int CRC16_1 = read_byte();  // 7E: CRC16
  int CRC16_2 = read_byte();  // 7E: CRC16
  write_byte(0xAA); // Release Byte
  HAL_Delay(1);
  int DUMMY_BYTE1 = read_byte();
  int LENGHT1 = read_byte();
  int RESULT1 = read_byte();
  int CRC16_3 = read_byte();  // XX: CRC16
  int CRC16_4 = read_byte();  // XX: CRC16
  if (RESULT1 != 0xAA) return HAL_ERROR;
  //RUN
  if (_1wire_reset() != HAL_OK) return HAL_ERROR;
  write_byte(0xCC); // Skip ROM
  write_byte(0x66); // COMMAND START
  write_byte(0x04); // Number of Bytes
  write_byte(0x33); // RUN SEQUENCER
  write_byte(0x00); // ADDR_LO
  write_byte(0x02); // SLEN_LO:ADDR_HI
  write_byte(0x00); // SLEN_HI
  int CRC16_5 = read_byte();  // XX: CRC16
  int CRC16_6 = read_byte();  // XX: CRC16
  write_byte(0xAA); // Release Byte
  HAL_Delay(5);
  int DUMMY_BYTE2 = read_byte();
  int LENGHT2 = read_byte();
  int RESULT2 = read_byte();
  if (RESULT2 == 0x88)
  {
    int RESULT2_1 = read_byte();  // XX: CRC16
    int RESULT2_2 = read_byte();  // XX: CRC16
  }
  int CRC16_7 = read_byte();  // XX: CRC16
  int CRC16_8 = read_byte();  // XX: CRC16
  if (_1wire_reset() != HAL_OK) return HAL_ERROR;
  if (RESULT2 == 0xAA) return HAL_OK;
  else return HAL_ERROR;
}
HAL_StatusTypeDef _1wire_SENS_VDD_OFF()
{
  //WRITE
  if (_1wire_reset() != HAL_OK) return HAL_ERROR;
  write_byte(0xCC); // Skip ROM
  write_byte(0x66); // COMMAND START
  write_byte(0x04); // Number of Bytes
  write_byte(0x11); // Write Sequencer Command
  write_byte(0x00); // ADDR_LO
  write_byte(0x00); // ADDR_HI
  write_byte(0xBB); // SENS_VDD OFF
  int CRC16_1 = read_byte();  // 7E: CRC16
  int CRC16_2 = read_byte();  // 7E: CRC16
  write_byte(0xAA); // Release Byte
  HAL_Delay(1);
  int DUMMY_BYTE1 = read_byte();
  int LENGHT1 = read_byte();
  int RESULT1 = read_byte();
  int CRC16_3 = read_byte();  // XX: CRC16
  int CRC16_4 = read_byte();  // XX: CRC16
  if (RESULT1 != 0xAA) return HAL_ERROR;
  //RUN
  if (_1wire_reset() != HAL_OK) return HAL_ERROR;
  write_byte(0xCC); // Skip ROM
  write_byte(0x66); // COMMAND START
  write_byte(0x04); // Number of Bytes
  write_byte(0x33); // RUN SEQUENCER
  write_byte(0x00); // ADDR_LO
  write_byte(0x02); // SLEN_LO:ADDR_HI
  write_byte(0x00); // SLEN_HI
  int CRC16_5 = read_byte();  // XX: CRC16
  int CRC16_6 = read_byte();  // XX: CRC16
  write_byte(0xAA); // Release Byte
  HAL_Delay(1);
  int DUMMY_BYTE2 = read_byte();
  int LENGHT2 = read_byte();
  int RESULT2 = read_byte();
  if (RESULT2 == 0x88)
  {
    int RESULT2_1 = read_byte();  // XX: CRC16
    int RESULT2_2 = read_byte();  // XX: CRC16
  }
  int CRC16_7 = read_byte();  // XX: CRC16
  int CRC16_8 = read_byte();  // XX: CRC16
  if (_1wire_reset() != HAL_OK) return HAL_ERROR;
  if (RESULT2 == 0xAA) return HAL_OK;
  else return HAL_ERROR;
}
HAL_StatusTypeDef _1wire_status()
{
  if (_1wire_reset() != HAL_OK) return HAL_ERROR;
  write_byte(0xCC); // Skip ROM
  write_byte(0x66); // COMMAND START
  write_byte(0x01); // Number of Bytes
  write_byte(0x7A); // Device status
  int CRC16_1 = read_byte();  // XX: CRC16
  int CRC16_2 = read_byte();  // XX: CRC16
  write_byte(0xAA); // Release Byte
  HAL_Delay(1);
  int DUMMY_BYTE = read_byte();
  int LENGHT = read_byte(); //05h
  int RESULT = read_byte();
  int DEVICE_STATUS = read_byte();
  int VERSION = read_byte();
  int MANID0 = read_byte();
  int MANID1 = read_byte();
  int CRC16_3 = read_byte();  // XX: CRC16
  int CRC16_4 = read_byte();  // XX: CRC16
  if (_1wire_reset() != HAL_OK) return HAL_ERROR;
  if (RESULT != 0xAA) return HAL_ERROR;
  return HAL_OK;
}
HAL_StatusTypeDef _1wire_SPI_BYTE(uint8_t mode, uint8_t* value)
{
  if (_1wire_reset() != HAL_OK) return HAL_ERROR;
  write_byte(0xCC); // Skip ROM
  write_byte(0x66); // COMMAND START
  write_byte(0xC0); // SPI WRITE READ BYTE
  if (mode)
  {
    write_byte(0x00); // Write Length
    write_byte(0x02);   // Read Length
  } else
  {
    write_byte(0x02);   // Write Length
    write_byte(0x00); // Read Length
  }
  for(int i = 0; i<2; i++) write_byte(value[i]);
  return HAL_OK;
}
void write_bit(int value)
{
  if (value) {
      uint8_t data_out = 0xff;
    HAL_UART_Transmit(&huart1, &data_out, 1, 10);
  } else {
      uint8_t data_out = 0x0;
    HAL_UART_Transmit(&huart1, &data_out, 1, 10);
  }
}
int read_bit(void)
{
  uint8_t data_out = 0xFF;
  uint8_t data_in = 0;
  HAL_UART_Transmit(&huart1, &data_out, 1, 10);
  HAL_UART_Receive(&huart1, &data_in, 1, 10);
  return data_in & 0x01;
}
void write_byte(uint8_t byte)
{
  for (int i = 0; i < 8; i++)
    write_bit(byte & (1 << i));
}
uint8_t read_byte(void)
{
  uint8_t val = 0;
  for (int i = 0; i < 8; i++)
    val |= (read_bit() << i);
  return val;
}

*/