#include "KMlib_1wire.h"
#include "stm32g431xx.h"
#include "usart.h"


void _1wire_SET_BAUDRATE(uint32_t baudrate) {
  huartx.Instance = USARTx;
  huartx.Init.BaudRate = baudrate;
  huartx.Init.WordLength = UART_WORDLENGTH_8B;
  huartx.Init.StopBits = UART_STOPBITS_1;
  huartx.Init.Parity = UART_PARITY_NONE;
  huartx.Init.Mode = UART_MODE_TX_RX;
  huartx.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huartx.Init.OverSampling = UART_OVERSAMPLING_16;
  huartx.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huartx.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huartx.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huartx.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  if (HAL_HalfDuplex_Init(&huartx) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huartx, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huartx, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huartx) != HAL_OK)
  {
    Error_Handler();
  }
}

HAL_StatusTypeDef _1wire_reset(void) {
  uint8_t data_out = 0xF0;
  uint8_t data_in = 0;
  _1wire_SET_BAUDRATE(9600);
  HAL_UART_Transmit(&huartx, &data_out, 1, 10);
  HAL_UART_Receive(&huartx, &data_in, 1, 10);
  _1wire_SET_BAUDRATE(115200);
  if (data_in == 0xF0)
    return HAL_ERROR;
  else
    return HAL_OK;
}

void _1wire_TX(uint8_t *data_out, uint8_t data_size) {
  uint8_t data_out_1 = 0xFF;
  uint8_t data_out_0 = 0x00;
  for (uint8_t bajt = 0; bajt < data_size; bajt++) {
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (data_out[bajt] & (1 << bit))
        HAL_UART_Transmit(&huartx, &data_out_1, 1, 10);
      else
        HAL_UART_Transmit(&huartx, &data_out_0, 1, 10);
    }
  }
}

uint8_t _1wire_RX() {
  uint8_t data_in = 0;
  uint8_t data_out = 0xFF;
  uint8_t val = 0;

  for (int bit = 0; bit < 8; bit++) {
    HAL_UART_Transmit(&huartx, &data_out, 1, 10);
    HAL_UART_Receive(&huartx, &data_in, 1, 10);
    val |= (data_in & 0x01) << bit;
  }
  return val;
}