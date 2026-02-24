#ifndef __KMlib_1wire_H__
#define __KMlib_1wire_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"

#define huartx huart1
#define USARTx USART1

void _1wire_SET_BAUDRATE(uint32_t baudrate);
HAL_StatusTypeDef _1wire_reset(void);
void _1wire_TX(uint8_t *data_out, uint8_t data_size);
uint8_t _1wire_RX();

#ifdef __cplusplus
}
#endif
#endif /*__ KMlib_1wire_H__ */