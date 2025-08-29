#ifndef __1wire_pot_H__
#define __1wire_pot_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"

HAL_StatusTypeDef _1wire_INIT(void);
HAL_StatusTypeDef _1wire_ROMID(uint8_t *rom_code_buf);
HAL_StatusTypeDef _1wire_MODE_CONFIG(uint8_t protocol, uint8_t speed);
HAL_StatusTypeDef _1wire_STATUS();
HAL_StatusTypeDef _1wire_BYTE_SEND(uint8_t *data_out, uint8_t data_lenght);
HAL_StatusTypeDef _1wire_RUN(uint8_t addr_lo, uint8_t addr_hi, uint8_t data_lenght);

#ifdef __cplusplus
}
#endif
#endif /*__ 1wire_pot_H__ */

