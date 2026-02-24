#ifndef __KMlib_DS28E18_H__
#define __KMlib_DS28E18_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"

#define DEVICES_NUMBER 1
// #define ROMID // Define specific ROMID to use
#define PERIPHERAL I2C
// #define PERIPHERAL SPI

#define I2C_SPEED I2C_SPEED_100KHZ // if SPI selected, leave it default
// #define I2C_SPEED I2C_SPEED_400KHZ
// #define I2C_SPEED I2C_SPEED_1MHZ
// #define I2C_SPEED I2C_SPEED_2_3MHZ

#define I2C_NACK_MODE I2C_NACK_STOP // if SPI selected, leave it default
// #define I2C_NACK_MODE I2C_NACK_IGNORE

#define SPI_MODE SPI_MODE_0 // if I2C selected, leave it default
// #define SPI_MODE SPI_MODE_3

HAL_StatusTypeDef DS28E18_INIT();
HAL_StatusTypeDef DS28E18_CONFIG();
HAL_StatusTypeDef DS28E18_ROMID(uint8_t *rom_code_buf);
HAL_StatusTypeDef DS28E18_STATUS(void);
HAL_StatusTypeDef DS28E18_RUN(uint8_t addr_lo, uint8_t addr_hi,
                              uint8_t data_lenght);
HAL_StatusTypeDef DS28E18_WRITE_SEQ(uint8_t *data_out, uint8_t data_lenght);
HAL_StatusTypeDef DS28E18_READ(uint8_t addr_lo, uint8_t addr_hi,
                               uint8_t data_lenght);

#ifdef __cplusplus
}
#endif
#endif /*__ KMlib_DS28E18_H__ */