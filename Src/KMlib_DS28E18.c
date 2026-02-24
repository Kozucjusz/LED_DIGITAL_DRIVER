#include "KMlib_DS28E18.h"
#include "KMlib_1wire.h"
#include <stdint.h>
#include <sys/types.h>

#define TMP_BUF 0x08

#define I2C 0b0
#define SPI 0b1

#define PROT PERIPHERAL
#define INACK I2C_NACK_MODE
#define SPD I2C_SPEED

#define SPI_MODE_0 0b00
#define SPI_MODE_3 0b11
#define I2C_NACK_STOP 0b0
#define I2C_NACK_IGNORE 0b1

#define I2C_SPEED_100KHZ 0b00
#define I2C_SPEED_400KHZ 0b01
#define I2C_SPEED_1MHZ 0b10
#define I2C_SPEED_2_3MHZ 0b11

#define RELASE_BYTE 0xAA
#define RESULT_OK 0xAA

#define READ_ROM 0x33
#define MATCH_ROM 0x55
#define SEARCH_ROM 0xF0
#define SKIP_ROM 0xCC
#define RESUME 0xA5
#define OD_SKIP 0x3C
#define OD_MATCH 0x69

#define CMD_START 0x66
#define WRITE_SEQ 0x11
#define READ_SEQ 0x22
#define RUN_SEQ 0x33
#define WRITE_CFG 0x55
#define READ_CFG 0x6A
#define WRITE_GPIO_CFG 0x83
#define READ_GPIO_CFG 0x7C
#define DEVICE_STATUS 0x7A

#define I2C_START 0x02
#define I2C_STOP 0x03
#define I2C_WRITE_DATA 0xE3
#define I2C_READ_DATA 0xD4
#define I2C_READ_DATA_WNACK 0xD3

#define SPI_WRITE_READ_BYTE 0xC0
#define SPI_WRITE_READ_BIT 0xB0
#define SS_HIGH 0x01
#define SS_LOW 0x80

#define DELAY 0xDD
#define SENS_VDD_ON 0xCC
#define SENS_VDD_OFF 0xBB
#define GPIO_BUF_WRITE 0xD1
#define GPIO_BUF_READ 0x1D
#define GPIO_CNTL_WRITE 0xE2
#define GPIO_CNTL_READ 0x2E

HAL_StatusTypeDef DS28E18_ROMID(uint8_t *rom_code_buf) {
  if (rom_code_buf == NULL)
    return HAL_ERROR;
  if (_1wire_reset() != HAL_OK)
    return HAL_ERROR;
  _1wire_TX((uint8_t[]){READ_ROM}, 1);
  for (uint8_t rx_byte = 0; rx_byte < 8; rx_byte++)
    rom_code_buf[rx_byte] = _1wire_RX();
  return HAL_OK;
}

HAL_StatusTypeDef DS28E18_INIT() {
  if (_1wire_reset() != HAL_OK)
    return HAL_ERROR;
  _1wire_TX((uint8_t[]){SKIP_ROM,       // Skip ROM,
                        CMD_START,      // COMMAND START
                        0x05,           // Number of Bytes
                        WRITE_GPIO_CFG, // Write GPIO Configuration
                        0x0B,  // Sets Access to the GPIO control Register
                        0x03,  // Only value allowed
                        0xA5,  // GPIO_CTRL_HI Value
                        0x0F}, // GPIO_CTRL_LO Value
            8);
  uint8_t CRC16_1 = _1wire_RX();
  uint8_t CRC16_2 = _1wire_RX();
  _1wire_TX((uint8_t[]){RELASE_BYTE}, 1); // Release Byte
  HAL_Delay(1);
  uint8_t DUMMY_BYTE = _1wire_RX();
  uint8_t LENGHT = _1wire_RX();
  uint8_t RESULT = _1wire_RX();
  uint8_t CRC16_3 = _1wire_RX();
  uint8_t CRC16_4 = _1wire_RX();
  if (RESULT != 0xAA)
    return HAL_ERROR;
  return HAL_OK;
}

HAL_StatusTypeDef DS28E18_STATUS() {
  if (_1wire_reset() != HAL_OK)
    return HAL_ERROR;
  _1wire_TX((uint8_t[]){SKIP_ROM,       //  Skip ROM,
                        CMD_START,      //  COMMAND START
                        0x01,           //  Number of Bytes
                        DEVICE_STATUS}, //  Device status
            4);
  uint8_t CRC16_1 = _1wire_RX();
  uint8_t CRC16_2 = _1wire_RX();
  _1wire_TX((uint8_t[]){RELASE_BYTE}, 1); // Release Byte
  HAL_Delay(1);
  uint8_t DUMMY_BYTE = _1wire_RX();
  uint8_t LENGHT = _1wire_RX();
  uint8_t RESULT = _1wire_RX();
  uint8_t STATUS = _1wire_RX();
  uint8_t VERSION = _1wire_RX();
  uint8_t MANID0 = _1wire_RX();
  uint8_t MANID1 = _1wire_RX();
  uint8_t CRC16_3 = _1wire_RX();
  uint8_t CRC16_4 = _1wire_RX();
  if (RESULT != RESULT_OK)
    return HAL_ERROR;
  return HAL_OK;
}

HAL_StatusTypeDef DS28E18_CONFIG() {
  uint8_t config_reg =
      (SPI_MODE << 4) | (PROT << 3) | (INACK << 2) | (SPD << 0);
  if (_1wire_reset() != HAL_OK)
    return HAL_ERROR;
  _1wire_TX((uint8_t[]){SKIP_ROM,    //  Skip ROM,
                        CMD_START,   //  COMMAND START
                        0x02,        // Number of Bytes
                        WRITE_CFG,   // WRITE CONFIG
                        config_reg}, // RFU, SPI_MODE, PROT, INACK, SPD
            5);
  uint8_t CRC16_1 = _1wire_RX();
  uint8_t CRC16_2 = _1wire_RX();
  _1wire_TX((uint8_t[]){RELASE_BYTE}, 1); // Release Byte
  HAL_Delay(1);                           // DELAY FOR COMMAND DURATION
  uint8_t DUMMY_BYTE = _1wire_RX();
  uint8_t LENGHT = _1wire_RX();
  uint8_t RESULT = _1wire_RX();
  uint8_t CRC16_3 = _1wire_RX(); // XX: CRC16
  uint8_t CRC16_4 = _1wire_RX(); // XX: CRC16
  if (RESULT != RESULT_OK)
    return HAL_ERROR;
  return HAL_OK;
}

HAL_StatusTypeDef DS28E18_WRITE_SEQ(uint8_t *data_out, uint8_t data_lenght) {
  uint8_t tx_data[128] = {
      SKIP_ROM,        //  Skip ROM,
      CMD_START,       //  COMMAND START
      data_lenght + 7, //  Number of Bytes: data_lenght + 9 (WRITE_SEQ, ADDR_LO,
                       //  ADDR_HI, DELAY, DELAY_MS, I2C START, I2C_WRITE_DATA,
                       //  Data Length, I2C_STOP)
      WRITE_SEQ,       //  Write Sequencer Command
      0x00,            //  ADDR_LO
      0x00,            //  ADDR_HI
      I2C_START,       //  I2C START
      I2C_WRITE_DATA,  //  Write Data
      data_lenght,     //  Data Length
  };
  for (uint8_t data_element = 0; data_element < data_lenght; data_element++)
    tx_data[9 + data_element] = data_out[data_element]; // Data fill
  tx_data[9 + data_lenght] = I2C_STOP;                  // I2C STOP

  if (_1wire_reset() != HAL_OK)
    return HAL_ERROR;
  _1wire_TX(tx_data,
            data_lenght +
                10); // Number of Bytes: data_lenght + 11 (SKIP_ROM, CMD_START,
                     // WRITE_SEQ, ADDR_LO, ADDR_HI, DELAY, DELAY_MS, I2C START,
                     // I2C_WRITE_DATA, Data Length, I2C_STOP)
  uint8_t CRC16_1 = _1wire_RX();
  uint8_t CRC16_2 = _1wire_RX();
  _1wire_TX((uint8_t[]){RELASE_BYTE}, 1); // Release Byte
  HAL_Delay(1);
  uint8_t DUMMY_BYTE = _1wire_RX();
  uint8_t LENGHT = _1wire_RX();
  uint8_t RESULT = _1wire_RX();
  uint8_t CRC16_3 = _1wire_RX();
  uint8_t CRC16_4 = _1wire_RX();
  if (RESULT != RESULT_OK)
    return HAL_ERROR;
  return DS28E18_RUN(0x00, 0x00, data_lenght + 4);
  // return DS28E18_READ(0x00, 0x00, data_lenght + 4); // for test
}

HAL_StatusTypeDef DS28E18_RUN(uint8_t addr_lo, uint8_t addr_hi,
                              uint8_t data_lenght) {
  if (_1wire_reset() != HAL_OK)
    return HAL_ERROR;
  _1wire_TX((uint8_t[]){SKIP_ROM,         //  Skip ROM,
                        CMD_START,        //  COMMAND START
                        0x04,             // Number of Bytes
                        RUN_SEQ,          // RUN SEQUENCER
                        0x00,             // ADDR_LO
                        data_lenght << 1, // SLEN_LO:ADDR_HI
                        0x00},            // SLEN_HI
            7);
  uint8_t CRC16_1 = _1wire_RX();
  uint8_t CRC16_2 = _1wire_RX();
  _1wire_TX((uint8_t[]){RELASE_BYTE}, 1); // Release Byte
  HAL_Delay(10);                          // DELAY FOR COMMAND DURATION
  uint8_t DUMMY_BYTE = _1wire_RX();
  uint8_t LENGHT = _1wire_RX();
  uint8_t RESULT = _1wire_RX();
  if (RESULT == 0x88) {
    uint8_t RX_1 = _1wire_RX(); // XX: CRC16
    uint8_t RX_2 = _1wire_RX(); // XX: CRC16
  }
  uint8_t CRC16_3 = _1wire_RX(); // XX: CRC16
  uint8_t CRC16_4 = _1wire_RX(); // XX: CRC16
  if (RESULT != RESULT_OK)
    return HAL_ERROR;
  return HAL_OK;
}

HAL_StatusTypeDef DS28E18_READ(uint8_t addr_lo, uint8_t addr_hi,
                               uint8_t data_lenght) {
  if (_1wire_reset() != HAL_OK)
    return HAL_ERROR;
  _1wire_TX((uint8_t[]){0xCC,              //  Skip ROM,
                        CMD_START,         //  COMMAND START
                        0x03,              // Number of Bytes
                        READ_SEQ,          // READ SEQUENCER
                        0x00,              // ADDR_LO
                        data_lenght << 1}, // SLEN_LO:ADDR_HI
            6);
  uint8_t CRC16_1 = _1wire_RX();
  uint8_t CRC16_2 = _1wire_RX();
  _1wire_TX((uint8_t[]){RELASE_BYTE}, 1); // Release Byte
  HAL_Delay(1);
  uint8_t DUMMY_BYTE = _1wire_RX();
  uint8_t LENGHT = _1wire_RX();
  uint8_t RESULT = _1wire_RX();
  uint8_t RX_ARRAY[128] = {0};
  for (uint8_t data_number = 0; data_number < LENGHT - 1; data_number++) {
    RX_ARRAY[data_number] = _1wire_RX();
  }
  uint8_t CRC16_3 = _1wire_RX(); // XX: CRC16
  uint8_t CRC16_4 = _1wire_RX(); // XX: CRC16
  if (RESULT != 0xAA)
    return HAL_ERROR;
  return HAL_OK;
}

HAL_StatusTypeDef DS28E18_SENS_VDD(uint8_t state) {
  if (_1wire_reset() != HAL_OK)
    return HAL_ERROR;
  uint8_t tx_data[9] = {
      SKIP_ROM,  //  Skip ROM,
      CMD_START, //  COMMAND START
      0x06,      //  Number of Bytes
      WRITE_SEQ, //  Write Sequencer Command
      0x00,      //  ADDR_LO
      0x00,      //  ADDR_HI
      DELAY,     //  DELAY
      0x03,      //  8ms
  };
  if (state)
    tx_data[8] = SENS_VDD_ON;
  else
    tx_data[8] = SENS_VDD_OFF;
  _1wire_TX(tx_data, tx_data[2] + 2); // Total data send
  return DS28E18_RUN(0x00, 0x00, 10);
}