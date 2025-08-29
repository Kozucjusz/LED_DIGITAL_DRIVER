#include "1wire_pot.h"

#define _1wire_I2C 0b0
#define _1wire_SPI 0b1

#define SPI_MODE_0 0b000010
#define SPI_MODE_3 0b001110
#define I2C_NACK_STOP 0b000000
#define I2C_NACK_IGNORE 0b000001

#define SPEED_100KHZ 0b00
#define SPEED_400KHZ 0b01
#define SPEED_1MHZ 0b10
#define SPEED_2_3MHZ 0b11

uint8_t _1wire_mode = 0;

void set_baudrate(uint32_t baudrate);
HAL_StatusTypeDef _1wire_reset(void);
void _1wire_TX(uint8_t *data_out, uint8_t data_size);
uint8_t _1wire_RX();

void set_baudrate(uint32_t baudrate)
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
  if ((data_in >= 0x10) || (data_in <= 0x90))
    return HAL_OK;
  else
    return HAL_ERROR;
}

void _1wire_TX(uint8_t *data_out, uint8_t data_size)
{
  uint8_t data_out_1 = 0xFF;
  uint8_t data_out_0 = 0x00;
  for (uint8_t i = 0; i < data_size; i++)
  {
    for (uint8_t j = 0; j < 8; j++)
    {
      if (data_out[i] & (1 << j))
        HAL_UART_Transmit(&huart1, &data_out_1, 1, 10);
      else
        HAL_UART_Transmit(&huart1, &data_out_0, 1, 10);
    }
  }
}

uint8_t _1wire_RX()
{
  uint8_t data_out = 0xFF;
  uint8_t data_in = 0;
  uint8_t val = 0;
  for (int i = 0; i < 8; i++)
  {
    HAL_UART_Transmit(&huart1, &data_out, 1, 10);
    HAL_UART_Receive(&huart1, &data_in, 1, 10);
    val |= (data_in & 0x01) << i;
  }
  return val;
}

HAL_StatusTypeDef _1wire_INIT(void)
{
  if (_1wire_reset() != HAL_OK)
    return HAL_ERROR;
  _1wire_TX((uint8_t[]){
                0xCC,  // Skip ROM,
                0x66,  // COMMAND START
                0x05,  // Number of Bytes
                0x83,  // Write GPIO Configuration
                0x0B,  // Sets Access to the GPIO control Register
                0x03,  // Only value allowed
                0xA5,  // GPIO_CTRL_HI Value
                0x0F}, // GPIO_CTRL_LO Value
            8);
  uint8_t CRC16_1 = _1wire_RX();
  uint8_t CRC16_2 = _1wire_RX();
  _1wire_TX((uint8_t[]){0xAA}, 1); // Release Byte
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

HAL_StatusTypeDef _1wire_ROMID(uint8_t *rom_code_buf)
{
  if (rom_code_buf == NULL)
    return HAL_ERROR;
  if (_1wire_reset() != HAL_OK)
    return HAL_ERROR;
  _1wire_TX((uint8_t[]){0x33}, 1);
  for (int i = 0; i < 8; i++)
    rom_code_buf[i] = _1wire_RX();
  return HAL_OK;
}

HAL_StatusTypeDef _1wire_MODE_CONFIG(uint8_t protocol, uint8_t speed)
{
  if ((protocol == SPI_MODE_0) || (protocol == SPI_MODE_3))
    _1wire_mode = _1wire_SPI;
  else if ((protocol == I2C_NACK_IGNORE) || (protocol == I2C_NACK_STOP))
    _1wire_mode = _1wire_I2C;
  else
    return HAL_ERROR;

  if (speed > 3)
    return HAL_ERROR;
  uint8_t config_register = (protocol << 2) | speed;

  if (_1wire_reset() != HAL_OK)
    return HAL_ERROR;
  _1wire_TX((uint8_t[]){0xCC,             // Skip ROM,
                        0x66,             // COMMAND START
                        0x02,             // Number of Bytes
                        0x55,             // Write Configuration Command
                        config_register}, // Configuration Register
            5);
  uint8_t CRC16_1 = _1wire_RX();
  uint8_t CRC16_2 = _1wire_RX();
  _1wire_TX((uint8_t[]){0xAA}, 1); // Release Byte
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

HAL_StatusTypeDef _1wire_STATUS()
{
  if (_1wire_reset() != HAL_OK)
    return HAL_ERROR;
  _1wire_TX((uint8_t[]){0xCC,  //  Skip ROM,
                        0x66,  //  COMMAND START
                        0x01,  //  Number of Bytes
                        0x7A}, //  Device status
            4);
  uint8_t CRC16_1 = _1wire_RX();
  uint8_t CRC16_2 = _1wire_RX();
  _1wire_TX((uint8_t[]){0xAA}, 1); // Release Byte
  HAL_Delay(1);
  uint8_t DUMMY_BYTE = _1wire_RX();
  uint8_t LENGHT = _1wire_RX();
  uint8_t RESULT = _1wire_RX();
  uint8_t DEVICE_STATUS = _1wire_RX();
  uint8_t VERSION = _1wire_RX();
  uint8_t MANID0 = _1wire_RX();
  uint8_t MANID1 = _1wire_RX();
  uint8_t CRC16_3 = _1wire_RX();
  uint8_t CRC16_4 = _1wire_RX();
  if (RESULT != 0xAA)
    return HAL_ERROR;
  return HAL_OK;
}

HAL_StatusTypeDef _1wire_BYTE_SEND(uint8_t *data_out, uint8_t data_lenght)
{
  if (_1wire_mode > 1)
    return HAL_ERROR;
  if (_1wire_reset() != HAL_OK)
    return HAL_ERROR;

  _1wire_TX((uint8_t[]){0xCC,            //  Skip ROM,
                        0x66,            //  COMMAND START
                        data_lenght + 5, //  Number of Bytes
                        0x11,            //  Write Sequencer Command
                        0x00,            //  ADDR_LO
                        0x00,            //  ADDR_HI
                        0xDD,            //  DELAY
                        0x03},           //  8ms
            8);
  if (_1wire_mode) // SPI
  {
    _1wire_TX((uint8_t[]) {0x80,         // SS low
                          0xC0,          // SPI Write
                          data_lenght,   // SPI write Lenght
                          0x00 }, // SPI read Lenght
              4);
    for (size_t i = 0; i < data_lenght; i++)
    {
      _1wire_TX((uint8_t[]){data_out[i]}, 1);
    }
    _1wire_TX((uint8_t[]){0x01}, // SS high
              1);
  }
  else // I2C
  {
    _1wire_TX((uint8_t[]){0x02,  //  I2C START
                          0xE3}, //  Write Data
              2);
  }
  for (uint8_t i = 0; i < data_lenght; i++)
  {
    _1wire_TX((uint8_t[]) {data_out[i]}, 1);
  }
  _1wire_TX((uint8_t[]){0x03}, // I2C STOP
            1);

  return _1wire_RUN(0x00, 0x00, data_lenght);
}

HAL_StatusTypeDef _1wire_RUN(uint8_t addr_lo, uint8_t addr_hi, uint8_t data_lenght)
{
  if (_1wire_reset() != HAL_OK)
    return HAL_ERROR;
  _1wire_TX((uint8_t[]){0xCC,             //  Skip ROM,
                        0x66,             //  COMMAND START
                        0x04,             // Number of Bytes
                        0x33,             // RUN SEQUENCER
                        0x00,             // ADDR_LO
                        data_lenght << 1, // SLEN_LO:ADDR_HI
                        0x00},            // SLEN_HI
            7);
  uint8_t CRC16_1 = _1wire_RX();
  uint8_t CRC16_2 = _1wire_RX();
  _1wire_TX((uint8_t[]){0xAA}, 1); // Release Byte
  HAL_Delay(100);
  uint8_t DUMMY_BYTE = read_byte();
  uint8_t LENGHT = read_byte();
  uint8_t RESULT = read_byte();
  if (RESULT == 0x88)
  {
    uint8_t RX_1 = read_byte(); // XX: CRC16
    uint8_t RX_2 = read_byte(); // XX: CRC16
  }
  uint8_t CRC16_3 = read_byte(); // XX: CRC16
  uint8_t CRC16_4 = read_byte(); // XX: CRC16
  if (RESULT != 0xAA)
    return HAL_ERROR;
  return HAL_OK;
}