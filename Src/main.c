#include "main.h"
#include "KMlib_1wire.h"
#include "KMlib_AD5282.h"
#include "KMlib_DS28E18.h"
#include "gpio.h"
#include "stm32_hal_legacy.h"
#include "stm32g4xx_hal.h"
#include "usart.h"


COM_InitTypeDef BspCOMInit;

void SystemClock_Config(void);

int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();

  BspCOMInit.BaudRate = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits = COM_STOPBITS_1;
  BspCOMInit.Parity = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE) {
    Error_Handler();
  }

  uint8_t ROMID[8] = {0};
  DS28E18_ROMID(ROMID);
  DS28E18_INIT();
  DS28E18_ROMID(ROMID);
  DS28E18_INIT();
  DS28E18_STATUS();
  DS28E18_CONFIG();
  DS28E18_WRITE_SEQ((uint8_t[]){POT0_WRITE, RDAC1_WRITE_O1_LOW, 0x00}, 3);
  HAL_Delay(100);
  DS28E18_WRITE_SEQ((uint8_t[]){POT0_WRITE, RDAC2_WRITE_O1_LOW, 0x00}, 3);
  HAL_Delay(100);
  DS28E18_WRITE_SEQ((uint8_t[]){POT3_WRITE, RDAC1_WRITE_O1_LOW, 0x00}, 3);
  HAL_Delay(100);
  DS28E18_WRITE_SEQ((uint8_t[]){POT3_WRITE, RDAC2_WRITE_O1_LOW, 0x00}, 3);
  HAL_Delay(100);

  while (1) {
    DS28E18_WRITE_SEQ((uint8_t[]){POT0_WRITE, RDAC1_WRITE_O1_HIGH, 0x50}, 3);
    HAL_Delay(3000);
    DS28E18_WRITE_SEQ((uint8_t[]){POT0_WRITE, RDAC1_WRITE_O1_LOW, 0x00}, 3);
    HAL_Delay(100);

    DS28E18_WRITE_SEQ((uint8_t[]){POT0_WRITE, RDAC2_WRITE_O1_HIGH, 0x50}, 3);
    HAL_Delay(3000);
    DS28E18_WRITE_SEQ((uint8_t[]){POT0_WRITE, RDAC2_WRITE_O1_LOW, 0x00}, 3);
    HAL_Delay(100);

    DS28E18_WRITE_SEQ((uint8_t[]){POT3_WRITE, RDAC1_WRITE_O1_HIGH, 0x50}, 3);
    HAL_Delay(3000);
    DS28E18_WRITE_SEQ((uint8_t[]){POT3_WRITE, RDAC1_WRITE_O1_LOW, 0x00}, 3);
    HAL_Delay(100);

    DS28E18_WRITE_SEQ((uint8_t[]){POT3_WRITE, RDAC2_WRITE_O1_HIGH, 0x50}, 3);
    HAL_Delay(3000);
    DS28E18_WRITE_SEQ((uint8_t[]){POT3_WRITE, RDAC2_WRITE_O1_LOW, 0x00}, 3);
    HAL_Delay(100);
  }
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {
  }
}
#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line) {}
#endif
