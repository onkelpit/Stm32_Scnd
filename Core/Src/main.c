/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const uint16_t BME_ADDR = (0x76 << 1);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
double getTempInCelsius(uint16_t raw);
double getTempInKelvin(uint16_t raw);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t myTxData[13] = "Hello World\r\n";
uint8_t myRxData[11];
uint32_t raw;
char msg[16];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart2, myRxData, 11);
  HAL_ADCEx_Calibration_Start(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      //HAL_UART_Transmit(&huart2, myTxData, 13, 10);
      /* HAL_ADC_Start_DMA(&hadc1, &raw, 1); */
      //HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
      //raw = HAL_ADC_GetValue(&hadc1);
      HAL_StatusTypeDef ret;
      uint8_t buf[2] = {0};
      //ret = HAL_I2C_Mem_Read(&hi2c1, BME_ADDR, 0xD0, 1, buf, 1, HAL_MAX_DELAY);
      char msg2[64] = {0};
      //sprintf(msg2, "0x%x (%d)\r\n", buf[0], ret);
      //HAL_UART_Transmit(&huart2, (uint8_t*)msg2, strlen(msg2), 10);
      buf[0] = 0xd0;
      ret = HAL_I2C_Master_Transmit(&hi2c1, BME_ADDR, buf, 1, HAL_MAX_DELAY);
      if ( ret != HAL_OK ) {
          sprintf(msg2, "UART ERROR\r\n");
          HAL_UART_Transmit(&huart2, (uint8_t*)msg2, strlen(msg2), 10);
      } else {
          buf[0] = 0;
      ret = HAL_I2C_Master_Receive(&hi2c1, BME_ADDR, buf, 1, HAL_MAX_DELAY);
          sprintf(msg2, "0x%x\r\n", buf[0]);
          HAL_UART_Transmit(&huart2, (uint8_t*)msg2, strlen(msg2), 10);
      }
      HAL_Delay(500);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    HAL_UART_Transmit(&huart2, myRxData, 11, 10);
}
double getTempInKelvin(uint16_t raw) {
    double resistance = 10000.0;
    double a = 1.129148e-3;
    double b = 2.34125e-4;
    double c = 8.76741e-8;
    double voltage = raw / 4096 * 3.3;
    double r2 = ((4096*resistance/raw)-resistance);
    double log_r = log(r2);
    double log_3r = log_r * log_r * log_r;
    double K = 9.5;
    double tempInKelvin = (1/(a+b*log_r + c*log_3r))-voltage*voltage/(K*resistance);
    return tempInKelvin;
}
double getTempInCelsius(uint16_t raw) {
    //double tempInCelsius = getTempInKelvin(raw) - 273.15;
    double Vin= 3.3;
    double Vout= 0;
    double R1= 10000; //change to your system
    double Ro= 10000; //change to your system
    double R2= 0;
    double ratio= 0;
    double Temp = 0;
    double Beta = 3950; //change to your system
    double To = 298.15;
    ratio = raw * Vin;
    Vout = (ratio)/4096.0;
    ratio = (Vin/Vout) -1;
    R2 = R1 *ratio;
    Temp = Beta/log(R2/(Ro*exp(-Beta/To)));
    Temp -= 273.15;
	return Temp;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
      sprintf(msg, "%lu\r\n", raw);
      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);
      sprintf(msg, "%4.2fC\r\n", getTempInCelsius(raw));
      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);
      sprintf(msg, "%4.2fK\r\n", getTempInKelvin(raw));
      HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
