/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>

// LCD
#include "LCD2x16.h"

// SD Card
#include "ff.h"
#include "diskio.h"

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

// Structure and variable definitions
FATFS   FatFs;              /* File system object for each logical drive */
FIL     Fil;                /* File objects */
DIR     Dir;                /* Directory object */

FRESULT fr;

// Analog measurement

// USED Load bank https://www.tindie.com/products/enginsubasi/simple-load-bank/

double volt         = 0;        // Calculated from ADC
double curr         = 0;
double voltFilt     = 0;        // Filtered values
double currFilt     = 0;

const double filterAlpha=0.1;   // EMAF coefficient

const double rshunt = 0.05;        // Ohm
const double ri     = 1100;     // Ohm
const double rf     = 22000;     // Ohm

double gain         = 0;                // Opamp amplifier gain rate

const double adcVoltageRef  = 3300;
const uint32_t adcUpValue   = 4095;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint32_t)  3)
static uint16_t   aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define TRUE 1
#define FALSE 0

/*
 * @about: Append data to file.
 */
uint8_t SD_Card_File_Write_Append ( char* path, char* txt, char* title, uint8_t retryNum )
{
    uint8_t retVal = FALSE;
    uint8_t retryNumDownCounter = retryNum;

    fr = FR_INVALID_PARAMETER;

    while ( retryNumDownCounter != 0 && fr != FR_OK )
    {
        fr = f_open ( &Fil, path, FA_READ | FA_WRITE );

        if ( fr == FR_OK )
        {
            fr = f_lseek ( &Fil, f_size ( &Fil ) );

            fr = f_write ( &Fil, txt, strlen ( txt ), NULL );

            fr = f_close ( &Fil );

            if ( fr == FR_OK )
            {
                retVal = TRUE;
            }
        }
        else if ( fr == FR_NO_FILE )
        {
            fr = f_open ( &Fil, path, FA_CREATE_ALWAYS );
            if ( fr == FR_OK )
            {
                fr = f_close ( &Fil );

                fr = f_open ( &Fil, path, FA_READ | FA_WRITE );
                fr = f_write ( &Fil, title, strlen ( title ), NULL );
                fr = f_write ( &Fil, txt, strlen ( txt ), NULL );
                fr = f_close ( &Fil );

                if ( fr == FR_OK )
                {
                    retVal = TRUE;
                }

            }
            else
            {
                f_mount ( &FatFs, "", 1 );
            }
        }
        else
        {
            f_mount ( &FatFs, "", 1 );
        }

        --retryNumDownCounter;
    }

    return ( retVal );
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t tempTickStr[ 64 ] = "";

  uint32_t cntr1sec=0;
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
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  // Calculate gain before calculation
  gain = 1 + ( rf / ri );

  HAL_ADCEx_Calibration_Start ( &hadc1 );

  fr=f_mount                           ( &FatFs, "", 1 );

  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  // Thanks to Olivier. https://github.com/4ilo/HD44780-Stm32HAL
  Lcd_PortType ports[] = {
            DB4_GPIO_Port, DB5_GPIO_Port, DB6_GPIO_Port, DB7_GPIO_Port
    };

  Lcd_PinType pins[] = {DB4_Pin, DB5_Pin, DB6_Pin, DB7_Pin};

  Lcd_HandleTypeDef lcd = Lcd_create(ports, pins, RS_GPIO_Port, RS_Pin, EN_GPIO_Port, EN_Pin, LCD_4_BIT_MODE);

  Lcd_string(&lcd, "4ilo");

  // Start measurement and filtering interrupt at 1 millisecond.
  HAL_TIM_Base_Start_IT ( &htim4 );

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if ( ( HAL_GetTick ( ) - cntr1sec ) > 999 )
	  {
	        cntr1sec = HAL_GetTick ( );

	        HAL_GPIO_WritePin ( LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET );

	        Lcd_clear ( &lcd );

            sprintf ( ( char* ) tempTickStr, "%lu", cntr1sec );

            Lcd_cursor ( &lcd, 0, 0 );
            Lcd_string ( &lcd, ( char* ) tempTickStr );

            Lcd_cursor ( &lcd, 1, 0 );
            Lcd_int ( &lcd, ( uint32_t ) voltFilt );

            Lcd_cursor ( &lcd, 1, 8 );
            Lcd_int ( &lcd, ( uint32_t ) currFilt );

            sprintf ( ( char* ) tempTickStr, "%lu\t%lu\t%lu\r\n", cntr1sec, ( uint32_t ) voltFilt, ( uint32_t ) currFilt );

            if ( HAL_GetTick ( ) > 20000 )
            {
                if ( SD_Card_File_Write_Append ( "test1.txt", ( char* ) tempTickStr, "", 3 ) == FALSE )
                {
                    HAL_GPIO_WritePin ( LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET );
                    HAL_GPIO_TogglePin ( LED2_GPIO_Port, LED2_Pin );
                }
            }



            HAL_GPIO_WritePin ( LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET );
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void Delay_NOP ( uint16_t DelayCount )
{
    uint16_t nopcounter;

    for (nopcounter = 0; nopcounter < DelayCount; nopcounter++)
    {
        __NOP();
    }
}

double adcToCurrentMilliamp ( uint32_t adcValue )
{
    // This calculation was not simplified for easy understanding.
    double current = 0;

    current = adcValue; /* to cast operation in double domain */

    current = ( ( current * adcVoltageRef ) / adcUpValue );

    current /= gain;

    current /= rshunt;

    return ( current );
}

void HAL_TIM_PeriodElapsedCallback ( TIM_HandleTypeDef *htim )
{
    /* 1 millisecond timer */
    if ( htim->Instance == TIM4 )
    {
        HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);

        volt = aADCxConvertedData[0];
        curr = adcToCurrentMilliamp(aADCxConvertedData[2]);

        volt = ( volt * adcVoltageRef ) / adcUpValue;

        voltFilt = ( volt * filterAlpha ) + ( ( 1 - filterAlpha ) * voltFilt );
        currFilt = ( curr * filterAlpha ) + ( ( 1 - filterAlpha ) * currFilt );

        HAL_ADC_Start_DMA ( &hadc1, ( uint32_t * ) aADCxConvertedData, ADC_CONVERTED_DATA_BUFFER_SIZE );

        HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
    }
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
