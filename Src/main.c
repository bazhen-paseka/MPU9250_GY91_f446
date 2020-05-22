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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

	#include <string.h>
	#include "stdio.h"
	#include "lcd.h"
	#include "i2c_techmaker_sm.h"
	#include "BMP280_sm.h"
	#include "MPU9250_sm.h"
	#include "MadgwickAHRS_sm.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_UART4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	char		debugString[150]	= {0};
	sprintf(debugString,"MPU-9250.\r\n");
	HAL_UART_Transmit(&huart4, (uint8_t *)debugString, strlen(debugString), 100);


	LCD_Init() ;
	LCD_SetRotation( 1 ) ;
	LCD_SetCursor( 0 , 0 ) ;
	LCD_FillScreen( ILI92_BLACK ) ;
	LCD_SetTextColor( ILI92_GREEN , ILI92_BLACK) ;
	sprintf(debugString,">> LCD_Init - Ok\r\n" );HAL_UART_Transmit(&huart4,(uint8_t*)debugString,strlen(debugString),100) ;
	LCD_Printf( "%s" , debugString ) ;

	#define	SOFT_VERSION	123
	int		soft_version_arr_int[3] = {0} ;
	soft_version_arr_int[0] 	= ((SOFT_VERSION) / 100) %10 ;
	soft_version_arr_int[1] 	= ((SOFT_VERSION) /  10) %10 ;
	soft_version_arr_int[2] 	= ((SOFT_VERSION)      ) %10 ;
	sprintf (	debugString																			,
				"MPU-9250.\r\n2020-May-22 v%d.%d.%d \r\nfor_debug UART4 115200/8-N-1\r\n"	,
				soft_version_arr_int[0]																,
				soft_version_arr_int[1]																,
				soft_version_arr_int[2]																) ;

	LCD_Printf( "%s" , debugString ) ;
	HAL_UART_Transmit(&huart4, (uint8_t *)debugString, strlen(debugString), 100);

	I2Cdev_init(&hi2c1);
	I2C_ScanBusFlow(&hi2c1, &huart4);

	double temp, press, alt;
	int8_t com_rslt;
	float ax, ay, az;
	float gx, gy, gz;
	float mx, my, mz;
	float aX, aY, aZ;	// angle
	uint32_t lastTime, currentTime, drawTime;

	#define ANGLEBASE 20.0
	#define PI180 0.017453292
	#define COS45 0.7071
	#define R1 25
	#define R2 50

	//LCD_Printf("Connecting to BMP280...\n");
	sprintf(debugString,"Connecting to BMP280... ");
	HAL_UART_Transmit(&huart4, (uint8_t *)debugString, strlen(debugString), 100);
	LCD_Printf("%s" , debugString);

	bmp280_t bmp280;
	com_rslt = BMP280_init(&bmp280);
	com_rslt += BMP280_set_power_mode(BMP280_NORMAL_MODE);
	com_rslt += BMP280_set_work_mode(BMP280_STANDARD_RESOLUTION_MODE);
	com_rslt += BMP280_set_standby_durn(BMP280_STANDBY_TIME_1_MS);
	if (com_rslt != SUCCESS) {
		sprintf(debugString," Check BMP280 connection! \t Program terminated!!!<<<<\r\n");
		HAL_UART_Transmit(&huart4, (uint8_t *)debugString, strlen(debugString), 100);
		LCD_Printf("%s" , debugString);
		//return 0;
	} else {
		//	LCD_Printf("Connection successful!\n");
		sprintf(debugString,"Connection successful!\r\n");
		HAL_UART_Transmit(&huart4, (uint8_t *)debugString, strlen(debugString), 100);
		LCD_Printf("%s" , debugString);
	}

	//LCD_Printf("Connecting to MPU9250...\n");
	sprintf(debugString,"Connecting to MPU9250...");
	HAL_UART_Transmit(&huart4, (uint8_t *)debugString, strlen(debugString), 100);
	LCD_Printf("%s" , debugString);

	while(!MPU9250_testConnection());
	MPU9250_initialize();
	MPU9250_setFullScaleGyroRange(MPU9250_GYRO_FS_1000); // 1000 град/сек
	MPU9250_setFullScaleAccelRange(MPU9250_ACCEL_FS_4); // 4g
	//LCD_Printf("Connection successful!\n\n");
	sprintf(debugString," connection successful!\r\n");
	HAL_UART_Transmit(&huart4, (uint8_t *)debugString, strlen(debugString), 100);
	LCD_Printf("%s" , debugString);

	Madgwick_init();
	lastTime = drawTime = HAL_GetTick();

	HAL_Delay(1000);
	LCD_FillScreen(ILI92_BLACK);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		sprintf(debugString,"cnt %d\r\n", (int)cnt_u32++);
//		HAL_UART_Transmit(&huart4, (uint8_t *)debugString, strlen(debugString), 100);
//		LCD_Printf( "%s" , debugString ) ;

		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		HAL_Delay(1000);

		BMP280_read_temperature_double(&temp);
		BMP280_read_pressure_double(&press);
		alt = BMP280_calculate_altitude(102900); // insert actual data here

		MPU9250_getMotion9Real(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

		currentTime = HAL_GetTick();
		Madgwick_update(gx, gy, gz, ax, ay, az, mx, my, mz, (currentTime - lastTime)/1000.0);
		lastTime = currentTime;

		aX = Madgwick_getPitch() ;
		aY = Madgwick_getRoll()  ;
		aZ = Madgwick_getYaw() - 180.0 ;

		LCD_SetCursor(0, 0);
		LCD_FillScreen(ILI92_BLACK);
		//LCD_Printf("T: %6.2f C  P: %6.0f Pa  A: %3.0f m\n", temp, press, alt);
		sprintf(debugString,"T: %6.2f C  P: %6.0f Pa  A: %3.0f m\r\n", temp, press, alt);
		HAL_UART_Transmit(&huart4, (uint8_t *)debugString, strlen(debugString), 100);
		LCD_Printf("%s" , debugString);

//	LCD_Printf("Accel:   %7.4f %7.4f %7.4f\n", ax, ay, az);
		sprintf(debugString,"Accel:   %7.4f %7.4f %7.4f\r\n", ax, ay, az);
		HAL_UART_Transmit(&huart4, (uint8_t *)debugString, strlen(debugString), 100);
		LCD_Printf("%s" , debugString);

//	LCD_Printf("Gyro:    %7.4f %7.4f %7.4f\n", gx, gy, gz);
		sprintf(debugString,"Gyro:    %7.4f %7.4f %7.4f\r\n", gx, gy, gz);
		HAL_UART_Transmit(&huart4, (uint8_t *)debugString, strlen(debugString), 100);
		LCD_Printf("%s" , debugString);

//	LCD_Printf("Compass: %7.1f %7.1f %7.1f\n\n", mx, my, mz);
		sprintf(debugString,"Compass: %7.1f %7.1f %7.1f\r\n", mx, my, mz);
		HAL_UART_Transmit(&huart4, (uint8_t *)debugString, strlen(debugString), 100);
		LCD_Printf("%s" , debugString);


//	LCD_Printf("Madg: X: %5.1f Y: %5.1f Z: %5.1f\n", aX, aY, aZ);
		sprintf(debugString,"Madg: X: %5.1f Y: %5.1f Z: %5.1f\r\n\r\n", aX, aY, aZ);
		HAL_UART_Transmit(&huart4, (uint8_t *)debugString, strlen(debugString), 100);
		LCD_Printf("%s" , debugString);
		//LCD_Printf("Max : X: %5.1f Y: %5.1f Z: %5.1f\n", aXmax, aYmax, aZmax);
		//LCD_Printf("Min : X: %5.1f Y: %5.1f Z: %5.1f\n", aXmin, aYmin, aZmin);

		//LCD_Printf("Old_Madg: P: %5.1f R: %5.1f Y: %5.1f\n", Madgwick_getPitch(), Madgwick_getRoll(), Madgwick_getYaw() );

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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
