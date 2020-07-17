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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "moja_konfiguracja.h"
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
volatile uint8_t start_stop,licznik;
uint16_t i,j;
uint8_t a,b,c;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_10)
	{

				 HAL_TIMEx_PWMN_Start_IT(&htim1, TIM_CHANNEL_1);
				 HAL_TIMEx_PWMN_Start_IT(&htim1, TIM_CHANNEL_2);
				 HAL_TIMEx_PWMN_Start_IT(&htim1, TIM_CHANNEL_3);

				 HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1);
				 HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);

				 start_stop=1;

	}
}

void HAL_TIMEx_CommutCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM1)
	{



	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance==TIM2)
	{

		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)
		{

		//  TIM1->ARR=TIM2->CCR1;
		//  TIM1->CCR1=TIM2->CCR2;
		//  TIM1->CCR2=TIM2->CCR2;
		//  TIM1->CCR3=TIM2->CCR2;
		}
	}

}


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
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //////// konfiguracja Timer 2  ////////////
    TIM2->ARR=0xFFFF;
    TIM2->PSC=500;

  //////// konfiguracja Timer 4  ////////////
    TIM4->ARR=0xFFFF;
    TIM4->PSC=500;
    TIM4->CCR2=2;

    //////// konfiguracja Timer 1  ////////////
    TIM1->ARR=0;
    TIM1->PSC=0;
    TIM1->CCR1=0;
    TIM1->CCR2=0;
    TIM1->CCR3=0;
  //  HAL_TIMEx_ConfigCommutEvent_IT(&htim1,TIM_TS_ITR3, TIM_COMMUTATION_TRGI);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
	 		  a=1;
	 	  else
	 		  a=0;

	 	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
	 	  		  b=1;
	 	  else
	 		  b=0;

	 	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
	 	  		  c=1;
	 	  else
	 		  c=0;

	 	  	  if(start_stop==1)
	 	  	  {
	 											if( a==1 && b==0 && c==0)
	 											{
	 											SET_CC1_T1;
	 											RESET_CC1N_T2;
	 											RESET_CC2_T3;
	 											SET_CC2N_T4;
	 											RESET_CC3_T5;
	 											RESET_CC3N_T6;
	 											}

	 											if( a==1 && b==1 && c==0)
	 											{
	 											SET_CC1_T1;
	 											RESET_CC1N_T2;
	 											RESET_CC2_T3;
	 											RESET_CC2N_T4;
	 											RESET_CC3_T5;
	 											SET_CC3N_T6;
	 											}


	 											if( a==0 && b==1 && c==0)
	 											{
	 											RESET_CC1_T1;
	 											RESET_CC1N_T2;
	 											SET_CC2_T3;
	 											RESET_CC2N_T4;
	 											RESET_CC3_T5;
	 											SET_CC3N_T6;
	 											}


	 											if( a==0 && b==1 && c==1)
	 											{
	 					    					RESET_CC1_T1;
	 					    					SET_CC1N_T2;
	 					    					SET_CC2_T3;
	 					    					RESET_CC2N_T4;
	 					    					RESET_CC3_T5;
	 					    					RESET_CC3N_T6;
	 											}


	 					    					if( a==0 && b==0 && c==1)
	 					    					{
	 					    					RESET_CC1_T1;
	 					    					SET_CC1N_T2;
	 					    					RESET_CC2_T3;
	 					    					RESET_CC2N_T4;
	 					    					SET_CC3_T5;
	 					    					RESET_CC3N_T6;
	 					    					}


	 					    					if( a==1 && b==0 && c==1)
	 					    					{
	 					    					RESET_CC1_T1;
	 					    					RESET_CC1N_T2;
	 					    					RESET_CC2_T3;
	 					    					SET_CC2N_T4;
	 					    					SET_CC3_T5;
	 					    					RESET_CC3N_T6;
	 					    					}

	 					    					TIM1->ARR=TIM2->CCR1;
	 					    				    TIM1->CCR1=TIM2->CCR2;
	 					    					TIM1->CCR2=TIM2->CCR2;
	 					    					TIM1->CCR3=TIM2->CCR2;
	 	  	  	  }
	 	  	  	  else
	 	  	  	  {
					TIM1->ARR=0;
					TIM1->CCR1=0;
					TIM1->CCR2=0;
					TIM1->CCR3=0;
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

  /** Configure the main internal regulator output voltage 
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
