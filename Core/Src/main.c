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
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "moja_konfiguracja.h"
#include "stdint.h"
#include "arm_math.h"
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
uint16_t i,j,k;

uint8_t a,b,c,d,recive;

uint16_t z,x,y;
uint32_t d1,d2,d3,d4;

//////////// Timer 1/////////////////////////////////////////////////////////
uint8_t tim1_ch1,tim1_ch1n,tim1_ch2,tim1_ch2n,tim1_ch3,tim1_ch3n,tim1_ch4;
//////////// Timer 1//////////////////////////////////////////////////////

//////////// Hall sensor//////////////////////////////////////////////////////
uint8_t hall_1,hall_2,hall_3;
//////////// Hall sensor//////////////////////////////////////////////////////


//////////// SVPWM//////////////////////////////////////////////////////////////
uint8_t sv_sector;
float32_t sv_angle_current;
float32_t sv_Vref;
float32_t sv_T[3];  // 0 - T0 , 1- T1, 2- T2

//////////// SVPWM//////////////////////////////////////////////////////////////
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void SVPWM(uint8_t *sector,float32_t *angle_current,float32_t *Vref, float32_t T[])
{


	T[1]=((*Vref * sv_Tz)/sv_Vdc_limit) * arm_sin_f32((*sector * 1,0472 - (*angle_current))); /// pi/3 = 1,0472
	T[2]=((*Vref * sv_Tz)/sv_Vdc_limit) * arm_sin_f32(  (-(*sector-1)* 1,0472 +  *angle_current) );
	T[0]=sv_Tz-T[1]-T[2];



}


void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{


	z++;
	d1= HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    while((hadc1.Instance->ISR &= (0x1<<5))!=0){}
	d2 =HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
	while((hadc1.Instance->ISR &= (0x1<<5))!=0){}
	d3 =HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
	while((hadc2.Instance->ISR &= (0x1<<5))!=0){}
	d4 =HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
	while((hadc2.Instance->ISR &= (0x1<<5))!=0){}

	 HAL_ADCEx_InjectedStart_IT(&hadc1);
	 HAL_ADCEx_InjectedStart_IT(&hadc2);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_10)
	{
		if(d==0)
		{
			    HAL_TIM_Base_Start_IT(&htim1);

				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

			    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
			    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

			    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
			    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

			    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);



				// start_stop=1;
				// d=1;
		}
		else
		{
			start_stop=0;
			d=0;

		}


	}
}

void HAL_TIMEx_CommutCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM1)
	{


y++;

	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM1)
		{
		x++;



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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART2)
	{
		start_stop=0;
		HAL_UART_Receive_IT(&huart2, &recive, 1);

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  /* USER CODE BEGIN 2 */
  //////// konfiguracja Timer 2 - PWM input enkoder ////////////
   TIM2->ARR= TIM2_ARR;
   TIM2->PSC= TIM2_PSC;
   HAL_TIM_Base_Start(&htim2);

    //////// konfiguracja Timer 4 - HALL sensor ////////////
    TIM4->ARR= TIM4_ARR;
    TIM4->PSC= TIM4_PSC;
    TIM4->CCR1=TIM4_CCR1;
    TIM4->CCR2=TIM4_CCR2;
    HAL_TIMEx_HallSensor_Start(&htim4);


    //////// konfiguracja Timer 1  ////////////
    TIM1->ARR= TIM1_ARR;
    TIM1->PSC= TIM1_PSC;
    TIM1->CCR1=TIM1_CCR1;
    TIM1->CCR2=TIM1_CCR2;
    TIM1->CCR3=TIM1_CCR3;
    TIM1->CCR4=TIM1_CCR4;
    HAL_TIMEx_ConfigCommutEvent_IT(&htim1,TIM_TS_ITR3, TIM_COMMUTATION_TRGI);

    /////UASRT 2 ///////////////
    HAL_UART_Receive_IT(&huart2, &recive, 1);


    ////ADC 1/////
   if(HAL_OK== ((HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED)) && (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED))) );
   {
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc2);
   }





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  	  	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8))
	 	 		  tim1_ch1=1;
	 	 	  else
	 	 		  tim1_ch1=0;

	 	 	  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
	 	 		  tim1_ch1n=1;
	 	 	  else
	 	 		  tim1_ch1n=0;

	 	 	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9))
	 	 		  tim1_ch2=1;
	 	 	  else
	 	 		  tim1_ch2=0;

	  	  	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12))
	 	 		  tim1_ch2n=1;
	 	 	  else
	 	 		  tim1_ch2n=0;

	 	 	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10))
	 	 		  tim1_ch3=1;
	 	 	  else
	 	 		  tim1_ch3=0;

	 	 	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15))
	 	 		  tim1_ch3n=1;
	 	 	  else
	 	 		  tim1_ch3n=0;

	 	 	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11))
	 	 		  tim1_ch4=1;
	 	 	  else
	 	 		  tim1_ch4=0;





	 	 	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
	 	 		hall_1=1;
	 	 	else
	 	 		hall_1=0;

	 	 	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))
	 	 		hall_2=1;
	 	    else
	 	    	hall_2=0;

	 	 	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
	 	 		hall_3=1;
	 	    else
	 	    	hall_3=0;




	  /**



	 	if(start_stop==1)
	 	{
	 											if( a==1 && b==0 && c==0)
	 											{
	 												TIM1->CCR1=TIM2->CCR2;
	 												TIM1->CCR2=0;
	 												TIM1->CCR3=0;

	 												SET_CC1_T1;
	 												SET_CC1N_T2;
	 												SET_CC2_T3;
	 												SET_CC2N_T4;
	 												RESET_CC3_T5;
	 												RESET_CC3N_T6;
	 											}

	 											if( a==1 && b==1 && c==0)
	 											{
	 												TIM1->CCR1=TIM2->CCR2;
	 												TIM1->CCR2=0;
	 												TIM1->CCR3=0;

	 												SET_CC1_T1;
	 												SET_CC1N_T2;
	 												RESET_CC2_T3;
	 												RESET_CC2N_T4;
	 												SET_CC3_T5;
	 												SET_CC3N_T6;
	 											}


	 											if( a==0 && b==1 && c==0)
	 											{
	 												TIM1->CCR1=0;
	 												TIM1->CCR2=TIM2->CCR2;
	 												TIM1->CCR3=0;

	 												RESET_CC1_T1;
	 												RESET_CC1N_T2;
	 												SET_CC2_T3;
	 												SET_CC2N_T4;
	 												SET_CC3_T5;
	 												SET_CC3N_T6;
	 											}


	 											if( a==0 && b==1 && c==1)
	 											{
	 												TIM1->CCR1=0;
	 												TIM1->CCR2=TIM2->CCR2;
	 												TIM1->CCR3=0;

	 												SET_CC1_T1;
	 												SET_CC1N_T2;
	 												SET_CC2_T3;
	 												SET_CC2N_T4;
	 												RESET_CC3_T5;
	 												RESET_CC3N_T6;
	 											}


	 					    					if( a==0 && b==0 && c==1)
	 					    					{
	 					    						TIM1->CCR1=0;
	 					    						TIM1->CCR2=0;;
	 					    						TIM1->CCR3=TIM2->CCR2;

	 					    						SET_CC1_T1;
	 					    						SET_CC1N_T2;
	 					    						RESET_CC2_T3;
	 					    						RESET_CC2N_T4;
	 					    						SET_CC3_T5;
	 					    						SET_CC3N_T6;
	 					    					}


	 					    					if( a==1 && b==0 && c==1)
	 					    					{
	 					    						TIM1->CCR1=0;
	 					    						TIM1->CCR2=0;
	 					    						TIM1->CCR3=TIM2->CCR2;

	 					    						RESET_CC1_T1;
	 					    						RESET_CC1N_T2;
	 					    						SET_CC2_T3;
	 					    						SET_CC2N_T4;
	 					    						SET_CC3_T5;
	 					    						SET_CC3N_T6;
	 					    					}


	 	  	}
	 	  	else
	 	  	{

				//	TIM1->CCR1=0;
				//	TIM1->CCR2=0;
				//	TIM1->CCR3=0;
	 	  	}


**/
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
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
