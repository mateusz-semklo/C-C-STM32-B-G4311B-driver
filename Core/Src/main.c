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
#include "math.h"


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

//////////// MAIN_COMMON/////////////////////////////////////////////////////////
volatile uint8_t sector;
volatile float32_t angle_current_deg,angle_current_rad, angle_rotor_deg,angle_rotor_rad;
volatile float32_t Vref;
//////////// MAI_COMMON/////////////////////////////////////////////////////////

//////////// ADC /////////////////////////////////////////////////////////
volatile uint32_t adc_measure[4],z;
//////////// ADC /////////////////////////////////////////////////////////

//////////// Timer 1/////////////////////////////////////////////////////////
uint8_t tim1_ch1,tim1_ch1n,tim1_ch2,tim1_ch2n,tim1_ch3,tim1_ch3n,tim1_ch4;
//////////// Timer 1////////////////////////////////////////////////////////

///!!!!!!!!!!!!!!!!!!!!!//
volatile uint32_t i;
volatile float32_t k,t,Ia,Ib,Ialpha,Ibeta,kat;


//////////// SVPWM//////////////////////////////////////////////////////////////
volatile float32_t sv_T[3];  // [0] - T0 , [1]- T1, [2]- T2
volatile float32_t sv_T_gate[4]; // [3]- T1+T2+T0/2, [1]- T1+T0/2, [2]- T2+T0/2, T[0]=T0/2
volatile float32_t sv_S[3]; // switch CH1, CH2  ,CH3
volatile float32_t sv_S1;
volatile float32_t sv_S2;
volatile float32_t sv_S3;
//////////// SVPWM//////////////////////////////////////////////////////////////


//////////// USART 2////////////////////////////////////////////////////////
volatile uint8_t recive,trans;
//////////// USART 2////////////////////////////////////////////////////////


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void start_up(void)
{
	 if(HAL_OK== ((HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED)) && (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED))) )
	   {
		//////// start ADC 1 2 ///////////////////////////////////
	    HAL_ADCEx_InjectedStart_IT(&hadc1);
	    HAL_ADCEx_InjectedStart_IT(&hadc2);

	    //////// konfiguracja Timer 1  //////////////////////////
	    TIM1->ARR= TIM1_ARR;
	    TIM1->PSC= TIM1_PSC;

		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
	    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	    TIM1->CCR1=(TIM1->ARR/20);
	    TIM1->CCR2=0;
	    TIM1->CCR3=0;

	    HAL_Delay(1000);

	    TIM1->CCR1=0;
	    TIM1->CCR2=0;
	    TIM1->CCR3=0;

	    //////// konfiguracja Timer 4 - encoder ///////////////////
	     TIM4->ARR= TIM4_ARR;
	     TIM4->PSC= TIM4_PSC;
	     HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
	     HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);

	     ////////  UASRT 2 /////////////////////////////////////////
	     HAL_UART_Receive_IT(&huart2, &recive, 1);


	   }

}

void AlphaBeta_To_Angle(float32_t Valpha,float32_t Vbeta,float32_t *angle_current_rad)
{


}
void Angle_To_Sector(float32_t angle_current_rad,uint8_t *sector)
{


	if((angle_current_rad>0) && (angle_current_rad<=1.047197)) // pi/3
		*sector=1;
	else if((angle_current_rad>1.047197) && (angle_current_rad<=2.094395)) //2/3*pi
		*sector=2;
	else if((angle_current_rad>2.094395) && (angle_current_rad<=3.141593))
		*sector=3;
	else if((angle_current_rad>-3.141593) && (angle_current_rad<=-2.094395))
		*sector=4;
	else if((angle_current_rad>-2.094395) && (angle_current_rad<=-1.047197))
		*sector=5;
	else if ((angle_current_rad>-1.047197) && (angle_current_rad<=0))
		*sector=6;
	else{}




}


void SVPWM(uint8_t sector,float32_t angle_current_rad,float32_t Vref, float32_t T[], float32_t T_gate[], float32_t S[])
{


	T[1]=((Vref * sv_Tz)/sv_Vdc_limit) * arm_sin_f32((sector * 1.0472) - (angle_current_rad)); /// pi/3 = 1,0472
	T[2]=((Vref * sv_Tz)/sv_Vdc_limit) * arm_sin_f32((-(sector-1) * 1.0472) +  angle_current_rad) ;
	T[0]=sv_Tz-T[1]-T[2];

	T_gate[0]= (T[0]/2);
	T_gate[1]= T[1]+(T_gate[0]);
	T_gate[2]= T[2]+(T_gate[0]);
	T_gate[3]= T[1]+T[2]+(T_gate[0]);


	if(sector == 1)
	{
		S[0]=T_gate[3];
		S[1]=T_gate[2];
		S[2]=T_gate[0];
	}
	else if(sector == 2)
	{
		S[0]=T_gate[1];
		S[1]=T_gate[3];
		S[2]=T_gate[0];
	}
	else if(sector == 3)
	{
		S[0]=T_gate[0];
		S[1]=T_gate[3];
		S[2]=T_gate[2];
	}
	else if(sector == 4)
	{
		S[0]=T_gate[0];
		S[1]=T_gate[1];
		S[2]=T_gate[3];
	}
	else if(sector == 5)
	{
		S[0]=T_gate[2];
		S[1]=T_gate[0];
		S[2]=T_gate[3];
	}
	else if(sector == 6)
	{
		S[0]=T_gate[3];
		S[1]=T_gate[0];
		S[2]=T_gate[1];
	}
	else{}


	sv_S1=S[0];
	sv_S2=S[1];
	sv_S3=S[2];




}


void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{


	z++;
	adc_measure[0]= HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    while((hadc1.Instance->ISR &= (0x1<<5))!=0){}
    adc_measure[1] =HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
	while((hadc1.Instance->ISR &= (0x1<<5))!=0){}
	adc_measure[2] =HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
	while((hadc2.Instance->ISR &= (0x1<<5))!=0){}
	adc_measure[3] =HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
	while((hadc2.Instance->ISR &= (0x1<<5))!=0){}

	 HAL_ADCEx_InjectedStart_IT(&hadc1);
	 HAL_ADCEx_InjectedStart_IT(&hadc2);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_10)
	{



	}
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM1)
		{



		}

	if(htim->Instance==TIM2)
		{
		t=t+0.001;
		Ia=arm_sin_f32( PI * t); //
		Ib=arm_sin_f32( ( PI *t) - 2.094395);  // 2/3*pi = 2.094395
		arm_clarke_f32(Ia, Ib, &Ialpha, &Ibeta);

		kat= atan2f(Ibeta,Ialpha);


		 Angle_To_Sector(kat , &sector);

				  SVPWM(sector, kat , Vref, sv_T, sv_T_gate, sv_S);







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
		//HAL_UART_Receive_IT(&huart2, &recive, 1);

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

  TIM2->PSC=3;
  TIM2->ARR=39999;

  Vref= sv_Vdc * sv_limit_max_voltage;

  HAL_TIM_Base_Start_IT(&htim2);

    start_up();

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
