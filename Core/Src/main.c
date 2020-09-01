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
#include "cJSON.h"


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

////////////??????????????????????????????????/////////////////////////////////////////////////////////
volatile float32_t t;
volatile float32_t t1,t2,t3;
volatile uint32_t a,b,c,d,p;


//////////// MAIN_COMMON/////////////////////////////////////////////////////////
volatile uint8_t sector;
volatile float32_t angle_current_deg,angle_current_rad, angle_rotor_deg,angle_rotor_rad;
volatile float32_t pSinVal,pCosVal;
volatile float32_t Vref;
volatile uint8_t start,licznik;

//////////// MAIN_COMMON/////////////////////////////////////////////////////////

//////////// PID /////////////////////////////////////////////////////////
volatile float32_t Ialpha,Ibeta,Id,Iq;
volatile float32_t Valpha,Vbeta,Vd,Vq,Vd_prev,Vq_prev;

volatile float32_t set_d, ed;
volatile arm_pid_instance_f32 pid_d;

volatile float32_t set_q, eq;
volatile arm_pid_instance_f32 pid_q;

//////////// PID /////////////////////////////////////////////////////////

//////////// SPEED PID /////////////////////////////////////////////////////////
volatile float32_t set_speed, e_speed, speed;
volatile arm_pid_instance_f32 pid_iq_speed;
volatile float32_t iq_speed, iq_speed_prev;
volatile uint32_t capture_tim8_ccr2;
volatile uint32_t index_speed_loop;
//////////// SPEED PID /////////////////////////////////////////////////////////


//////////// ADC /////////////////////////////////////////////////////////
volatile int32_t adc_Ia,adc_Ib,adc_Ic,adc_V,offset1,offset2,offset3;
volatile uint32_t index_event_adc;
volatile float32_t Ia,Ib,Ic;
volatile int32_t sum_currents;

//////////// ADC /////////////////////////////////////////////////////////

//////////// Timer 1/////////////////////////////////////////////////////////
uint8_t tim1_ch1,tim1_ch1n,tim1_ch2,tim1_ch2n,tim1_ch3,tim1_ch3n,tim1_ch4;
volatile uint32_t index_event_timer1;
//////////// Timer 1////////////////////////////////////////////////////////


//////////// SVPWM//////////////////////////////////////////////////////////////
volatile float32_t sv_T[3];  // [0] - T0 , [1]- T1, [2]- T2
volatile float32_t sv_T_gate[4]; // [3]- T1+T2+T0/2, [1]- T1+T0/2, [2]- T2+T0/2, T[0]=T0/2
volatile float32_t sv_S1; // switch CH1, CH2  ,CH3
volatile float32_t sv_S2;
volatile float32_t sv_S3;
//////////// SVPWM//////////////////////////////////////////////////////////////


//////////// USART 2////////////////////////////////////////////////////////
volatile uint8_t jstring[size_uart_tab];
volatile uint16_t index_uart;
volatile uint8_t recive;
//////////// USART 2////////////////////////////////////////////////////////


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


void start_up(void)
{

	 if(HAL_OK== ((HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED)) && (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED))) )
	   {
		if(HAL_OK== (HAL_OPAMPEx_SelfCalibrateAll(&hopamp1, &hopamp2, &hopamp3)))
		{

			//////// konfiguracja Timer 1  //////////////////////////
			TIM1->ARR= TIM1_ARR;
			TIM1->PSC= TIM1_PSC;

			TIM1->CCR1=(TIM1->ARR/10);
			TIM1->CCR2=0;
			TIM1->CCR3=0;
			TIM1->CCR4=TIM1_CCR4;

			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
			//HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);


			HAL_Delay(800);

			//////// konfiguracja Timer 4 - encoder ///////////////////
			TIM4->ARR= TIM4_ARR;
			TIM4->PSC= TIM4_PSC;
			HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
			HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);
			//////// konfiguracja Timer 4 - encoder ///////////////////

			HAL_Delay(400);

			TIM1->CCR1=0;
			TIM1->CCR2=0;
			TIM1->CCR3=0;

			HAL_Delay(200);


			//////// konfiguracja Timer 8  ///////////////////
			TIM8->ARR= TIM8_ARR;
			TIM8->PSC= TIM8_PSC;
			HAL_TIM_IC_Start(&htim8, TIM_CHANNEL_2);


			//////// start ADC 1 2 ///////////////////////////////////
			HAL_OPAMP_Start(&hopamp1);
			HAL_OPAMP_Start(&hopamp2);
			HAL_OPAMP_Start(&hopamp3);
			//////// start ADC 1 2 ///////////////////////////////////
			HAL_ADCEx_InjectedStart_IT(&hadc1);
			HAL_ADCEx_InjectedStart_IT(&hadc2);



			/////////// inicjalizacja pid_d ////////////////
			set_d=0;
			pid_d.Kp=1;
			pid_d.Ki=1;
			pid_d.Kd=0;
			arm_pid_init_f32(&pid_d, 1);

			/////////// inicjalizacja pid_q ////////////////
			set_q=0.5;
			pid_q.Kp=4;
			pid_q.Ki=1;
			pid_q.Kd=0;
			arm_pid_init_f32(&pid_q, 1);

			/////////// inicjalizacja pid_speed ////////////////
			set_speed=2200;
			pid_iq_speed.Kp=5;
			pid_iq_speed.Ki=5;
			pid_iq_speed.Kd=0;
			arm_pid_init_f32(&pid_iq_speed, 1);
		}


	   }

}

void start1(void)
{

			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

}

void stop(void)
{

	TIM1->CCR1=0;
	TIM1->CCR2=0;
	TIM1->CCR3=0;

	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);


	arm_pid_reset_f32(&pid_d);
	arm_pid_reset_f32(&pid_q);
	arm_pid_reset_f32(&pid_iq_speed);

	arm_pid_init_f32(&pid_d, 1);
	arm_pid_init_f32(&pid_q, 1);
	arm_pid_init_f32(&pid_iq_speed, 1);



}

void AlphaBeta_To_Angle_Vref(float32_t alpha,float32_t beta,float32_t *angle_current_rad,float32_t *Vref)
{
	*angle_current_rad = atan2f(beta,alpha);
	arm_sqrt_f32( ((alpha*alpha)+(beta*beta)), Vref);

	 if(*Vref>=sv_Vdc_limit)  // saturacja Vref
	    	*Vref=sv_Vdc_limit;
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


void SVPWM(uint8_t sector,float32_t angle_current_rad,float32_t Vref, float32_t T[], float32_t T_gate[], float32_t *S1,float32_t *S2,float32_t *S3)
{

	T[1]=sv_modulation * ((Vref * sv_Tz)/sv_Vdc_limit) * arm_sin_f32((sector * 1.047197) - (angle_current_rad)); /// pi/3 = 1,0472
	T[2]=sv_modulation * ((Vref * sv_Tz)/sv_Vdc_limit) * arm_sin_f32((-(sector-1) * 1.047197) +  angle_current_rad) ;
	T[0]=sv_Tz-T[1]-T[2];

	t1=T[1];
	t2=T[2];
	t3=T[0];

	T_gate[0]= (T[0]/2);
	T_gate[1]= T[1]+(T_gate[0]);
	T_gate[2]= T[2]+(T_gate[0]);
	T_gate[3]= T[1]+T[2]+(T_gate[0]);


	if(sector == 1)
	{
		*S1=T_gate[3];
		*S2=T_gate[2];
		*S3=T_gate[0];
	}
	else if(sector == 2)
	{
		*S1=T_gate[1];
		*S2=T_gate[3];
		*S3=T_gate[0];
	}
	else if(sector == 3)
	{
		*S1=T_gate[0];
		*S2=T_gate[3];
		*S3=T_gate[2];
	}
	else if(sector == 4)
	{
		*S1=T_gate[0];
		*S2=T_gate[1];
		*S3=T_gate[3];
	}
	else if(sector == 5)
	{
		*S1=T_gate[2];
		*S2=T_gate[0];
		*S3=T_gate[3];
	}
	else if(sector == 6)
	{
		*S1=T_gate[3];
		*S2=T_gate[0];
		*S3=T_gate[1];
	}
	else{}

}


void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	capture_tim8_ccr2= TIM8->CCR2;
	if(capture_tim8_ccr2 <= 0)
		speed=0;
	else
		speed=revolution_per_min/capture_tim8_ccr2;


	adc_Ia= HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    while((hadc1.Instance->ISR &= (0x1<<5))!=0){}
    adc_Ic =HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
	while((hadc1.Instance->ISR &= (0x1<<5))!=0){}
	adc_Ib =HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
	while((hadc2.Instance->ISR &= (0x1<<5))!=0){}
	//adc_V =HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
	//while((hadc2.Instance->ISR &= (0x1<<5))!=0){}

	if(index_event_adc<300)
	{
		Ia=0;
		Ib=0;
		Ic=0;
		index_event_adc++;


	}
	else if(index_event_adc == 300)
	{
			   offset1=adc_Ia;
			   offset2=adc_Ib;
			   offset3=adc_Ic;
			   index_event_adc++;
	}
	else
	{
	 //   sum_currents=(adc_Ia-offset1)+(adc_Ic-offset3)+(adc_Ib-offset2);

	    adc_Ia=(adc_Ia-offset1);
	    adc_Ib=(adc_Ib-offset2);
	    adc_Ic=(adc_Ic-offset3);

	    Ia=-adc_Ia/33.0;
	    Ib=-adc_Ib/33.0;
	    Ic=-adc_Ic/33.0;

	        arm_clarke_f32(Ia, Ib, &Ialpha, &Ibeta);
	    	angle_rotor_deg=TIM4->CCR1;
	    	arm_sin_cos_f32(angle_rotor_deg, &pSinVal, &pCosVal);
	    	arm_park_f32(Ialpha, Ibeta, &Id, &Iq, pSinVal, pCosVal);



	    	// pid speed
	   							index_speed_loop++;
	   							if(index_speed_loop==1)
	   							{

	    						e_speed=set_speed-speed;
	    						iq_speed_prev=pid_iq_speed.state[2];
	    						iq_speed=arm_pid_f32(&pid_iq_speed, e_speed);
	    	// saturacja i anty-wind-up
	    						if(iq_speed>=current_limit_max_iq)
	    						{
	    							pid_iq_speed.state[2]=iq_speed_prev;
	    							iq_speed=current_limit_max_iq;
	    						}

	    						if(iq_speed<=current_limit_min_iq)
	    						{
	    							pid_iq_speed.state[2]=iq_speed_prev;
	    							iq_speed=current_limit_min_iq;
	    						}
	   							}
	   							if(index_speed_loop==5)
	   								index_speed_loop=0;



	    	// pid dla osi d
	    						ed=set_d-Id;
	    						Vd_prev=pid_d.state[2];
	    						Vd=arm_pid_f32(&pid_d, ed);
	    	// saturacja i anty-wind-up
	    						if(Vd>=sv_Vdc_limit)
	    						{
	    							pid_d.state[2]=Vd_prev;
	    							Vd=sv_Vdc_limit;
	    						}

	    						if(Vd<=(-sv_Vdc_limit))
	    						{
	    							pid_d.state[2]=Vd_prev;
	    							Vd=(-sv_Vdc_limit);
	    						}

	    	// pid dla osi q
	    						//eq=set_q-Iq;
	    						eq=iq_speed-Iq;
	    						Vq_prev=pid_q.state[2];
	    						Vq=arm_pid_f32(&pid_q, eq);
	    	// saturacja i anty-wind-up
	    						if(Vq>=sv_Vdc_limit)
	    						{
	    						pid_q.state[2]=Vq_prev;
	    						Vq=sv_Vdc_limit;
	    						}

	    						if(Vq<=(-sv_Vdc_limit))
	    						{
	    						pid_q.state[2]=Vq_prev;
	    						Vq=(-sv_Vdc_limit);
	    						}

	    	//angle_rotor_deg=TIM4->CCR1;
	    	//arm_sin_cos_f32(angle_rotor_deg, &pSinVal, &pCosVal);
	    	arm_inv_park_f32(Vd, Vq, &Valpha, &Vbeta, pSinVal, pCosVal);

	    	AlphaBeta_To_Angle_Vref(Valpha, Vbeta, &angle_current_rad, &Vref);
	    	Angle_To_Sector(angle_current_rad, &sector);
	    	SVPWM(sector, angle_current_rad , Vref, sv_T, sv_T_gate, &sv_S1, &sv_S2, &sv_S3);

	    	TIM1->CCR1 = sv_S1;
	    	TIM1->CCR2 = sv_S2;
	    	TIM1->CCR3 = sv_S3;
	}

	 HAL_ADCEx_InjectedStart_IT(&hadc1);
	 HAL_ADCEx_InjectedStart_IT(&hadc2);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_10)
	{
		if(start==0)
		{
			start=1;
			start1();

		}
		else
		{
			start=0;
			stop();

		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM3)
	{

	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART2)
	{
			//if(index_uart==0)
			//	stop();



			jstring[index_uart]=recive;

			if(recive=='}')
			{

				for (int i=(index_uart+1);i<size_uart_tab;i++)
				{
					jstring[i]=0;
				}
				index_uart=0;

				/////////// parse JSON ///////////////////////////////
				 cJSON * root = cJSON_Parse((char *)jstring);
				 cJSON * speed = cJSON_GetObjectItemCaseSensitive(root, "speed");
				 set_speed =  atoi(cJSON_GetStringValue(speed));

				 cJSON * current = cJSON_GetObjectItemCaseSensitive(root, "current");
				 set_q =  atoi(cJSON_GetStringValue(current));

				 cJSON * iq_Kp = cJSON_GetObjectItemCaseSensitive(root, "iq_Kp");
				 pid_q.Kp =  atoi(cJSON_GetStringValue(iq_Kp));

				 cJSON * iq_Ki = cJSON_GetObjectItemCaseSensitive(root, "iq_Ki");
				 pid_q.Ki =  atoi(cJSON_GetStringValue(iq_Ki));

				 cJSON * id_Kp = cJSON_GetObjectItemCaseSensitive(root, "id_Kp");
				 pid_d.Kp =  atoi(cJSON_GetStringValue(id_Kp));

				 cJSON * id_Ki = cJSON_GetObjectItemCaseSensitive(root, "id_Ki");
				 pid_d.Ki =  atoi(cJSON_GetStringValue(id_Ki));



			     cJSON_Delete(speed);
			     cJSON_Delete(root);
			 	for(int i=0;i<size_uart_tab;i++)
			 	{
			 		jstring[i]=0;
			 	}

				//start1();
			}
			else
				index_uart++;

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */


  HAL_UART_Receive_IT(&huart2, &recive, 1);

   start_up();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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
  RCC_OscInitStruct.PLL.PLLN = 85;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK)
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
