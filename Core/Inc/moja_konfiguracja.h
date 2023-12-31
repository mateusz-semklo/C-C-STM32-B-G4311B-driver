/*
 * moja_konfiguracja.h
 *
 *  Created on: Jun 6, 2020
 *      Author: Mateusz
 */

#ifndef INC_MOJA_KONFIGURACJA_H_
#define INC_MOJA_KONFIGURACJA_H_

#define SET_MODE_OCC1_PWM1       TIM1->CCMR1=((0xFF8F & TIM1->CCMR1) | (0x6<<4))
#define SET_MODE_OCC2_PWM1       TIM1->CCMR1=((0x8FFF & TIM1->CCMR1) | (0x6<<12))
#define SET_MODE_OCC3_PWM1       TIM1->CCMR2=((0xFF8F & TIM1->CCMR2) | (0x6<<4))
#define SET_MODE_OCC4_PWM1       TIM1->CCMR2=((0x8FFF & TIM1->CCMR2) | (0x6<<12))

#define SET_MODE_OCC1_F_ACTIVE   TIM1->CCMR1=((0xFF8F & TIM1->CCMR1) | (0x5<<4))
#define SET_MODE_OCC2_F_ACTIVE   TIM1->CCMR1=((0x8FFF & TIM1->CCMR1) | (0x5<<12))
#define SET_MODE_OCC3_F_ACTIVE   TIM1->CCMR2=((0xFF8F & TIM1->CCMR2) | (0x5<<4))
#define SET_MODE_OCC4_F_ACTIVE   TIM1->CCMR2=((0x8FFF & TIM1->CCMR2) | (0x5<<12))

#define SET_MODE_OCC1_F_INACTIVE   TIM1->CCMR1=((0xFF8F & TIM1->CCMR1) | (0x4<<4))
#define SET_MODE_OCC2_F_INACTIVE   TIM1->CCMR1=((0x8FFF & TIM1->CCMR1) | (0x4<<12))
#define SET_MODE_OCC3_F_INACTIVE   TIM1->CCMR2=((0xFF8F & TIM1->CCMR2) | (0x4<<4))
#define SET_MODE_OCC4_F_INACTIVE   TIM1->CCMR2=((0x8FFF & TIM1->CCMR2) | (0x4<<12))

#define SET_MODE_OCC1_DISTABLE   TIM1->CCMR1=((0xFF8F & TIM1->CCMR1) | (0x0<<4))
#define SET_MODE_OCC2_DISTABLE   TIM1->CCMR1=((0x8FFF & TIM1->CCMR1) | (0x0<<12))
#define SET_MODE_OCC3_DISTABLE   TIM1->CCMR2=((0xFF8F & TIM1->CCMR2) | (0x0<<4))
#define SET_MODE_OCC4_DISTABLE   TIM1->CCMR2=((0x8FFF & TIM1->CCMR2) | (0x0<<12))

#define SET_MODE_OCC1_SET_ACTIVE   TIM1->CCMR1=((0xFF8F & TIM1->CCMR1) | (0x1<<4))
#define SET_MODE_OCC2_SET_ACTIVE   TIM1->CCMR1=((0x8FFF & TIM1->CCMR1) | (0x1<<12))
#define SET_MODE_OCC3_SET_ACTIVE   TIM1->CCMR2=((0xFF8F & TIM1->CCMR2) | (0x1<<4))
#define SET_MODE_OCC4_SET_ACTIVE   TIM1->CCMR2=((0x8FFF & TIM1->CCMR2) | (0x1<<12))


#define RESET_OSSR         	     TIM1->BDTR&=(~(0x1<<11))
#define RESET_OSSI         	     TIM1->BDTR&=(~(0x1<<10))
#define RESET_MOE                TIM1->BDTR&=(~(0x1<<15))
#define SET_OSSR				 TIM1->BDTR|=(0x1<<11)
#define SET_OSSI				 TIM1->BDTR|=(0x1<<10)
#define SET_MOE                  TIM1->BDTR|=(0x1<<15)

#define SET_CC1_T1_CC2N_T4             TIM1->CCER=((TIM1->CCER & 0x2AAA) | ((0x1<<0) | (0x1<<6)))  // 0x2AAA = 10 1010 1010 1010
#define SET_CC1_T1_CC3N_T6             TIM1->CCER=((TIM1->CCER & 0x2AAA) | ((0x1<<0) | (0x1<<10)))

#define SET_CC2_T3_CC3N_T6             TIM1->CCER=((TIM1->CCER & 0x2AAA) | ((0x1<<4) | (0x1<<10)))
#define SET_CC2_T3_CC1N_T2             TIM1->CCER=((TIM1->CCER & 0x2AAA) | ((0x1<<4) | (0x1<<2)))

#define SET_CC3_T5_CC1N_T2             TIM1->CCER=((TIM1->CCER & 0x2AAA) | ((0x1<<8) | (0x1<<2)))
#define SET_CC3_T5_CC2N_T4             TIM1->CCER=((TIM1->CCER & 0x2AAA) | ((0x1<<8) | (0x1<<6)))


#define SET_CC1_T1          		   TIM1->CCER|=(0x1<<0)
#define RESET_CC1_T1          		   TIM1->CCER&=(~(0x1<<0))
#define SET_CC1N_T2           		   TIM1->CCER|=(0x1<<2)
#define RESET_CC1N_T2          		   TIM1->CCER&=(~(0x1<<2))

#define SET_CC2_T3          		   TIM1->CCER|=(0x1<<4)
#define RESET_CC2_T3          		   TIM1->CCER&=(~(0x1<<4))
#define SET_CC2N_T4           		   TIM1->CCER|=(0x1<<6)
#define RESET_CC2N_T4          		   TIM1->CCER&=(~(0x1<<6))

#define SET_CC3_T5          		   TIM1->CCER|=(0x1<<8)
#define RESET_CC3_T5          		   TIM1->CCER&=(~(0x1<<8))
#define SET_CC3N_T6          		   TIM1->CCER|=(0x1<<10)
#define RESET_CC3N_T6          		   TIM1->CCER&=(~(0x1<<10))


#define RESET_ALL_CCx				  TIM1->CCER=((TIM1->CCER & 0x2AAA))

#define SET_CC1_POLARITY               TIM1->CCER|= (0x1<<1)
#define SET_CC1N_POLARITY              TIM1->CCER|= (0x1<<3)
#define SET_CC2_POLARITY               TIM1->CCER|= (0x1<<5)
#define SET_CC2N_POLARITY              TIM1->CCER|= (0x1<<7)
#define SET_CC3_POLARITY               TIM1->CCER|= (0x1<<9)
#define SET_CC3N_POLARITY              TIM1->CCER|= (0x1<<11)

#define RESET_CC1_POLARITY               TIM1->CCER&= (~(0x1<<1))
#define RESET_CC1N_POLARITY              TIM1->CCER&= (~(0x1<<3))
#define RESET_CC2_POLARITY               TIM1->CCER&= (~(0x1<<5))
#define RESET_CC2N_POLARITY              TIM1->CCER&= (~(0x1<<7))
#define RESET_CC3_POLARITY               TIM1->CCER&= (~(0x1<<9))
#define RESET_CC3N_POLARITY              TIM1->CCER&= (~(0x1<<11))


#define SYSCLK_FREQ      170000000uL
#define TIM_CLOCK_DIVIDER  1
#define ADV_TIM_CLK_MHz  170

#define SW_DEADTIME_NS                   800
#define DEADTIME_NS  SW_DEADTIME_NS

#define DEAD_TIME_ADV_TIM_CLK_MHz (ADV_TIM_CLK_MHz * TIM_CLOCK_DIVIDER)
#define DEAD_TIME_COUNTS_1  (DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/1000uL)

#if (DEAD_TIME_COUNTS_1 <= 255)
#define DEAD_TIME_COUNTS (uint16_t) DEAD_TIME_COUNTS_1
#elif (DEAD_TIME_COUNTS_1 <= 508)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/2) /1000uL) + 128)
#elif (DEAD_TIME_COUNTS_1 <= 1008)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/8) /1000uL) + 320)
#elif (DEAD_TIME_COUNTS_1 <= 2015)
#define DEAD_TIME_COUNTS (uint16_t)(((DEAD_TIME_ADV_TIM_CLK_MHz * DEADTIME_NS/16) /1000uL) + 384)
#else
#define DEAD_TIME_COUNTS 510
#endif

#endif /* INC_MOJA_KONFIGURACJA_H_ */
