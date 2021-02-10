/*
 * drv_timer.c
 *
 *  Created on: 2019. 7. 1.
 *      Author: jk.choi
 */
#include "drv_timer.h"
#include "drv_led.h"

#define TRUE 1
#define FALSE 0

volatile int TimeResult;
volatile int TimeSec;
volatile uint8_t TimeState = 0;

uint8_t overshoot_flag = FALSE;
uint32_t	overshoot_cnt = 0;

extern uint32_t ms_cnt;
extern uint32_t task_ms_cnt;
uint32_t init_delay_cnt = 0;

uint8_t led_toggle = 0;	// bjk 191107
extern  uint8_t InitLED_flag;

extern uint32_t pelt_ms_cnt; 	/* Add 1024 */
extern uint8_t pelt_timer_flag;	/* Add 1024 */
extern uint32_t pelt_ms_cnt_rult;    /* Add 1024 */

void drv_timer1_init(void)
{
    TIM_TimeBaseInitTypeDef TIMER_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //TIM1
  	TIM_TimeBaseStructInit(&TIMER_InitStructure);
	
    TIMER_InitStructure.TIM_CounterMode 	= TIM_CounterMode_Up;
    TIMER_InitStructure.TIM_Prescaler 	= 72-1;
    TIMER_InitStructure.TIM_Period 		= 1000-1;  	/* F=72000000/72/1000 = 1ms */
    TIM_TimeBaseInit(TIM1, &TIMER_InitStructure);
		
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    	/* NVIC Configuration */
    	/* Enable the TIM1_IRQn Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel 				= TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority 		= 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd 				= ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void drv_timer2_init(void)
{
   	TIM_TimeBaseInitTypeDef TIMER_InitStructure;
   	NVIC_InitTypeDef NVIC_InitStructure;

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //TIM2
  	TIM_TimeBaseStructInit(&TIMER_InitStructure);
	
   	TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
   	TIMER_InitStructure.TIM_Prescaler = 72-1;
   	TIMER_InitStructure.TIM_Period = 1000-1;
   	TIM_TimeBaseInit(TIM2, &TIMER_InitStructure);
		
   	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    	/* NVIC Configuration */
    	/* Enable the TIM2_IRQn Interrupt */
   	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
   	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
   	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   	NVIC_Init(&NVIC_InitStructure);
}

void drv_timer8_init(void)	/* Add 0919 */
{
   	TIM_TimeBaseInitTypeDef TIMER_InitStructure;
   	NVIC_InitTypeDef NVIC_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE); 
  	TIM_TimeBaseStructInit(&TIMER_InitStructure);
	
   	TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
   	TIMER_InitStructure.TIM_Prescaler = 7200;
   	TIMER_InitStructure.TIM_Period = 10;
   	TIM_TimeBaseInit(TIM8, &TIMER_InitStructure);
		
   	TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);

    	/* NVIC Configuration */
    	/* Enable the TIM8_IRQn Interrupt */
   	NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQn;
   	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
   	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   	NVIC_Init(&NVIC_InitStructure);
}

void drv_timer5_init(void)
{
    TIM_TimeBaseInitTypeDef TIMER_InitStructure;
   	NVIC_InitTypeDef NVIC_InitStructure;

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  	TIM_TimeBaseStructInit(&TIMER_InitStructure);

   	TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
   	TIMER_InitStructure.TIM_Prescaler = 7200;
   	TIMER_InitStructure.TIM_Period = 5000;
   	TIM_TimeBaseInit(TIM5, &TIMER_InitStructure);

   	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

    	/* NVIC Configuration */
    	/* Enable the TIM5_IRQn Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void drv_timer6_init(void)
{
    TIM_TimeBaseInitTypeDef TIMER_InitStructure;
   	NVIC_InitTypeDef NVIC_InitStructure;

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  	TIM_TimeBaseStructInit(&TIMER_InitStructure);

   	TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
   	TIMER_InitStructure.TIM_Prescaler = 72-1;
   	TIMER_InitStructure.TIM_Period = 1000-1;
   	TIM_TimeBaseInit(TIM6, &TIMER_InitStructure);

   	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

    	/* NVIC Configuration */
    	/* Enable the TIM6_IRQn Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		task_ms_cnt++;
		ms_cnt++;	 /* Add 0919 */
		
		if( pelt_timer_flag == TRUE) 	/* Add 1024 : Time check */
		{	
			pelt_ms_cnt++;	 
		}
		if(overshoot_flag == TRUE)	/* Add 1128 */
		{
			overshoot_cnt++;
		}
	}
}


void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		peltier_pwm_irq();
	}
}

void TIM8_UP_IRQHandler(void)
{
//	static uint8_t toggle = 0;
	
	if (TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
		
		peltier_pwm_irq();
	
	}
}
#if 0
void TIM5_IRQHandler(void)		/* 0.5sec timer */
{
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
		if(InitLED_flag == TRUE)
		{
			if(led_toggle == 0)
			{
				GPIOC->BRR = GPIO_Pin_8;
				led_toggle = 1;
			}
			else
			{
				GPIOC->BSRR = GPIO_Pin_8;
				led_toggle = 0;
			}
		}
	}
}
#endif
void TIM6_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
		init_delay_cnt++;
		
		if(overshoot_flag == TRUE)	/* Add 1128 */
		{
			overshoot_cnt++;
		}
	}
}


void timer_enable(TIM_TypeDef* TIMx)
{
	TIM_Cmd(TIMx, ENABLE);
}

void timer_disable(TIM_TypeDef* TIMx)
{
	TIM_Cmd(TIMx, DISABLE);
}

uint32_t init_delay_time(void)
{
	return init_delay_cnt;
}

uint32_t get_time_ms_cnt(void)
{
	return ms_cnt;
}


void	set_overshoot_time(void) 	/* Add 1128 */
{
	overshoot_flag = TRUE;
}

uint32_t get_overshoot_time(void) 	/* Add 1128 */
{
	return overshoot_cnt;
}
