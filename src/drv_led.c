/*
 * drv_led.c
 *
 *  Created on: 2019. 7. 1.
 *      Author: jk.choi
 */
#include "drv_led.h"
#include "stm3210b_eval.h"
#include "drv_timer.h"


uint8_t InitLED_flag = FALSE;

 
 void drv_RGB_led_init(void)
 {
  	GPIO_InitTypeDef  GPIO_InitStructure;
	/***********************
		LED_R : PD3
		LED_G : PC8
		LED_B : PC9
	***********************/
  
  	/* Enable the GPIO_LED Clock */
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

  	/* Configure the GPIO_LED pin */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure the GPIO_LED pin */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  	GPIO_Init(GPIOD, &GPIO_InitStructure);
 }
 
void drv_led_init_on(void)
{   
	GPIOD->BRR = GPIO_Pin_3;
	GPIOC->BSRR = GPIO_Pin_8;
	GPIOC->BRR = GPIO_Pin_9;

	InitLED_flag = TRUE;
	timer_enable(TIM5);
}

void drv_led_init_off(void)
{   
	GPIOC->BRR = GPIO_Pin_8;

	InitLED_flag = FALSE;
	timer_disable(TIM5);
}

void drv_led_green_on(void)
{   
	GPIOD->BRR = GPIO_Pin_3;
	GPIOC->BSRR = GPIO_Pin_8;
	GPIOC->BRR = GPIO_Pin_9;
}

void drv_led_green_off(void)
{   
	GPIOC->BRR = GPIO_Pin_8;
}

void drv_led_red_on(void)
{   
	GPIOD->BSRR = GPIO_Pin_3;
	GPIOC->BRR = GPIO_Pin_8;
	GPIOC->BRR = GPIO_Pin_9;
}

void drv_led_red_off(void)
{   
	GPIOD->BRR = GPIO_Pin_3;
}
