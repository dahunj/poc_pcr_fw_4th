

#ifndef DRV_TIMER_H_
#define DRV_TIMER_H_

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"
#include "stdio.h"
#include "misc.h"
#include "peltier_ctrl.h"


void drv_timer1_init(void);
void drv_timer2_init(void);
void drv_timer8_init(void);
void drv_timer5_init(void);
void drv_timer6_init(void);

void timer_enable(TIM_TypeDef* TIMx);
void timer_disable(TIM_TypeDef* TIMx);


void TIM2_IRQHandler(void);
void TIM1_UP_IRQHandler(void);
void TIM5_IRQHandler(void);
void TIM6_IRQHandler(void);

uint32_t get_time_ms_cnt(void);
uint32_t init_delay_time(void);
void	set_overshoot_time(void) ;
uint32_t get_overshoot_time(void); 

#endif /* DRV_TIMER_H_ */
