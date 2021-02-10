/*
 * drv_pwm.h
 *
 *  Created on: 2019. 7. 9.
 *      Author: jk.choi
 */

#ifndef DRV_PWM_H_
#define DRV_PWM_H_

#include "stm32f10x.h"
#include "stm3210b_eval.h"
#include  "stm32f10x_gpio.h"
#include "drv_timer.h"
#include "task.h"
#include "Drv_limit_sensor.h"

#define  	US_10_PERIOD 		10
#define  	US_100_PERIOD 		100
#define 	MSEC_1_PERIOD		1000
#define	MSEC_10_PERIOD		10000



typedef struct  
{
	uint8_t		set_flag;
	uint16_t 		g_pwm_cnt;
	uint16_t 		g_set_pulse_num;
	
}  omPWM_Info;



typedef enum
{
	PWMSPEED_UP = 0,
	PWMSPEED_DOWN = 1,
	PWMSPEED_NORMAL,
	PWMSPEED_FIXED,
}PWMSMOOTHING_PHASE;

typedef struct
{
	uint8_t		set_flag;
	uint16_t 		g_pwm_cnt;
	uint16_t 		g_set_pulse_num;

}  stPWM_Info;

// step motor speed smoothing
typedef struct{
	PWMSMOOTHING_PHASE smoothing_Phase;
	uint16_t step_cnt;

	uint16_t normal_period;
	
	uint16_t up_end_step;
	uint16_t nor_end_step;
	uint16_t down_end_step;

	// information
	int8_t motor_num;
	uint8_t end_flag;
}stSmoothPWM;


void drv_pwm_init(void);
void drv_pwm_set(  uint8_t tim_x, uint8_t channel_no, uint16_t period_len, uint16_t pulse_len, uint8_t pwm_cnt);
void servo_init(void) ;

void drv_pwm_set_pulse_freq(TIM_TypeDef* tim_x, uint16_t channel_x, uint16_t pulse_count, uint16_t pulse_period, uint8_t isFixed);
void drv_pwm_tim3_ch1_init(uint16_t period , uint16_t pulse);

void drv_pwm_tim4_channel_init(uint16_t period , uint16_t pulse);
void drv_pwm_variable_init(void);
void drv_pwm_all_stop(void);
void drv_pwm_Motor_reset(uint16_t tim, uint16_t channel_num);
void drv_pwm_smoothPwm_control(uint16_t max_up_cnt, uint16_t max_down_cnt, uint16_t min_period);

#endif /* DRV_PWM_H_ */
