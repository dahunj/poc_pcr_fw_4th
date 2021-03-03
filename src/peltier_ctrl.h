/*
 * peltier_ctrl.h
 *
 *  Created on: 2019. 7. 30.
 *      Author: jk.choi
 */

#ifndef PELTIER_CTRL_H_
#define PELTIER_CTRL_H_


#include "stm32f10x.h"
#include "stm3210b_eval.h"
#include  "stm32f10x_gpio.h"
#include "drv_timer.h"
#include "task.h"

#define NONE 0
#define HEAT 1
#define COOL 2

#define ON 1
#define OFF 0

#define  PELTIER_PWM_PERIOD 100//200 //50

#define TRUE 1
#define FALSE 0

// 변수화. #define HEAT_SETPOINT		96//104//100//[105]//101.5 //102 //107//104//105//(95.0 + 25.0) 		/* 110d C */ //@@@  사이클 내 1st step 동작 함수
// 변수화. #define COOL_SETPOINT 	   60//59.5 //[58]//60//58//57//54//59//63//80  //(60.0 + 15.0)		/* 75d C */
// 변수화. #define PRE_COND_SETPOINT	96//98.5//[102]//99.5//98.5//105//HEAT_SETPOINT
#define READY_SETPOINT	66 // COOL_SETPOINT

#define OVERSHOOT_OFFSET 	2.5f//5.0f //2.5f
#define UNDERSHOOT_OFFSET  -5.0f//-6.0f//-4.0f//-8.5f//-10.0f //-12.0f//-7.0f

// 변수화. #define ROUTINE_CYCLE_MAX 	40		// @@@ 동작할 최종 사이클 설정한 define 값
#define STABILIZE_TIME 			100
#define HEAT_GRADIENT			6.0 ///1.1
#define COOL_GRADIENT 		(-0.2)
#define TEMP_RESOLUTION 		0.25

#define MILI_SEC_CNT_PER_SEC		1000
#define TIME_LOSS_FACTOR_COEFF	0.9
#define SECOND_UNIT	MILI_SEC_CNT_PER_SEC //(uint32_t)(((float)MILI_SEC_CNT_PER_SEC) * TIME_LOSS_FACTOR_COEFF) //@@@  사이클 내 1st step 동작 함수
#define MINUTE_UNIT	(SECOND_UNIT * 60)		// @@@ MINUTE_UNIT: 사이클 전 precondition 구간 유지시간 세팅값

#if 0
static uint8_t 	peltier_timer_flag = 0;
static uint8_t 	peltier_timer_expired_flag = 0;
static uint16_t	peltier_set_time = 0;
#endif


typedef struct
{
	uint8_t   pwm_on_flag;
	uint16_t pwm_period;
	uint16_t pwm_high;
	uint16_t pwm_low;
	uint16_t pwm_time_count;
	uint8_t   pwm_high_out_flag;
	uint8_t   pwm_low_out_flag;

	uint8_t pwm_prev_high;		// previous high duty value	
}st_PWM;


typedef struct
{
	float		setpoint;
	st_PWM 	pwm;
	uint8_t 	heat_or_cool_flag;
	float 		pid_val;
	float 		pid_val_prev;
	uint8_t 	stabilize_time_flag;
	uint8_t 	stabilize_time_expired_flag;
	uint16_t 	stabilize_time_count;
	uint8_t 	keep_time_flag;
	uint8_t 	keep_time_expired_flag;
	uint32_t 	keep_time_count;
	float 		mea_temp_prev;
	uint8_t	fan_ctrl_flag;
	uint8_t	fan_state;
	uint8_t	routine_id; 
	uint8_t	trend;
	uint8_t	ctrl_kind;
	
}st_Peltier;

typedef struct
{
	float temp_section_start;
	float temp_section_end;
	st_PWM 	pwm;

}stSectTempInfo;


typedef struct
{
	float  target_temp;
	float  start_temp;
	stSectTempInfo  sect_temp[3];

}stTrajectTempInfo;

stTrajectTempInfo traject_temp;


void peltier_ctrl_init_variables(void);
//uint8_t peltier_ctrl_pre_cond(void);
//uint8_t peltier_ctrl_pre_cond(float setpoint);
uint8_t peltier_ctrl_pre_cond(float setpoint, uint32_t keep_time);
uint8_t  peltier_ctrl_cycle_routine(void);
uint8_t peltier_ctrl_heating(float target_temp, float measure_temp);
uint8_t peltier_ctrl_cooling(float target_temp, float measure_temp);
void peltier_set_timer(uint32_t set_time);
//void peltier_set_timer( uint8_t set_kind, uint32_t set_time);
#if 0
float PID_calc(float mea_temp);
uint8_t convert_pwm(float f_temp, uint8_t heat_or_cool);
#endif
void output_pwm( uint8_t heat_or_cool);
//float peltier_set_setpoint(uint8_t kind_of_ctrl);
//uint8_t peltier_judge_reach_setpoint(float set_point, float curr_point);
uint8_t peltier_judge_reach_setpoint(float set_point, float curr_point, uint8_t heat_or_cool);
//uint16_t peltier_get_tempearture(void);
float peltier_get_tempearture(void);
void make_trajectory_temp(  float measure_temp, float target_temp);
//uint8_t set_tempsect_pwm(float curr_temp);
uint8_t set_tempsect_pwm(float curr_temp, uint8_t heat_or_cool);
uint8_t peltier_ctrl_maintain_heat(float target_temp, float measure_temp);
//uint8_t peltier_ctrl_pre_cond_cooling(float setpoint);
uint8_t peltier_ctrl_pre_cond_cooling(float setpoint, uint32_t keep_time);
uint8_t peltier_ctrl_maintain_cool(float target_temp, float measure_temp);
uint8_t peltier_ctrl_heating_routine(float setpoint, uint32_t keep_time);
uint8_t peltier_ctrl_cooling_routine(float setpoint, uint32_t keep_time);
//void peltier_data(uint8_t flag, float f_temp);
uint16_t peltier_data(uint8_t flag, float f_temp, float f_temp2);
void peltier_print(uint8_t choice);
void peltier_print2(uint8_t choice);
void peltier_data_init(void);
void peltier_pwm_irq(void);		/* 3/13 ADD */
void peltier_variable_init(void);	/* 3/13 ADD */

/* I2C2 for chamber temperature measure */
/* Add 1029 */
float peltier_get_tempearture_2(void);	
uint8_t peltier_pwm_test(uint16_t pwm_rate, uint32_t keep_time);  
void peltier_event_gen(void);
void peltier_switch_alloff(void);
float peltier_delayed_SetPoint(float targetPoint, float offset, uint32_t delay_time);
void peltier_delayed_reset(void);
float peltier_overshoot_setpoint(float over_setpoint, float orig_setpoint, uint32_t set_time, uint16_t set_num);
void peltier_stop(void);

void pel_smooth_reset(void);	/* 0402 add */

#endif /* PELTIER_CTRL_H_*/
