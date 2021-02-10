/*
 * hw_config.c
 *
 *  Created on: 2019. 6. 20.
 *      Author: jk.choi
 */

#include "hw_config.h"

#define NULL (void *)0

extern uint32_t ms_cnt;
extern uint32_t task_ms_cnt;
extern uint32_t init_delay_cnt;

uint8_t MotorZeroInitDone_flag = FALSE;

void hw_init(void)
{
	/* System Clock setting */
	drv_clk_port_init();
	drv_init();
	variable_init();
	/* Block tempr 1126 */
	drv_motor_init();
	init_mod_hardware();
	 
	drv_enable();
}



void drv_init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	drv_timer1_init();
	drv_timer2_init();
//	drv_timer5_init();		/* LOCK. Because unused LED*/
	drv_timer6_init();

//	drv_RGB_led_init();
//	drv_led_init_on();
 
	drv_peltier_init();

	drv_usart2_init();		/* for  optic sensor */
	drv_usart3_init();
	drv_usart1_init();		/* pc to board Serial com */

	I2C1_init();
	drv_mcp9600_config();
		/* I2C2 for chamber temperature measure */
	/* Add 1029 */
	I2C2_init();
	drv_mcp9600_2_config();
}


void drv_motor_init(void)
{
	drv_stepmotor_init();
	timer_enable(TIM6);

	uint32_t start = init_delay_time();

	while(1)
	{
		uint32_t current = init_delay_time();

		if((current - start) > 1000)	// 1000
		{
			break;
		}

	}

	timer_disable(TIM6);

	drv_stepmotor_output_enable(STEP_MOTOR_1, MOTOR_ENABLE );
	drv_stepmotor_output_enable(STEP_MOTOR_2, MOTOR_ENABLE );
	drv_stepmotor_output_enable(STEP_MOTOR_3, MOTOR_ENABLE );

	timer_enable(TIM6);

	start = init_delay_time();

	while(1)
	{
		uint32_t current = init_delay_time();

		if((current - start) > 1000)	// 1000
		{
			break;
		}

	}

	timer_disable(TIM6);

	drv_st_enable();

	drv_limit_sensor_init();


	drv_motor_zero_position();

    MotorZeroInitDone_flag = TRUE;
}



void drv_enable(void)
{
	
	timer_enable(TIM1);
	timer_enable(TIM2);
    
	//timer_enable(TIM3); // di: disabled
	//timer_enable(TIM4); // di: disabled
	
}

void drv_st_enable(void)
{
	drv_stepmotor_output_enable(STEP_MOTOR_1, MOTOR_ENABLE );
	drv_stepmotor_output_enable(STEP_MOTOR_2, MOTOR_ENABLE );
	drv_stepmotor_output_enable(STEP_MOTOR_3, MOTOR_ENABLE );
	drv_stepmotor_output_enable(STEP_MOTOR_4, MOTOR_ENABLE );
//	drv_stepmotor_output_enable(STEP_MOTOR_5, MOTOR_ENABLE );
}

void variable_init(void)
{
	task_ms_cnt = 0;
	ms_cnt = 0;
	init_delay_cnt = 0;

	//modbus_init();
	peltier_variable_init();
	drv_pwm_variable_init();
}

