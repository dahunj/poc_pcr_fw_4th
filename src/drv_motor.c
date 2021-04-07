/*
 * drv_motor.c
 *
 *  Created on: 2019. 7. 16.
 *      Author: jk.choi
 */

#include "drv_motor.h"

#define TRUE 1
#define FALSE 0

//uint32_t		tim6_cnt; //1015
uint8_t door_open_delay_flag = FALSE;	/* 200709 Add */

void delay_motor(uint16_t x);

extern stPWM_Info tim4_pwm_info[4];
extern omPWM_Info tim3_pwm_info[2];
uint8_t g_zero_cw_flag = FALSE;
uint8_t g_zero_ccw_flag = FALSE;

extern uint8_t g_optic_test1_on_flag;
extern uint8_t g_optic_test2_on_flag;

uint8_t self_check_flag = FALSE;

/* 210303 LOG test */
extern uint32_t ext_seq_num;
extern uint32_t mea_seq_num;
extern uint8_t g_op_exe_flag;
extern uint8_t g_op_mea_exe_flag;


void drv_stepmotor_port_init(void)
{
	GPIO_InitTypeDef port;

	/* STEP Motor */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	
	/***************************************** 
	Step Motor Pin Map
	SM1: PWM- PD12, Enable-PE8,		Dir- PE12
	SM2: PWM- PD13, Enable-PE9,		Dir- PE13
	SM3: PWM- PD14, Enable-PE10,	Dir- PE14
	SM4: PWM- PD15,	Enable-PE11,	Dir- PE15
	SM5: PWM- PA7,  Enable-PD7,		Dir- PD11

	Solenoide Motor Pin Map
	Door Lock: PB13
	******************************************/
	GPIO_StructInit(&port);
	port.GPIO_Mode 			= GPIO_Mode_Out_PP;
	port.GPIO_Pin 			= GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	port.GPIO_Speed 		= GPIO_Speed_2MHz;
	GPIO_Init(GPIOE,  &port);

	GPIO_StructInit(&port);
	port.GPIO_Mode 			= GPIO_Mode_Out_PP;
	port.GPIO_Pin 			= GPIO_Pin_7 | GPIO_Pin_11;
	port.GPIO_Speed 		= GPIO_Speed_2MHz;
	GPIO_Init(GPIOD,  &port);

	GPIO_StructInit(&port);
	port.GPIO_Mode 			= GPIO_Mode_Out_PP;
	port.GPIO_Pin 			= GPIO_Pin_13;
	port.GPIO_Speed 		= GPIO_Speed_2MHz;
	GPIO_Init(GPIOB,  &port);
}


void drv_orientalmotor_port_init(void)  //bjk 190730
{
	GPIO_InitTypeDef port;
	
	/***************************************** 
	Oriental Motor Pin Map
	PWM- PA6, Enable-PD8, Dir- PD9
	******************************************/

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

	GPIO_StructInit(&port);
	port.GPIO_Mode 			= GPIO_Mode_Out_PP;
	port.GPIO_Pin 			= GPIO_Pin_8 | GPIO_Pin_9; 
	port.GPIO_Speed 		= GPIO_Speed_2MHz;
	GPIO_Init(GPIOD,  &port);
}


void drv_stepmotor_init(void)
{
		/* period = 1000 = 1ms */
	drv_pwm_tim4_channel_init(20000, 1000); 
	drv_pwm_tim3_ch1_init(20000 , 1000);
	
	drv_stepmotor_port_init();
	drv_orientalmotor_port_init();

	drv_stepmotor_output_enable(STEP_MOTOR_1, MOTOR_DISABLE );
	drv_stepmotor_output_enable(STEP_MOTOR_2, MOTOR_DISABLE );
	drv_stepmotor_output_enable(STEP_MOTOR_3, MOTOR_DISABLE );
	drv_stepmotor_output_enable(STEP_MOTOR_4, MOTOR_DISABLE );
	drv_stepmotor_output_enable(STEP_MOTOR_5, MOTOR_DISABLE );

	drv_orientalmotor_output_enable(ORIENTAL_MOTOR, 1);	
}

void drv_stepmotor_done(uint8_t id_sm)
{
	switch (id_sm)
	{
		case STEP_MOTOR_1:
			drv_pwm_Motor_reset(4,1);
		break;

		case STEP_MOTOR_2:
			drv_pwm_Motor_reset(4,2);
		break;

		case STEP_MOTOR_3:
			drv_pwm_Motor_reset(4,3);
		break;

		case STEP_MOTOR_4:
			drv_pwm_Motor_reset(4,4);
		break;

		case STEP_MOTOR_5:
			drv_pwm_Motor_reset(3,2);
		break;

		default:
		break;
	}
}

void drv_stepmotor_output_pulse(uint8_t id_sm, uint8_t rot_dir, uint16_t step_count, uint16_t  pulse_period, uint32_t config)
{
//	printf("motor> id=%d, step=%d, period=%d, dir=%d\n", id_sm, step_count, pulse_period, rot_dir);
	/* 210303 LOG test */
#if 1
	if((g_op_exe_flag == TRUE) && (g_op_mea_exe_flag == FALSE))
	{
		printf("LOG motor_id=%d, ext_seq_id=%d\n", id_sm, ext_seq_num);
	}
	else if((g_op_exe_flag == TRUE) && (g_op_mea_exe_flag == TRUE))
	{
		printf("LOG motor_id=%d, mea_seq_id=%d\n", id_sm, mea_seq_num);
	}
	else
	{}
#endif

	if((rot_dir != 0) && (drv_stepmotor_check_sensor(id_sm) == TRUE))
	{
		drv_stepmotor_done(id_sm);
		printf("motor> contacted\n");

		return;
	}

	switch(id_sm)
	{
		case STEP_MOTOR_1:
			tim4_pwm_info[0].set_flag = TRUE;
			tim4_pwm_info[0].g_set_pulse_num = step_count;
			drv_stepmotor_output_direction(STEP_MOTOR_1, rot_dir);
			drv_limit_sensor_modify_interrupt(STEP_MOTOR_1, rot_dir);
			drv_pwm_set_pulse_freq(TIM4, 1, step_count, pulse_period, config & CONFIG_PWM_FIXED);
			break;
			
		case STEP_MOTOR_2:
			tim4_pwm_info[1].set_flag = TRUE;
			tim4_pwm_info[1].g_set_pulse_num = step_count;
			drv_stepmotor_output_direction(STEP_MOTOR_2, rot_dir);
			/*when SM move up, sensor port enable*/
			drv_limit_sensor_modify_interrupt(STEP_MOTOR_2, rot_dir);
			drv_pwm_set_pulse_freq(TIM4, 2,  step_count, pulse_period, config & CONFIG_PWM_FIXED);
			break;
			
		case STEP_MOTOR_3:
			tim4_pwm_info[2].set_flag = TRUE;
			tim4_pwm_info[2].g_set_pulse_num = step_count;
			drv_stepmotor_output_direction(STEP_MOTOR_3, rot_dir);
			drv_limit_sensor_modify_interrupt(STEP_MOTOR_3, rot_dir);
			drv_pwm_set_pulse_freq(TIM4, 3,  step_count, pulse_period, config & CONFIG_PWM_FIXED);
			break;	
			
		case STEP_MOTOR_4:
			tim4_pwm_info[3].set_flag = TRUE;
			tim4_pwm_info[3].g_set_pulse_num = step_count;	
			drv_stepmotor_output_direction(STEP_MOTOR_4, rot_dir);
			drv_limit_sensor_modify_interrupt(STEP_MOTOR_4, rot_dir);
			drv_pwm_set_pulse_freq(TIM4, 4,  step_count, pulse_period, config & CONFIG_PWM_FIXED);
			break;

		case STEP_MOTOR_5:
			tim3_pwm_info[1].set_flag = TRUE;
			tim3_pwm_info[1].g_set_pulse_num = step_count;	
			drv_stepmotor_output_direction(STEP_MOTOR_5, rot_dir);
			drv_limit_sensor_modify_interrupt(STEP_MOTOR_5, rot_dir);
			drv_pwm_set_pulse_freq(TIM3, 2,  step_count, pulse_period, config & CONFIG_PWM_FIXED);
			break;
			
		default:
			break;
	}
}

void drv_orientalmotor_output_pulse(uint8_t id_sm, uint8_t rot_dir, uint16_t step_count, uint16_t  pulse_period)  
{
//	printf("orien> id=%d, step=%d, period=%d, dir=%d\n", id_sm, step_count, pulse_period, rot_dir);

	/* 210303 LOG test */
#if 01
	if((g_op_exe_flag == TRUE) && (g_op_mea_exe_flag == FALSE))
	{
		printf("LOG motor_id=%d, ext_seq_id=%d\n", id_sm, ext_seq_num);
	}
	else if((g_op_exe_flag == TRUE) && (g_op_mea_exe_flag == TRUE))
	{
		printf("LOG motor_id=%d, mea_seq_id=%d\n", id_sm, mea_seq_num);
	}
	else
	{}
#endif	

	tim3_pwm_info[0].set_flag = TRUE;
	tim3_pwm_info[0].g_set_pulse_num = step_count;
	drv_orientalmotor_output_direction(ORIENTAL_MOTOR, rot_dir);
	drv_pwm_set_pulse_freq(TIM3, 1, step_count, pulse_period, 0);
}


void drv_stepmotor_output_enable(uint8_t id_sm,  uint8_t val)
{	
	switch(id_sm)
	{
		case STEP_MOTOR_1:
			GPIO_WriteBit( GPIOE, GPIO_Pin_8, val);
			break;
			
		case STEP_MOTOR_2:
			GPIO_WriteBit( GPIOE, GPIO_Pin_9, val);
			break;
			
		case STEP_MOTOR_3:
			GPIO_WriteBit( GPIOE, GPIO_Pin_10, val);
			break;	
			
		case STEP_MOTOR_4:
			GPIO_WriteBit( GPIOE, GPIO_Pin_11, val);
			break;

		case STEP_MOTOR_5:
			GPIO_WriteBit( GPIOD, GPIO_Pin_7, val);
			break;
			
		default:
			break;
	}
	delay_motor(17);	// di
}


void drv_orientalmotor_output_enable(uint8_t id_sm,  uint8_t val) 
{
	GPIO_WriteBit(GPIOD, GPIO_Pin_8, val);
	delay_motor(17);	// di
}

void drv_stepmotor_output_direction(uint8_t id_sm, uint8_t rot_dir)
{
	switch(id_sm)
	{
		case STEP_MOTOR_1:
			GPIO_WriteBit( GPIOE, GPIO_Pin_12, rot_dir);
			break;
			
		case STEP_MOTOR_2:
			GPIO_WriteBit( GPIOE, GPIO_Pin_13, rot_dir);
			break;
			
		case STEP_MOTOR_3:
			GPIO_WriteBit( GPIOE, GPIO_Pin_14, rot_dir);
			break;	
			
		case STEP_MOTOR_4:
			GPIO_WriteBit( GPIOE, GPIO_Pin_15, rot_dir);
			break;

		case STEP_MOTOR_5:
			GPIO_WriteBit( GPIOD, GPIO_Pin_11, rot_dir);
			break;
			
		default:
			break;
	
	}
	delay_motor(17);	// di
}

void drv_orientalmotor_output_direction(uint8_t id_sm, uint8_t rot_dir) 
{
	switch(id_sm)
	{
		case ORIENTAL_MOTOR:
			GPIO_WriteBit(GPIOD, GPIO_Pin_9, rot_dir);
			break;

		default:
			break;
	}
	delay_motor(17);	// di
}

uint8_t drv_stepmotor_check_sensor(uint8_t id_sm)
{
	uint8_t touch = FALSE;

	switch(id_sm)
	{
		case STEP_MOTOR_1:
#if 1
			if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1) == 0)
			{
				touch = TRUE;
			}
#endif
			break;
			
		case STEP_MOTOR_2:
			if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0) == 0)
			{
				touch = TRUE;
			}
			break;
			
		case STEP_MOTOR_3:
			if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7) == 0)
			{
				touch = TRUE;
			}
			break;	
			
		case STEP_MOTOR_4:
			if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_5) == 0)
			{
				touch = TRUE;
			}
			break;

		case STEP_MOTOR_5:
			if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6) == 0)
			{
				touch = TRUE;
			}
			break;
			
		default:
			break;
	}
	
	return touch;
}


void drv_motor_zero_position(void)
{
#if 1
	lim_sens_stat_sm1 = DETATCHED;
	lim_sens_stat_sm2 = DETATCHED;
	lim_sens_stat_sm3 = DETATCHED;
	lim_sens_stat_sm4 = DETATCHED;
//	lim_sens_stat_sm5 = DETATCHED;
	lim_sens_stat_om = DETATCHED;
	basket_sensor = DETATCHED;
	tube_sensor = DETATCHED;

	drv_stepmotor1_zero_position();
	drv_stepmotor2_zero_position();
	drv_stepmotor3_zero_position();
	drv_stepmotor4_zero_position();
//	drv_stepmotor5_zero_position();
	drv_om_serch_zero_position();
	
	drv_pwm_all_stop();
#endif
}

void drv_om_serch_zero_position(void)  
{
	if((GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_15) == 1)) //detatched
	{
		drv_orientalmotor_output_pulse(ORIENTAL_MOTOR, 1, 4600, 600); // counter clock wise
		g_zero_ccw_flag = TRUE;
		while(tim3_pwm_info[0].set_flag ==TRUE)
		{
			if(lim_sens_stat_om == CONTACTED)
			{
//				printf("ccw touch\n");
				if(self_check_flag == TRUE)
				{
					printf("OM OK\n");
				}
				break;
			}
		}

		delay_motor(17);

		if((lim_sens_stat_om != CONTACTED))
		{
			drv_orientalmotor_output_pulse(ORIENTAL_MOTOR, 0, 18000, 600); // clock wise
			g_zero_cw_flag = TRUE;
			while(tim3_pwm_info[0].set_flag ==TRUE)
			{
				if(lim_sens_stat_om == CONTACTED)
				{
//					printf("cw touch\n");
					if(self_check_flag == TRUE)
					{
						printf("OM OK\n");
					}
					break;
				}
			}
		}
	}
}


void drv_stepmotor1_zero_position(void)
{	
	if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_1) == 1)
	{
		drv_stepmotor_output_pulse(STEP_MOTOR_1, 1, 1500, 800, 0);

		while(lim_sens_stat_sm1 != CONTACTED)
		{
			;
		}
	}
	if(self_check_flag == TRUE)
	{
		printf("SM1 OK\n");
	}
}



void drv_stepmotor2_zero_position(void)
{	
	if((GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_0) == 1))	/*sensor detached*/
	{	
		drv_stepmotor_output_pulse(STEP_MOTOR_2, 1, 6000, 800, 0); /* motor up*/

		while(lim_sens_stat_sm2 != CONTACTED)
		{
			;
		}
	}
	if(self_check_flag == TRUE)
	{
		printf("SM2 OK\n");
	}
}


void drv_stepmotor3_zero_position(void)
{	
	if((GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7) == 1))
	{	
		drv_stepmotor_output_pulse(STEP_MOTOR_3, 1, 4000, 800, 0);
		
		while(lim_sens_stat_sm3 != CONTACTED)
		{
			;
		}
	}
	if(self_check_flag == TRUE)
	{
		printf("SM3 OK\n");
	}
}


void drv_stepmotor4_zero_position(void)
{	
	if((GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_5) == 1))
	{
		drv_stepmotor_output_pulse(STEP_MOTOR_4, 1, 6000, 800, 0);

		while(lim_sens_stat_sm4 != CONTACTED)
		{
			;
		}
	}
	if(self_check_flag == TRUE)
	{
		printf("SM4 OK\n");
	}
}


void drv_stepmotor5_zero_position(void)
{	
	if((GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6) == 1))
	{
		drv_stepmotor_output_pulse(STEP_MOTOR_5, 1, 4000, 800, 0);

		while(lim_sens_stat_sm5 != CONTACTED)
		{
			;
		}
	}
}


/* 17 = about 5us */
void delay_motor(uint16_t x)
{
	volatile uint16_t i ;
	for(i = 0; i < x; i++)
	{
		;
	}
}

	/* 200707 add*/
void door_lock_open(uint8_t state)
{
	uint32_t open_time = get_time_ms_cnt();

	if(state == ON)
	{
		GPIO_WriteBit(GPIOB, GPIO_Pin_13, 1);

		while(door_up_sensor_flag == CLOSE)
		{
			door_sensor_check();
#if 1
			uint32_t current_time = get_time_ms_cnt();
			if(current_time - open_time >= SECOND_UNIT * 4)
			{
				printf("If you want door open, click 'Door button'\n");
				break;
			}
#endif
		}
		for(int i = 0; i < 500000; i++)
		{
			;
		}

		GPIO_WriteBit(GPIOB, GPIO_Pin_13, 0);
		for(int i = 0; i < 10000; i++)
		{
			;
		}

		door_sensor_check();
		if(door_up_sensor_flag == OPEN)
		{
			drv_inside_LED(ON);		/* 201008 add */
		}
		else if(door_up_sensor_flag == CLOSE)
		{
			drv_inside_LED(OFF);		/* 201008 add */
		}
		else
		{
			;
		}

	}
	else if(state == OFF)
	{
		GPIO_WriteBit(GPIOB, GPIO_Pin_13, 0);
	}
	else
	{
		;
	}
}


void drv_self_check(void)
{
	self_check_flag = TRUE;
	uint32_t start_time = get_time_ms_cnt();
	uint32_t curr_time = 0;

	drv_stepmotor_output_pulse(STEP_MOTOR_1, 0, 1000, 800, 0);
	while(tim4_pwm_info[0].set_flag == TRUE)
	{
		curr_time = get_time_ms_cnt();
		if(curr_time - start_time >= 6 * SECOND_UNIT)
		{
			printf("SM1 FAULT\n");
			break;
		}

		if(tim4_pwm_info[0].set_flag == FALSE)
		{
			break;
		}
	}
	drv_stepmotor1_zero_position();

	start_time = get_time_ms_cnt();
	drv_stepmotor_output_pulse(STEP_MOTOR_2, 0, 1000, 800, 0);
	while(tim4_pwm_info[1].set_flag == TRUE)
	{
		curr_time = get_time_ms_cnt();
		if(curr_time - start_time >= 6 * SECOND_UNIT)
		{
			printf("SM2 FAULT\n");
			break;
		}

		if(tim4_pwm_info[1].set_flag == FALSE)
		{
			break;
		}
	}
	drv_stepmotor2_zero_position();

	start_time = get_time_ms_cnt();
	drv_stepmotor_output_pulse(STEP_MOTOR_3, 0, 1000, 800, 0);
	while(tim4_pwm_info[2].set_flag == TRUE)
	{
		curr_time = get_time_ms_cnt();
		if(curr_time - start_time >= 6 * SECOND_UNIT)
		{
			printf("SM3 FAULT\n");
			break;
		}

		if(tim4_pwm_info[2].set_flag == FALSE)
		{
			break;
		}
	}
	drv_stepmotor3_zero_position();

	start_time = get_time_ms_cnt();
	drv_stepmotor_output_pulse(STEP_MOTOR_4, 0, 1000, 800, 0);
	while(tim4_pwm_info[3].set_flag == TRUE)
	{
		curr_time = get_time_ms_cnt();
		if(curr_time - start_time >= 6 * SECOND_UNIT)
		{
			printf("SM4 FAULT\n");
			break;
		}
		if(tim4_pwm_info[3].set_flag == FALSE)
		{
			break;
		}
	}
	drv_stepmotor4_zero_position();

	drv_orientalmotor_output_pulse(ORIENTAL_MOTOR, 0, 2000, 600);
	while(tim3_pwm_info[0].set_flag == TRUE)
	{
		if(tim3_pwm_info[0].set_flag == FALSE)
		{
			break;
		}
	}
	drv_om_serch_zero_position();

	optic_qiagen_init();

//	ADC1_peltier_check();
}



