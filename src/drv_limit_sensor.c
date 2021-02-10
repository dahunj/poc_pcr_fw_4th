/*
 * drv_limit_sensor.c
 *
 *  Created on: 2019. 7. 25.
 *      Author: jk.choi
 */

#include  <stm32f10x.h>
#include "drv_limit_sensor.h"

extern uint16_t om_move_count;
extern uint8_t g_chamber_check_flag;
extern uint8_t g_zero_cw_flag;
extern uint8_t g_zero_ccw_flag;
extern uint8_t g_cw_flag;		/* test add 201013 */
extern uint8_t g_ccw_flag;		/* test add 201013 */
extern uint8_t MotorZeroInitDone_flag;


void drv_limit_sensor_irq_cmd(uint8_t cmd)
{
	if(cmd  != DISABLE)
	{

	}
	else
	{

	}

}


void drv_limit_sensor_init(void)
{
	drv_limit_sensor_gpio_init();
	
	drv_limit_sensor_sm1_init();
	drv_limit_sensor_sm2_init();
	drv_limit_sensor_sm3_init();
	drv_limit_sensor_sm4_init();
//	drv_limit_sensor_sm5_init();
	drv_limit_sensor_om_init();
	drv_door_sensor_up_init();
//	drv_tube_sensor_init();
//	drv_basket_sensor_init();
}

void drv_limit_sensor_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/***************************************** 
	Sensor Pin Map
	SM1 Sensor: PD1
	SM2 Sensor: PD0
	SM3 Sensor: PC7
	SM4 Sensor: PD5
	SM5 Sensor: PD6
	OM Sensor: PC15
	Door up Sensor: PC8
	Tube Sensor: PC9		X
	Basket Sensor: PB14		X
	******************************************/

	/* Configure the GPIO ports */
	GPIO_InitStructure.GPIO_Pin 		=  GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin 		=  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IPU;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}


#if 1
void drv_limit_sensor_sm1_init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Connect EXTI */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,   GPIO_PinSource1);

	/* Configure EXTI  to generate an interrupt on falling edge */
	EXTI_InitStructure.EXTI_Line 		= EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode 		= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger 		= EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd 	= ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* 2 bit for pre-emption priority, 2 bits for subpriority */

	NVIC_InitStructure.NVIC_IRQChannel 	= EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 	= 1; 
	NVIC_InitStructure.NVIC_IRQChannelCmd 			= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Clear EXTI Line Pending Bit */
	EXTI_ClearITPendingBit(EXTI_Line1);
	
	/* Enable the Key EXTI line Interrupt */
	NVIC_ClearPendingIRQ(EXTI1_IRQn);
}
#endif


void drv_limit_sensor_sm2_init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Connect EXTI */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,   GPIO_PinSource0); 

	/* Configure EXTI  to generate an interrupt on falling edge */
	EXTI_InitStructure.EXTI_Line 		= EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode 		= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger 		= EXTI_Trigger_Falling; 
	EXTI_InitStructure.EXTI_LineCmd 	= ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* 2 bit for pre-emption priority, 2 bits for subpriority */

	NVIC_InitStructure.NVIC_IRQChannel 	= EXTI0_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 	= 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd 			= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Clear EXTI Line Pending Bit */
	EXTI_ClearITPendingBit(EXTI_Line0);
	
	/* Enable the Key EXTI line Interrupt */
	NVIC_ClearPendingIRQ(EXTI0_IRQn);
}



void drv_limit_sensor_sm3_init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Connect EXTI */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,   GPIO_PinSource7); 

	/* Configure EXTI  to generate an interrupt on falling edge */
	EXTI_InitStructure.EXTI_Line 		= EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode 		= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger 		= EXTI_Trigger_Falling; 
	EXTI_InitStructure.EXTI_LineCmd 	= ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* 2 bit for pre-emption priority, 2 bits for subpriority */

	NVIC_InitStructure.NVIC_IRQChannel 	= EXTI9_5_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 	= 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd 			= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Clear EXTI Line Pending Bit */
	EXTI_ClearITPendingBit(EXTI_Line7);
	
	/* Enable the Key EXTI line Interrupt */
	NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
	
}


void drv_limit_sensor_sm4_init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Connect EXTI */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,   GPIO_PinSource5); 

	/* Configure EXTI  to generate an interrupt on falling edge */
	EXTI_InitStructure.EXTI_Line 		= EXTI_Line5;
	EXTI_InitStructure.EXTI_Mode 		= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger 		= EXTI_Trigger_Falling; 
	EXTI_InitStructure.EXTI_LineCmd 	= ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* 2 bit for pre-emption priority, 2 bits for subpriority */

	NVIC_InitStructure.NVIC_IRQChannel 	= EXTI9_5_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 	= 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd 			= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Clear EXTI Line Pending Bit */
	EXTI_ClearITPendingBit(EXTI_Line5);
	
	/* Enable the Key EXTI line Interrupt */
	NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
}



void drv_limit_sensor_sm5_init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Connect EXTI */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,   GPIO_PinSource6); 

	/* Configure EXTI  to generate an interrupt on falling edge */
	EXTI_InitStructure.EXTI_Line 		= EXTI_Line6;
	EXTI_InitStructure.EXTI_Mode 		= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger 		= EXTI_Trigger_Falling; 
	EXTI_InitStructure.EXTI_LineCmd 	= ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* 2 bit for pre-emption priority, 2 bits for subpriority */

	NVIC_InitStructure.NVIC_IRQChannel 	= EXTI9_5_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 	= 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd 			= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Clear EXTI Line Pending Bit */
	EXTI_ClearITPendingBit(EXTI_Line6);
	
	/* Enable the Key EXTI line Interrupt */
	NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
}


void drv_limit_sensor_om_init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Connect EXTI */
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,   GPIO_PinSource15); 

	/* Configure EXTI  to generate an interrupt on falling edge */
	EXTI_InitStructure.EXTI_Line 		= EXTI_Line15;
	EXTI_InitStructure.EXTI_Mode 		= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger 		= EXTI_Trigger_Falling; 
	EXTI_InitStructure.EXTI_LineCmd 	= ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* 2 bit for pre-emption priority, 2 bits for subpriority */

	NVIC_InitStructure.NVIC_IRQChannel 	= EXTI15_10_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 	= 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd 			= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Clear EXTI Line Pending Bit */
	EXTI_ClearITPendingBit(EXTI_Line15);

	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
}

void drv_limit_sensor_modify_interrupt(uint8_t id_motor, uint8_t rot_dir)
{
//	EXTI_ClearITPendingBit(EXTI_Line0 | EXTI_Line5 | EXTI_Line6 | EXTI_Line7);
	EXTI_ClearITPendingBit(EXTI_Line0 | EXTI_Line1 | EXTI_Line5 | EXTI_Line6 | EXTI_Line7);	/* 20.07.06 */
	
	switch(id_motor)
	{
		case STEP_MOTOR_1:
			/* not available */
			/* 200512 test ready */
			if (rot_dir == 1 )
			{
				SET_BIT(EXTI->IMR, EXTI_Line1);
			}
			else
			{
				CLEAR_BIT(EXTI->IMR, EXTI_Line1);
			}
			break;
			
		case STEP_MOTOR_2:
			if (rot_dir == 1 )
			{
				SET_BIT(EXTI->IMR, EXTI_Line0);
			}
			else
			{
				CLEAR_BIT(EXTI->IMR, EXTI_Line0);
			}
			break;
			
		case STEP_MOTOR_3:
			if (rot_dir == 1 )
			{
				SET_BIT(EXTI->IMR, EXTI_Line7);
			}
			else
			{
				CLEAR_BIT(EXTI->IMR, EXTI_Line7);
			}
			break;	
			
		case STEP_MOTOR_4:
			if (rot_dir == 1 )
			{
				SET_BIT(EXTI->IMR, EXTI_Line5);
			}
			else
			{
				CLEAR_BIT(EXTI->IMR, EXTI_Line5);
			}
			break;
#if 0
		case STEP_MOTOR_5:
			if (rot_dir == 1 )
			{
				SET_BIT(EXTI->IMR, EXTI_Line6);
			}
			else
			{
				CLEAR_BIT(EXTI->IMR, EXTI_Line6);
			}
			break;
#endif
		case ORIENTAL_MOTOR:
			/* not used */
			break;

		default:
			break;
	}
}

void drv_limit_sensor_reset_interrupt(uint8_t id_motor)
{
	switch(id_motor)
	{
		case STEP_MOTOR_1:
			/* not available */
			/* 200512 test ready */
			CLEAR_BIT(EXTI->IMR, EXTI_Line1);
			break;
			
		case STEP_MOTOR_2:
			CLEAR_BIT(EXTI->IMR, EXTI_Line0);
			break;
			
		case STEP_MOTOR_3:
			CLEAR_BIT(EXTI->IMR, EXTI_Line7);
			break;	
			
		case STEP_MOTOR_4:
			CLEAR_BIT(EXTI->IMR, EXTI_Line5);
			break;
#if 0
		case STEP_MOTOR_5:
			CLEAR_BIT(EXTI->IMR, EXTI_Line6);
			break;
#endif
		case ORIENTAL_MOTOR:
			/* not used */
			break;

		default:
			break;
	}

	EXTI_ClearITPendingBit(EXTI_Line0 | EXTI_Line1 | EXTI_Line5 | EXTI_Line6 | EXTI_Line7);
}


void EXTI0_IRQHandler(void)  
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		TIM_CCxCmd(TIM4,  TIM_Channel_2, DISABLE);	// disable pwm output
		TIM_Cmd(TIM4, DISABLE);		// disable pwm timer
		CLEAR_BIT(EXTI->IMR, EXTI_Line0);		// disable io interrupt
		EXTI_ClearITPendingBit(EXTI_Line0);

		drv_pwm_Motor_reset(4,2);	// reset pwm variables			
		lim_sens_stat_sm2 = CONTACTED;
	}
}

void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		TIM_CCxCmd(TIM4,  TIM_Channel_1, DISABLE);
		TIM_Cmd(TIM4, DISABLE);
		CLEAR_BIT(EXTI->IMR, EXTI_Line1);
		EXTI_ClearITPendingBit(EXTI_Line1);

		drv_pwm_Motor_reset(4,1);
		lim_sens_stat_sm1 = CONTACTED;
	}
}


void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line5) != RESET)
	{
		TIM_CCxCmd(TIM4,  TIM_Channel_4, DISABLE);
		TIM_Cmd(TIM4, DISABLE);
		CLEAR_BIT(EXTI->IMR, EXTI_Line5);
		EXTI_ClearITPendingBit(EXTI_Line5);

		drv_pwm_Motor_reset(4,4);
		lim_sens_stat_sm4 = CONTACTED;
	}
	else if(EXTI_GetITStatus(EXTI_Line6) != RESET)
	{
		TIM_CCxCmd(TIM3,  TIM_Channel_2, DISABLE);
		TIM_Cmd(TIM3, DISABLE);
		CLEAR_BIT(EXTI->IMR, EXTI_Line6);
		EXTI_ClearITPendingBit(EXTI_Line6);

		drv_pwm_Motor_reset(3,2);		
		lim_sens_stat_sm5 = CONTACTED;
	}
	else if(EXTI_GetITStatus(EXTI_Line7) != RESET)
	{
		TIM_CCxCmd(TIM4,  TIM_Channel_3, DISABLE);
		TIM_Cmd(TIM4, DISABLE);
		CLEAR_BIT(EXTI->IMR, EXTI_Line7);
		EXTI_ClearITPendingBit(EXTI_Line7);
					
		drv_pwm_Motor_reset(4,3);
		lim_sens_stat_sm3 = CONTACTED;
	}
	else if(EXTI_GetITStatus(EXTI_Line8) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line8);
		drv_inside_LED(OFF);		/* 201008 add */
		door_up_sensor_flag = CLOSE;
	}
	else if(EXTI_GetITStatus(EXTI_Line9) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line9);
		if(g_chamber_check_flag == TRUE)
		{
			TIM_CCxCmd(TIM3, TIM_Channel_1, DISABLE);
			TIM_Cmd(TIM3, DISABLE);
			drv_pwm_Motor_reset(3,1);
			printf("CCW 30 touch\n");
			g_chamber_check_flag = FALSE;
		}
		tube_sensor = CONTACTED;
	}
	else
	{
		;
	}
}


void EXTI15_10_IRQHandler(void)  
{
	if(EXTI_GetITStatus(EXTI_Line15) != RESET)
	{
//		printf("om_move_count = %d\n", om_move_count);
		TIM_CCxCmd(TIM3, TIM_Channel_1, DISABLE);
		TIM_Cmd(TIM3, DISABLE);
		// CLEAR_BIT(EXTI->IMR, EXTI_Line15);  // TODO: consider remove
		EXTI_ClearITPendingBit(EXTI_Line15);

		drv_pwm_Motor_reset(3,1);

		drv_zero_position_tune();		/* add 201013 */

		lim_sens_stat_om = CONTACTED;
//		om_move_count = 0;
	}
	else if(EXTI_GetITStatus(EXTI_Line14) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line14);
		if(g_chamber_check_flag == TRUE)
		{
			TIM_CCxCmd(TIM3, TIM_Channel_1, DISABLE);
			TIM_Cmd(TIM3, DISABLE);
			drv_pwm_Motor_reset(3,1);
			printf("CW 15 touch\n");
			g_chamber_check_flag = FALSE;
		}
		basket_sensor = CONTACTED;
	}
	else
	{
		;
	}
}


	/* 0402 door safety sensor code test */
void drv_door_sensor_up_init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource8);

	EXTI_InitStructure.EXTI_Line		= EXTI_Line8;
	EXTI_InitStructure.EXTI_Mode		= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger		= EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd	= ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel		= EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority	= 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd	= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	EXTI_ClearITPendingBit(EXTI_Line8);

	NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
}

#if 0
void drv_tube_sensor_init(void)		/* add 200922 */
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource9);

	EXTI_InitStructure.EXTI_Line		= EXTI_Line9;
	EXTI_InitStructure.EXTI_Mode		= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger		= EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd	= ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel		= EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority	= 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd	= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	EXTI_ClearITPendingBit(EXTI_Line9);

	NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
}
#endif

#if 0
void drv_basket_sensor_init(void)		/* add 200922 */
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource14);

	EXTI_InitStructure.EXTI_Line		= EXTI_Line14;
	EXTI_InitStructure.EXTI_Mode		= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger		= EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd	= ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel		= EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority	= 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd	= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	EXTI_ClearITPendingBit(EXTI_Line14);

	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
}
#endif

void drv_zero_position_tune(void)		/* add 201013 */
{
	/* add test 200925 zero position tune */
	if((g_zero_cw_flag == TRUE) || (g_cw_flag == TRUE))
	{
		g_zero_cw_flag = FALSE;
		g_cw_flag = FALSE;		/* test add 201013 */
		g_ccw_flag = FALSE;		/* test add 201013 */
		drv_orientalmotor_output_pulse(ORIENTAL_MOTOR, 0, 23, 200);
//		printf("cw test finish\n");
	}
	else if((g_zero_ccw_flag == TRUE) || (g_ccw_flag == TRUE))
	{
		g_zero_ccw_flag = FALSE;
		g_cw_flag = FALSE;		/* test add 201013 */
		g_ccw_flag = FALSE;		/* test add 201013 */
		drv_orientalmotor_output_pulse(ORIENTAL_MOTOR, 1, 23, 200);
//		printf("ccw test finish\n");
	}
	else
	{}
}
