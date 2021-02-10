/*
 * drv_peltier.c
 *
 *  Created on: 2019. 7. 31.
 *      Author: jk.choi
 */
#include "drv_peltier.h"

#define ON 1
#define OFF 0

#define HEAT_PORT1	GPIO_Pin_4 	/* PA4, PA5 */
#define HEAT_PORT2	GPIO_Pin_5
#define COOL_PORT1	GPIO_Pin_5	/* PE5, PE7 */
#define COOL_PORT2	GPIO_Pin_7

#define PWM_PORT	GPIO_Pin_5 	/* PA5 */
#define DIR_PORT	GPIO_Pin_5	/* PE5 */
#define FAN_PORT	GPIO_Pin_7 	/* PE7 */


#define PWM_PORT_GRP	GPIOA
#define DIR_PORT_GRP	GPIOE

#define DEADTIME_CHECK_EN	0



#if DEADTIME_CHECK_EN
static uint32_t deadTimeHeat_since;
static uint32_t deadTimeCool_since;

static void check_heat_deadTime(int32_t heatCool)
{
	uint32_t curr = get_time_ms_cnt();
	uint32_t time = curr - deadTimeHeat_since;

	if( (time <= 10) && (heatCool !=0))
	{
		printf("hea %d\n", time);
	}

	deadTimeHeat_since = get_time_ms_cnt();
 }

 static void check_cool_deadTime(int32_t heatCool)
 {
	uint32_t curr = get_time_ms_cnt();
	uint32_t time = curr - deadTimeCool_since;

	if((time <= 10) && (heatCool !=0))
	{
		printf("coo %d\n", time);
	}

	deadTimeCool_since = get_time_ms_cnt();
 }
 #endif
 
void drv_peltier_init(void)
{
	GPIO_InitTypeDef port;


	GPIO_StructInit(&port);
	port.GPIO_Mode 		= GPIO_Mode_Out_PP;
	port.GPIO_Pin 			= PWM_PORT;
	port.GPIO_Speed 		= GPIO_Speed_2MHz;
	GPIO_Init(PWM_PORT_GRP,  &port);

	GPIO_StructInit(&port);
	port.GPIO_Mode 		= GPIO_Mode_Out_PP;
	port.GPIO_Pin 			= DIR_PORT | FAN_PORT;
	port.GPIO_Speed 		= GPIO_Speed_2MHz;
	GPIO_Init(DIR_PORT_GRP,  &port);

	GPIO_StructInit(&port);
	port.GPIO_Mode		= GPIO_Mode_Out_PP;
	port.GPIO_Pin			= GPIO_Pin_14 | GPIO_Pin_15;
	port.GPIO_Speed			= GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &port);

	GPIO_StructInit(&port);
	port.GPIO_Mode		= GPIO_Mode_Out_PP;
	port.GPIO_Pin			= GPIO_Pin_9;
	port.GPIO_Speed			= GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &port);

	GPIO_WriteBit( PWM_PORT_GRP, PWM_PORT, OFF);
	GPIO_WriteBit( DIR_PORT_GRP, DIR_PORT, OFF);
	GPIO_WriteBit( DIR_PORT_GRP, FAN_PORT, OFF);
	GPIO_WriteBit( GPIOB, GPIO_Pin_14, OFF);
	GPIO_WriteBit( GPIOB, GPIO_Pin_15, OFF);
	GPIO_WriteBit( GPIOC, GPIO_Pin_9, OFF);
	
}



void drv_peltier_heating(uint8_t ctrl_onoff)
{
	if(ctrl_onoff == ON)
	{	
		#if DEADTIME_CHECK_EN
		check_heat_deadTime(1);
		#endif
		/* Cooling Switch OFF */
//		GPIO_WriteBit( DIR_PORT_GRP, DIR_PORT, OFF);
			
		/* Heating Switch ON */
		GPIO_WriteBit( PWM_PORT_GRP, PWM_PORT, ON);
		
		//printf(" drv_peltier_heating: Cool_Port OFF, Heat_Port ON\n");
		
	}
	else
	{
		#if DEADTIME_CHECK_EN
		check_cool_deadTime(0);
		#endif
		/* Cooling Switch OFF(HEAT) */
//		GPIO_WriteBit( DIR_PORT_GRP, DIR_PORT, OFF);

		/* Heating Switch OFF */
		GPIO_WriteBit( PWM_PORT_GRP, PWM_PORT, OFF);
		//printf(" drv_peltier_heating: Heat_Port OFF \n");
	}
}



void drv_peltier_cooling(uint8_t ctrl_onoff)
{
	if(ctrl_onoff == ON)
	{
		#if DEADTIME_CHECK_EN
		check_cool_deadTime(1);
		#endif
		/* Heating Switch OFF */
		GPIO_WriteBit( PWM_PORT_GRP, PWM_PORT, ON);

		/* Cooling Switch ON(COOL) */
//		GPIO_WriteBit( DIR_PORT_GRP, DIR_PORT, ON);
		//printf(" drv_peltier_cooling: Heat_Port OFF, Cool_Port ON\n");
	}
	else
	{
		#if DEADTIME_CHECK_EN
		check_heat_deadTime(0);
		#endif
		/* Cooling Switch OFF */
//		GPIO_WriteBit( DIR_PORT_GRP, DIR_PORT, OFF);
		//printf(" drv_peltier_cooling: Cool_Port OFF\n");

		/* Heating Switch OFF */
		GPIO_WriteBit( PWM_PORT_GRP, PWM_PORT, OFF);
	}
}

void drv_peltier_dir(uint8_t dir_onoff)
{
	if(dir_onoff == ON)
	{
		GPIO_WriteBit( DIR_PORT_GRP, DIR_PORT, ON);
	}
	else
	{
		GPIO_WriteBit( DIR_PORT_GRP, DIR_PORT, OFF);
	}
}



void drv_peltier_fan_blow(uint8_t fan_ctrl)
{
	if( fan_ctrl == ON)
	{
		GPIO_WriteBit( DIR_PORT_GRP, FAN_PORT, ON);
		//printf(" drv_peltier_fan_blow: Fan_Port ON\n");
	}
	else
	{
		GPIO_WriteBit( DIR_PORT_GRP, FAN_PORT, OFF);
		//printf(" drv_peltier_fan_blow: Fan_Port OFF\n");
	}
}

void drv_peltier_stop(void)
{
	GPIO_WriteBit( PWM_PORT_GRP, PWM_PORT, OFF);
	GPIO_WriteBit( DIR_PORT_GRP, DIR_PORT, OFF);
	GPIO_WriteBit( DIR_PORT_GRP, FAN_PORT, OFF);
	//printf(" drv_peltier_stop: All_Port OFF\n");
}

void drv_inside_LED(uint8_t on_off)		/* 201008 White LED (upside_fan -> inside_LED change) */
{
	if(on_off == ON)
	{
		GPIO_WriteBit(GPIOB, GPIO_Pin_14, ON);
	}
	else
	{
		GPIO_WriteBit(GPIOB, GPIO_Pin_14, OFF);
	}
}

void drv_water_fan(uint8_t on_off)
{
	if(on_off == ON)
	{
		GPIO_WriteBit(GPIOB, GPIO_Pin_15, ON);
	}
	else
	{
		GPIO_WriteBit(GPIOB, GPIO_Pin_15, OFF);
	}
}
