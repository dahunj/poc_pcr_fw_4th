
#include "peltier_ctrl.h"
#include "drv_peltier.h"
#include "util.h"
#include "modbus_hal.h"
#include "optic_qiagen.h"
#include "Glob_Var_IF.h"
#define TRUE 1
#define FALSE 0

#if 0
#define HEAT_SETPOINT		95.0	
#define COOL_SETPOINT 	60.0
#define PRE_COND_SETPOINT	95.0
#define READY_SETPOINT	60.0

#define ROUTINE_CYCLE_MAX 40 //40
#define STABILIZE_TIME 100
#define HEAT_GRADIENT	6.0 ///1.1
#define COOL_GRADIENT (-0.2)
#define TEMP_RESOLUTION 0.25
#endif

#define OFF_THRESHOLD_95 	(g_PRE_COND_SETPOINT + 0.5) 	//94.0
#define ON_THRESHOLD_95 	(g_PRE_COND_SETPOINT - 0.5)	//94.8
#define OFF_THRESHOLD_60 	(g_COOL_SETPOINT + 0.5)		//59.0
#define ON_THRESHOLD_60 	(g_COOL_SETPOINT - 0.5)		//59.8
#define OFF_ROUTINETHRESHOLD_60 	(g_COOL_SETPOINT - 0.5)
#define ON_ROUTINETHRESHOLD_60 		(g_COOL_SETPOINT + 0.5)

#define OSC_NONE		0
#define OSC_UP		1
#define OSC_DOWN		2

#define TEMP_DATA_MAX	1000  /* TEST DATA 1010 */

#define TIME_DEV	0.2 /* 200ms */

#define HEATING_ROUTINE	1
#define COOLING_ROUTINE	2

#define SAMPLE_RATE		10		/* Add 1111 */
#define AVG_CNT			5		/* Add 1111 */

#define SET_STABILIZE_TIME 	1
#define SET_KEEP_TIME		2

#define PRECOND_60	1
#define PRECOND_95	2
#define CYCLE_60		3	
#define CYCLE_95		4
#define HEAT_CUTOFF 120.0f
extern uint8_t 	overshoot_flag;
extern uint32_t	overshoot_cnt;

uint8_t  g_temp_buf_idx = 0;	/* Add 1111 */
float g_temp_flt_avg_prev;	/* Add 1111 */
float g_delta_t_prev;		/* Add 1111 */
 
uint8_t  g_temp_buf_idx2 = 0;	/* Add 1111 */
float g_temp_flt_avg_prev2;	/* Add 1111 */
float g_delta_t_prev2;		/* Add 1111 */
 
uint32_t pelt_ms_cnt = 0; 	/* Add 1024 */
uint8_t pelt_timer_flag = 0;	/* Add 1024 */
uint32_t pelt_ms_cnt_rult = 0; 	/* Add 1024 */

uint8_t g_heat_state 	= OFF;
uint8_t g_cool_state 	= OFF;

uint8_t g_heat_traject_flag 	= FALSE;
uint8_t g_cool_traject_flag 	= FALSE;
uint8_t g_cool_traject_flag2 	= FALSE;

uint8_t g_rountine_cnt = 1;
uint8_t g_rountine_cnt_prev = 0;
uint8_t g_optic1_tube_no = 0;
uint8_t g_optic2_tube_no = 0;

uint8_t g_temp_data_flag = FALSE; 	/* TEST DATA 1010 */
//uint8_t g_data_start_flag = FALSE;  /* TEST DATA 1010 */


float temp_data[TEMP_DATA_MAX];  	/* TEST DATA 1010*/
float temp_data_water[TEMP_DATA_MAX];  	/* TEST DATA 1101*/
uint16_t temp_idx = 0;				/* TEST DATA 1010*/
uint16_t g_error_cnt = 0; /* Add 1107 */

float g_temp_buf[5] = {0};
float g_temp_buf2[5] = {0};

float g_avg_buf[3] = {0};
float g_avg_buf2[3] = {0};
uint8_t g_avg_idx = 0;
uint8_t g_avg_idx2 = 0;
uint16_t g_print_cnt = 0;
uint8_t g_temp_threshold = 0;

//uint8_t g_on_threshold = 0;
//uint8_t g_off_threshold = 0;
/* 200715 bjk */
float g_on_threshold = 0;
float g_off_threshold = 0;

uint8_t  g_top_flag = FALSE;
float thermo_temp[2];
uint8_t g_overshoot_first_flag = TRUE;
uint8_t g_time_check_flag = FALSE;
uint8_t g_delay_cnt = 0;
uint8_t g_cycle_toggle = 0;

uint32_t keep_time_counter = 0;

uint8_t g_plt_heat_rtn_flag = FALSE;

static uint8_t heating_routine_flag = FALSE;		// for peltier_ctrl_heating_routine
static uint8_t cooling_routine_flag = FALSE;		// for peltier_ctrl_cooling_routine
static uint32_t stabilize_time_counter;		// for peltier_pwm_irq

st_Peltier peltctrl;

uint8_t g_optic1_tube_no;

uint8_t temp_err_flag = FALSE;	/* 210204 add */
uint8_t g_pel_ctrl_heat_flag = FALSE;		/* 210303 test */
uint8_t g_pel_ctrl_cool_flag = FALSE;		/* 210303 test */
extern uint8_t g_pel_temp_check_flag;

void peltier_ctrl_init_variables(void)
{
	g_temp_buf_idx = 0;	/* Add 1111 */
	g_temp_flt_avg_prev = 0;	/* Add 1111 */
	g_delta_t_prev = 0;		/* Add 1111 */

	g_temp_buf_idx2 = 0;	/* Add 1111 */
	g_temp_flt_avg_prev2 = 0;	/* Add 1111 */
	g_delta_t_prev2 = 0;		/* Add 1111 */

	pelt_ms_cnt = 0; 	/* Add 1024 */
	pelt_timer_flag = 0;	/* Add 1024 */
	pelt_ms_cnt_rult = 0; 	/* Add 1024 */

	g_heat_state 	= OFF;
	g_cool_state 	= OFF;

	g_heat_traject_flag 	= FALSE;
	g_cool_traject_flag 	= FALSE;
	g_cool_traject_flag2 	= FALSE;

	g_rountine_cnt = 1;
	g_rountine_cnt_prev = 0;

	g_temp_data_flag = FALSE; 	/* TEST DATA 1010 */


	//temp_data[TEMP_DATA_MAX];  	/* TEST DATA 1010*/
	memset(temp_data, 0, sizeof(temp_data));
	//temp_data_water[TEMP_DATA_MAX];  	/* TEST DATA 1101*/
	memset(temp_data_water, 0, sizeof(temp_data_water));
	temp_idx = 0;				/* TEST DATA 1010*/
	g_error_cnt = 0; /* Add 1107 */

	// g_temp_buf[5] = {0};
	memset(g_temp_buf, 0, sizeof(g_temp_buf));
	// g_temp_buf2[5] = {0};
	memset(g_temp_buf2, 0, sizeof(g_temp_buf2));

	// g_avg_buf[3] = {0};
	memset(g_avg_buf, 0, sizeof(g_avg_buf));
	// g_avg_buf2[3] = {0};
	memset(g_avg_buf2, 0, sizeof(g_avg_buf2));
	g_avg_idx = 0;
	g_avg_idx2 = 0;
	g_print_cnt = 0;
	g_temp_threshold = 0;

	g_on_threshold = 0;
	g_off_threshold = 0;
	g_top_flag = FALSE;
	// thermo_temp[2];
	memset(thermo_temp, 0, sizeof(thermo_temp));
	g_overshoot_first_flag = TRUE;
	g_time_check_flag = FALSE;
	g_delay_cnt = 0;
	g_cycle_toggle = 0;

	g_plt_heat_rtn_flag = FALSE;

	heating_routine_flag = FALSE;
	cooling_routine_flag = FALSE;
	stabilize_time_counter = 0;
	peltier_delayed_reset();
	keep_time_counter = 0;

	peltctrl.pwm.pwm_prev_high = 0;
}

static inline uint16_t check_coolHeatTime(void)
{
	
}

static inline void heat_control(uint8_t input)
{
	if(input == OFF)
	{
		if(g_heat_state == ON)
		{
			drv_peltier_heating(OFF);
			g_heat_state = OFF;
		}
	}
	else
	{
		if( g_heat_state != ON) 
		{
			drv_peltier_heating(ON);
			g_heat_state = ON;
		}
	}
}

static inline void cool_control(uint8_t input)
{
	if(input == OFF)
	{
		if(g_cool_state == ON)
		{
			drv_peltier_cooling(OFF);
			g_cool_state = OFF;
		}
	}
	else
	{
		if(g_cool_state != ON)
		{
			drv_peltier_cooling(ON);
			g_cool_state = ON;
		}
	}
}

static inline void fan_control(uint8_t input)
{
	if(input == OFF)
	{
		if( peltctrl.fan_state == ON)
		{
			drv_peltier_fan_blow(OFF);
			peltctrl.fan_state = OFF;
		}
	}
	else
	{
		if( peltctrl.fan_state == OFF)
		{
			drv_peltier_fan_blow(ON);
			peltctrl.fan_state = ON;
		}
	}
}

//Temp_Pwm_Tbl[100] = {

//uint8_t peltier_ctrl_pre_cond(float setpoint)
uint8_t peltier_ctrl_pre_cond(float setpoint, uint32_t keep_time)
{
//	static float  measure_temp;
//	static float  target_temp;
	uint8_t	ret = FALSE;
	// static uint8_t traject_flag = FALSE;
#if 1 // TEST 1105
	static uint8_t data_start_flag = FALSE;  /* TEST DATA 1010 */
//	uint8_t tmp;				/* TEST DATA 1010 */


	if(data_start_flag == FALSE)  /* TEST DATA 1010 */
	{
		g_temp_data_flag = TRUE;
		data_start_flag = TRUE;
		peltctrl.routine_id = HEATING_ROUTINE; /* Add 1108 */
	}
#endif	

	/* Check Temperature Control Kind *//* Add 1120 */
	if( setpoint < 90)	/* Add 1120 */
	{
		peltctrl.ctrl_kind = PRECOND_60;
	}
	else
	{
		peltctrl.ctrl_kind = PRECOND_95;
	}

	
	if( peltier_ctrl_heating_routine(setpoint, keep_time ) == TRUE)  
	{
		ret = TRUE;
		
		#if 1 // TEST 1105
		data_start_flag = FALSE; /* Add 1105 */
		#endif	
#if 0
		g_temp_data_flag = FALSE; /* TEST DATA 1010 */
		tmp = temp_data[0];	 /* TEST DATA 1010 */
#endif		
	}

	return ret;
}



uint8_t peltier_ctrl_pre_cond_cooling(float setpoint, uint32_t keep_time)
{
//	static float  measure_temp;
//	static float  target_temp;
	uint8_t	ret;
	// static uint8_t traject_flag = FALSE;
	// static uint8_t traject_flag2 = FALSE;

	if( peltier_ctrl_cooling_routine(setpoint, keep_time ) == TRUE)  
	{
		ret = TRUE;
	}
	
	return ret;
}

uint8_t  peltier_ctrl_cycle_routine(void)
{
	// static uint16_t routine_count = 1;
	// static uint8_t	toggle =0;
	uint8_t	ret = FALSE;
	// static uint8_t data_start_flag = FALSE;  /* TEST DATA 1010 */
//	uint8_t tmp;				/* TEST DATA 1010 */
	uint32_t keep_time = 0;
	float setPoint = 0;


	if( g_cycle_toggle == 1)	/* Heating routine */	
	{

		peltctrl.ctrl_kind = CYCLE_95;
		
//		setPoint = peltier_delayed_SetPoint(g_HEAT_SETPOINT, 7.0f, SECOND_UNIT*17);
		drv_water_fan(OFF);
		// 고온유지 온도 시간. g_Keeping_time_for_High_Temperature
		if( peltier_ctrl_heating_routine(g_HEAT_SETPOINT,  SECOND_UNIT * g_Keeping_time_for_High_Temperature ) == TRUE)  /* set 10 sec */
		{
			g_cycle_toggle = g_cycle_toggle ?  0 : 1;
			peltier_delayed_reset();
		}
	}
	else 			/* Cooling routine  */
	{
		peltctrl.ctrl_kind = CYCLE_60;
		if((g_rountine_cnt == 1) /*|| (g_rountine_cnt == 2) || (g_rountine_cnt == 3)*/)
		{
			keep_time = SECOND_UNIT * 60;//MINUTE_UNIT * 5;
//			setPoint = peltier_delayed_SetPoint(63, UNDERSHOOT_OFFSET, SECOND_UNIT*30); //14, -8.0
		}
		else if(((g_rountine_cnt >= 5) && (g_rountine_cnt <= 7)) || Optic_Measure_Index_Flag[g_rountine_cnt]==1 || (g_rountine_cnt == MEA_ROUTINE_NO))
		{
			// 광학계동작할 때 유지시간 35초.
			keep_time = SECOND_UNIT * g_Optic_Operation_Keeping_Temp_Sec; // 38; //[43];	//38;	//48;	// 22; //@@@  사이클 내 2nd step 온도 유지시간 세팅값 변수
//			setPoint = peltier_delayed_SetPoint(g_COOL_SETPOINT, UNDERSHOOT_OFFSET, SECOND_UNIT*26);
		}
		else
		{
			// 광학계 동작하지 않을 때는 30초.
			keep_time = SECOND_UNIT * g_Optic_No_Operation_Keeping_Temp_Sec; //30; //[35];	//30;	//40;	// 20; //@@@  사이클 내 2nd step 온도 유지시간 세팅값 변수
//			setPoint = peltier_delayed_SetPoint(g_COOL_SETPOINT, UNDERSHOOT_OFFSET, SECOND_UNIT*26);
		}

		setPoint = g_COOL_SETPOINT;
//		setPoint = peltier_delayed_SetPoint(g_COOL_SETPOINT, 2.0, SECOND_UNIT*17);

		drv_water_fan(ON); /* 210201 test */

		if( peltier_ctrl_cooling_routine(setPoint,  keep_time ) == TRUE)   /* set 20 sec */ //@@@ setPoint, keeptime
		{
			if(g_rountine_cnt  >= g_ROUTINE_CYCLE_MAX)		// end of 45 cycles
			{
				ret = TRUE;
			}
			else
			{
				g_rountine_cnt++;
				ret = FALSE;				
//				printf(" g_rountine_cnt = %u\n", g_rountine_cnt);
			}		

			g_cycle_toggle = g_cycle_toggle ?  0 : 1;	
			peltier_delayed_reset();
			printf("g_rountine_cnt = %u\n", g_rountine_cnt);
		}	
	}


	return ret;
}

static uint8_t pel_smooth_flag = FALSE;		/* 0402 add */

uint8_t peltier_ctrl_heating(float target_temp, float measure_temp)
{
//	float pid_val;
	uint8_t ret = FALSE;
	volatile uint32_t time = get_time_ms_cnt();
	static uint32_t since_time;

	/* If measured temperature reached the move_point */
	if(peltier_judge_reach_setpoint(target_temp, measure_temp, HEAT) == TRUE)
	{
		g_pel_ctrl_heat_flag = FALSE;		/* 210303 test */
		g_pel_temp_check_flag = FALSE;		/* 210303 test */
		ret = TRUE;
	}
	else
	{
		lock();
		g_pel_ctrl_heat_flag = TRUE;		/* 210303 test */
		if( target_temp == g_HEAT_SETPOINT )  		/* 95d */
		{
			if(pel_smooth_flag == FALSE)		/* 0402 test */
			{
				pel_smooth_flag = TRUE;
				since_time = time;
			}

			if((time - since_time) < 200)
			{
				peltctrl.pwm.pwm_period	= 100;
				peltctrl.pwm.pwm_high	= 100;
				peltctrl.pwm.pwm_low	= peltctrl.pwm.pwm_period - peltctrl.pwm.pwm_high;
			}
			else if(((time - since_time) >= 200) && ((time - since_time) < 400))
			{
				peltctrl.pwm.pwm_period	= 100;
				peltctrl.pwm.pwm_high	= 100;
				peltctrl.pwm.pwm_low	= peltctrl.pwm.pwm_period - peltctrl.pwm.pwm_high;
			}
			else if(((time - since_time) >= 400) && ((time - since_time) < 600))
			{
				peltctrl.pwm.pwm_period	= 100;
				peltctrl.pwm.pwm_high	= 100;
				peltctrl.pwm.pwm_low	= peltctrl.pwm.pwm_period - peltctrl.pwm.pwm_high;
			}
			else if(((time - since_time) >= 600) && ((time - since_time) < 800))
			{
				peltctrl.pwm.pwm_period	= 100;
				peltctrl.pwm.pwm_high	= 100;
				peltctrl.pwm.pwm_low	= peltctrl.pwm.pwm_period - peltctrl.pwm.pwm_high;
			}
			else
			{
				peltctrl.pwm.pwm_period	= 100;
				peltctrl.pwm.pwm_high	= 100;
				peltctrl.pwm.pwm_low	= peltctrl.pwm.pwm_period - peltctrl.pwm.pwm_high;
			}
		}
		else if(( target_temp == g_COOL_SETPOINT ))	/* 60d */
		{
			peltctrl.pwm.pwm_period	= 100;
			peltctrl.pwm.pwm_high 		= 100;		// TEST TEST 63
			peltctrl.pwm.pwm_low 		= 0;
		}
		else if( target_temp > g_HEAT_SETPOINT )  		/* 95d */
		{
			if(pel_smooth_flag == FALSE)		/* 0402 test */
			{
				pel_smooth_flag = TRUE;
				since_time = time;
			}

			if((time - since_time) < 200)
			{
				peltctrl.pwm.pwm_period	= 100;
				peltctrl.pwm.pwm_high	= 100;
				peltctrl.pwm.pwm_low	= peltctrl.pwm.pwm_period - peltctrl.pwm.pwm_high;
			}
			else if(((time - since_time) >= 200) && ((time - since_time) < 400))
			{
				peltctrl.pwm.pwm_period	= 100;
				peltctrl.pwm.pwm_high	= 100;
				peltctrl.pwm.pwm_low	= peltctrl.pwm.pwm_period - peltctrl.pwm.pwm_high;
			}
			else if(((time - since_time) >= 400) && ((time - since_time) < 600))
			{
				peltctrl.pwm.pwm_period	= 100;
				peltctrl.pwm.pwm_high	= 100;
				peltctrl.pwm.pwm_low	= peltctrl.pwm.pwm_period - peltctrl.pwm.pwm_high;
			}
			else if(((time - since_time) >= 600) && ((time - since_time) < 800))
			{
				peltctrl.pwm.pwm_period	= 100;
				peltctrl.pwm.pwm_high	= 100;
				peltctrl.pwm.pwm_low	= peltctrl.pwm.pwm_period - peltctrl.pwm.pwm_high;
			}
			else
			{
				peltctrl.pwm.pwm_period	= 100;
				peltctrl.pwm.pwm_high	= 100;
				peltctrl.pwm.pwm_low	= peltctrl.pwm.pwm_period - peltctrl.pwm.pwm_high;
			}
		}
		else if(target_temp >= g_PRE_COND_SETPOINT)    // 2019.12.19
		{
			if(pel_smooth_flag == FALSE)		/* 0402 test */
			{
				pel_smooth_flag = TRUE;
				since_time = time;
			}

			if((time - since_time) < 200)
			{
				peltctrl.pwm.pwm_period	= 100;
				peltctrl.pwm.pwm_high	= 100;
				peltctrl.pwm.pwm_low	= peltctrl.pwm.pwm_period - peltctrl.pwm.pwm_high;
			}
			else if(((time - since_time) >= 200) && ((time - since_time) < 400))
			{
				peltctrl.pwm.pwm_period	= 100;
				peltctrl.pwm.pwm_high	= 100;
				peltctrl.pwm.pwm_low	= peltctrl.pwm.pwm_period - peltctrl.pwm.pwm_high;
			}
			else if(((time - since_time) >= 400) && ((time - since_time) < 600))
			{
				peltctrl.pwm.pwm_period	= 100;
				peltctrl.pwm.pwm_high	= 100;
				peltctrl.pwm.pwm_low	= peltctrl.pwm.pwm_period - peltctrl.pwm.pwm_high;
			}
			else if(((time - since_time) >= 600) && ((time - since_time) < 800))
			{
				peltctrl.pwm.pwm_period	= 100;
				peltctrl.pwm.pwm_high	= 100;
				peltctrl.pwm.pwm_low	= peltctrl.pwm.pwm_period - peltctrl.pwm.pwm_high;
			}
			else
			{
				peltctrl.pwm.pwm_period	= 100;
				peltctrl.pwm.pwm_high	= 100;
				peltctrl.pwm.pwm_low	= peltctrl.pwm.pwm_period - peltctrl.pwm.pwm_high;
			}
		}
		else if( target_temp < g_COOL_SETPOINT )  		/* 95d */
		{
			peltctrl.pwm.pwm_period	= 100;
			peltctrl.pwm.pwm_high 		= 70;
			peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period - peltctrl.pwm.pwm_high;
		}
		else
		{
			peltctrl.pwm.pwm_period	= 100;
			peltctrl.pwm.pwm_high 		= 100;//70;
			peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period - peltctrl.pwm.pwm_high;//30;
		}
		
		output_pwm(HEAT);
		drv_water_fan(OFF); /* 210201 test */

		peltctrl.mea_temp_prev = measure_temp;
		unlock();
	}


	return ret;
}



uint8_t peltier_ctrl_cooling(float target_temp, float measure_temp)	
{
//	float pid_val;
	uint8_t ret = FALSE;
	volatile uint32_t time = get_time_ms_cnt();
	static uint32_t since_time;

	if( peltier_judge_reach_setpoint( target_temp, measure_temp, COOL ) == TRUE)
	{
		g_pel_ctrl_cool_flag = FALSE;		/* 210303 test */
		g_pel_temp_check_flag = FALSE;		/* 210303 test */
		ret = TRUE;
	}
	else
	{
		lock();
		g_pel_ctrl_cool_flag = TRUE;		/* 210303 test */
#if 1
		if(pel_smooth_flag == FALSE)		/* 0402 test */
		{
			pel_smooth_flag = TRUE;
			since_time = time;
		}

		if((time - since_time) < 200)
		{
			peltctrl.pwm.pwm_period	= 100;
			peltctrl.pwm.pwm_high	= 0;
			peltctrl.pwm.pwm_low	= peltctrl.pwm.pwm_period - peltctrl.pwm.pwm_high;
		}
		else if(((time - since_time) >= 200) && ((time - since_time) < 400))
		{
			peltctrl.pwm.pwm_period	= 100;
			peltctrl.pwm.pwm_high	= 0;
			peltctrl.pwm.pwm_low	= peltctrl.pwm.pwm_period - peltctrl.pwm.pwm_high;
		}
		else if(((time - since_time) >= 400) && ((time - since_time) < 600))
		{
			peltctrl.pwm.pwm_period	= 100;
			peltctrl.pwm.pwm_high	= 0;
			peltctrl.pwm.pwm_low	= peltctrl.pwm.pwm_period - peltctrl.pwm.pwm_high;
		}
		else if(((time - since_time) >= 600) && ((time - since_time) < 800))
		{
			peltctrl.pwm.pwm_period	= 100;
			peltctrl.pwm.pwm_high	= 0;
			peltctrl.pwm.pwm_low	= peltctrl.pwm.pwm_period - peltctrl.pwm.pwm_high;
		}
		else
		{
			peltctrl.pwm.pwm_period	= 100;
			peltctrl.pwm.pwm_high	= 0;
			peltctrl.pwm.pwm_low	= peltctrl.pwm.pwm_period - peltctrl.pwm.pwm_high;
		}
#endif
		peltctrl.fan_ctrl_flag	= TRUE;

		output_pwm(COOL);
		drv_water_fan(ON); /* 210201 test */

		peltctrl.mea_temp_prev = measure_temp;
		unlock();
		//printf(" COOLING: peltctrl.fan_ctrl_flag = %u, peltctrl.fan_state = %u\n", peltctrl.fan_ctrl_flag, peltctrl.fan_state);
	}

	return ret;

}



void peltier_set_timer(uint32_t set_time)
{
	switch( peltctrl.ctrl_kind )
	{
		case PRECOND_60:
			peltctrl.stabilize_time_count 	= SECOND_UNIT * 5;		// 10
			break;
			
		case PRECOND_95:
			peltctrl.stabilize_time_count 	= SECOND_UNIT * 5;		// 10
			break;
			
		case CYCLE_60:
			peltctrl.stabilize_time_count 	= SECOND_UNIT * 5;		// 10
			break;
			
		case CYCLE_95:
			peltctrl.stabilize_time_count 	= SECOND_UNIT * 5;		// 10
			break;
			
		default:
			break;
	}
	
	peltctrl.stabilize_time_flag 		= TRUE;
	peltctrl.stabilize_time_expired_flag 	= FALSE;
		
	peltctrl.keep_time_flag 			= FALSE;
	peltctrl.keep_time_expired_flag 	= FALSE;
	peltctrl.keep_time_count 			= set_time;

#if 0
	if(set_kind == SET_STABILIZE_TIME)
	{
		peltctrl.stabilize_time_flag 			= TRUE;
		peltctrl.stabilize_time_expired_flag 		= FALSE;
		peltctrl.stabilize_time_count 			= set_time;
	}
	else if( set_kind == SET_KEEP_TIME)
	{
		peltctrl.keep_time_flag 			= TRUE;
		peltctrl.keep_time_expired_flag 	= FALSE;
		peltctrl.keep_time_count 			= set_time;
	}
	else
	{
		;
	}
#endif	
//	printf(" Time Setted!: peltctrl.keep_time_count  = %lu\n", peltctrl.keep_time_count ); /* Add 1108 */
}

void output_pwm( uint8_t heat_or_cool)
{
	if((peltctrl.heat_or_cool_flag != heat_or_cool) && (peltctrl.heat_or_cool_flag != NONE))
	{
		if( (peltctrl.pwm.pwm_prev_high >= 89) && (peltctrl.pwm.pwm_high >0))
		{
			// printf("pel= %d, %d\n", peltctrl.heat_or_cool_flag, heat_or_cool);
			peltctrl.heat_or_cool_flag 		= heat_or_cool;
			peltctrl.pwm.pwm_time_count 	= 0;
			peltctrl.pwm.pwm_prev_high = peltctrl.pwm.pwm_high; // save pwm high value

			peltctrl.pwm.pwm_on_flag = FALSE;
			return;
		}
	}

	peltctrl.heat_or_cool_flag 		= heat_or_cool;
	peltctrl.pwm.pwm_time_count 	= 0;
	peltctrl.pwm.pwm_prev_high = peltctrl.pwm.pwm_high; // save pwm high value
	
	peltctrl.pwm.pwm_on_flag 	= TRUE;
}



uint8_t peltier_judge_reach_setpoint(float set_point, float curr_point, uint8_t heat_or_cool)
{
	float ratio;
	uint8_t ret = FALSE;
	float percent = 0;


	 if( heat_or_cool == HEAT)
	 {
		ratio = (float)curr_point /(float)set_point;


		if(set_point >= 90)
		{
			percent =  0.95; //0.92
		}
		else /* 60 degree */
		{
			percent = 0.90; //0.85;
		}
		
		if( (ratio >= percent) || (curr_point >= set_point))
		{
			ret = TRUE;
		}
		else
		{
			ret = FALSE;
		}
		
	 }
	 else 		/* COOL */
	 {
	 	if((int)curr_point == 0)
	 	{
	 		//printf("(int)curr_point =  %d\n", (int)curr_point);
			return FALSE;
	 	}
		ratio = (float)set_point /(float)curr_point;
	
		//printf("set_point = %d, curr_point = %d, ratio = %\n", (int)set_point, (int)curr_point, ratio);
		percent = 0.90;//0.936f;// 1.0; //0.99;
		
		if( (ratio >= percent) || (curr_point <= set_point))
		{
			ret = TRUE;
			//printf("reach_setpoint = TRUE\n");
		}
		else
		{
			ret = FALSE;
		}
	 }
	
	return ret;
}



float peltier_get_tempearture(void)	
{
	float temp_flt;
	uint8_t buf[4] = {0};

	drv_mcp9600_read_multiple(ADDR_STATUS_REG, 1, buf);

	if( (buf[0] & UPDATE_MASK) == UPDATE_MASK)
	{
		drv_mcp9600_set_config(ADDR_STATUS_REG, buf[0] &(~UPDATE_MASK)); /* Add 1113 */
		temp_flt = drv_mcp9600_read_temp( ADDR_HOT_JUNCTION_TEMP_REG );
		thermo_temp[0] = temp_flt;

		if(buf[0] & (0x10))
		{
			printf("err update\n");
			temp_err_flag = TRUE;	/* 210204 add */
		}
		return temp_flt;
	}
	else
	{
		temp_err_flag = TRUE;	/* 210204 add */
//		printf("err temp1\n");
		return 0;
	}
}

#if 0
void make_trajectory_temp(  float measure_temp, float target_temp)
{
	float temp_diff = 0;
	
	traject_temp.start_temp 		= measure_temp;
	traject_temp.target_temp 	= target_temp;
	
	temp_diff 		= target_temp - measure_temp;

	if( temp_diff >= 0) /* Heating */
	{

		/* TEMP */
		traject_temp.sect_temp[0].temp_section_start = traject_temp.start_temp;
		traject_temp.sect_temp[0].temp_section_end  = (temp_diff * 0.67)/*(2/3)*/ + traject_temp.sect_temp[0].temp_section_start;
		
		traject_temp.sect_temp[1].temp_section_start = traject_temp.sect_temp[0].temp_section_end;
		traject_temp.sect_temp[1].temp_section_end  = (temp_diff *0.22) /* (2/9)*/ + traject_temp.sect_temp[1].temp_section_start;

		traject_temp.sect_temp[2].temp_section_start = traject_temp.sect_temp[1].temp_section_end;
		traject_temp.sect_temp[2].temp_section_end  = traject_temp.target_temp;

#if 1
		if( target_temp <= 60)
		{
			/* PWM */
			traject_temp.sect_temp[0].pwm.pwm_period 	= PELTIER_PWM_PERIOD;
			traject_temp.sect_temp[0].pwm.pwm_high 	= (uint16_t)((traject_temp.sect_temp[0].pwm.pwm_period * 5)/10);
			traject_temp.sect_temp[0].pwm.pwm_low 	= traject_temp.sect_temp[0].pwm.pwm_period -traject_temp.sect_temp[0].pwm.pwm_high; 

			traject_temp.sect_temp[1].pwm.pwm_period 	= PELTIER_PWM_PERIOD;
			traject_temp.sect_temp[1].pwm.pwm_high 	= (uint16_t)((traject_temp.sect_temp[0].pwm.pwm_period * 2)/10);
			traject_temp.sect_temp[1].pwm.pwm_low 	= traject_temp.sect_temp[1].pwm.pwm_period - traject_temp.sect_temp[1].pwm.pwm_high;

			traject_temp.sect_temp[2].pwm.pwm_period 	= PELTIER_PWM_PERIOD;
			traject_temp.sect_temp[2].pwm.pwm_high 	= (uint16_t)((traject_temp.sect_temp[0].pwm.pwm_period * 1)/10);
			traject_temp.sect_temp[2].pwm.pwm_low 	= traject_temp.sect_temp[2].pwm.pwm_period - traject_temp.sect_temp[2].pwm.pwm_high;
		}
		else
		{
			/* PWM */
			traject_temp.sect_temp[0].pwm.pwm_period 	= PELTIER_PWM_PERIOD;
			traject_temp.sect_temp[0].pwm.pwm_high 	= traject_temp.sect_temp[0].pwm.pwm_period/2;
			traject_temp.sect_temp[0].pwm.pwm_low 	= traject_temp.sect_temp[0].pwm.pwm_period -traject_temp.sect_temp[0].pwm.pwm_high; 

			traject_temp.sect_temp[1].pwm.pwm_period 	= PELTIER_PWM_PERIOD;
			traject_temp.sect_temp[1].pwm.pwm_high 	= traject_temp.sect_temp[0].pwm.pwm_high/4;
			traject_temp.sect_temp[1].pwm.pwm_low 	= traject_temp.sect_temp[1].pwm.pwm_period - traject_temp.sect_temp[1].pwm.pwm_high;

			traject_temp.sect_temp[2].pwm.pwm_period 	= PELTIER_PWM_PERIOD;
			traject_temp.sect_temp[2].pwm.pwm_high 	= traject_temp.sect_temp[0].pwm.pwm_high/8;
			traject_temp.sect_temp[2].pwm.pwm_low 	= traject_temp.sect_temp[2].pwm.pwm_period - traject_temp.sect_temp[2].pwm.pwm_high;

		}
#else
		/* PWM */
		traject_temp.sect_temp[0].pwm.pwm_period 	= PELTIER_PWM_PERIOD;
		traject_temp.sect_temp[0].pwm.pwm_high 	= (uint16_t)((traject_temp.sect_temp[0].pwm.pwm_period * 7)/10);
		traject_temp.sect_temp[0].pwm.pwm_low 	= traject_temp.sect_temp[0].pwm.pwm_period -traject_temp.sect_temp[0].pwm.pwm_high; 

		traject_temp.sect_temp[1].pwm.pwm_period 	= PELTIER_PWM_PERIOD;
		traject_temp.sect_temp[1].pwm.pwm_high 	=  (uint16_t)((traject_temp.sect_temp[0].pwm.pwm_high * 5)/10);
		traject_temp.sect_temp[1].pwm.pwm_low 	= traject_temp.sect_temp[1].pwm.pwm_period - traject_temp.sect_temp[1].pwm.pwm_high;

		traject_temp.sect_temp[2].pwm.pwm_period 	= PELTIER_PWM_PERIOD;
		traject_temp.sect_temp[2].pwm.pwm_high 	=  (uint16_t)((traject_temp.sect_temp[0].pwm.pwm_high * 3)/10);
		traject_temp.sect_temp[2].pwm.pwm_low 	= traject_temp.sect_temp[2].pwm.pwm_period - traject_temp.sect_temp[2].pwm.pwm_high;


#endif
	}
	else 		/* Cooling */
	{
		/* TEMP */
		traject_temp.sect_temp[0].temp_section_start = traject_temp.start_temp;
		traject_temp.sect_temp[0].temp_section_end  = (temp_diff * 0.67)/*(2/3)*/ + traject_temp.sect_temp[0].temp_section_start;
		
		traject_temp.sect_temp[1].temp_section_start = traject_temp.sect_temp[0].temp_section_end;
		traject_temp.sect_temp[1].temp_section_end  = (temp_diff *0.22) /* (2/9)*/ + traject_temp.sect_temp[1].temp_section_start;

		traject_temp.sect_temp[2].temp_section_start = traject_temp.sect_temp[1].temp_section_end;
		traject_temp.sect_temp[2].temp_section_end  = traject_temp.target_temp;

#if 0
		/* PWM */
		traject_temp.sect_temp[0].pwm.pwm_period 	= PELTIER_PWM_PERIOD;
		traject_temp.sect_temp[0].pwm.pwm_high 	= traject_temp.sect_temp[0].pwm.pwm_period -10;
		traject_temp.sect_temp[0].pwm.pwm_low 	= traject_temp.sect_temp[0].pwm.pwm_period -traject_temp.sect_temp[0].pwm.pwm_high; 

		traject_temp.sect_temp[1].pwm.pwm_period 	= PELTIER_PWM_PERIOD;
		traject_temp.sect_temp[1].pwm.pwm_high 	= (uint16_t)( (traject_temp.sect_temp[1].pwm.pwm_period * 9)/10);
		traject_temp.sect_temp[1].pwm.pwm_low 	= traject_temp.sect_temp[1].pwm.pwm_period - traject_temp.sect_temp[1].pwm.pwm_high;

		traject_temp.sect_temp[2].pwm.pwm_period 	= PELTIER_PWM_PERIOD;
		traject_temp.sect_temp[2].pwm.pwm_high 	= (uint16_t)((traject_temp.sect_temp[2].pwm.pwm_period * 9)/10);
		traject_temp.sect_temp[2].pwm.pwm_low 	= traject_temp.sect_temp[2].pwm.pwm_period - traject_temp.sect_temp[2].pwm.pwm_high;
#else
		/* PWM */
		traject_temp.sect_temp[0].pwm.pwm_period 	= PELTIER_PWM_PERIOD;
		traject_temp.sect_temp[0].pwm.pwm_high 	= PELTIER_PWM_PERIOD;
		traject_temp.sect_temp[0].pwm.pwm_low 	= traject_temp.sect_temp[0].pwm.pwm_period -traject_temp.sect_temp[0].pwm.pwm_high; 

		traject_temp.sect_temp[1].pwm.pwm_period 	= PELTIER_PWM_PERIOD;
		traject_temp.sect_temp[1].pwm.pwm_high 	= PELTIER_PWM_PERIOD;
		traject_temp.sect_temp[1].pwm.pwm_low 	= traject_temp.sect_temp[1].pwm.pwm_period - traject_temp.sect_temp[1].pwm.pwm_high;

		traject_temp.sect_temp[2].pwm.pwm_period 	= PELTIER_PWM_PERIOD;
		traject_temp.sect_temp[2].pwm.pwm_high 	= PELTIER_PWM_PERIOD;
		traject_temp.sect_temp[2].pwm.pwm_low 	= traject_temp.sect_temp[2].pwm.pwm_period - traject_temp.sect_temp[2].pwm.pwm_high;


#endif
	}

}
#endif

#if 0
uint8_t set_tempsect_pwm(float curr_temp, uint8_t heat_or_cool)
{
	uint8_t i;
	uint8_t sect_idx = 0;
	uint8_t ret = 0;

	if( heat_or_cool == HEAT)
	{
		for(i = 0; i < 3; i++)
		{
			if( (curr_temp >= traject_temp.sect_temp[i].temp_section_start) && (curr_temp <  traject_temp.sect_temp[i].temp_section_end) )
			{
				sect_idx = i;
				break;
			}
		}

		if( (sect_idx >= 0) && ( sect_idx < 3))
		{
			ret = TRUE;
			peltctrl.pwm.pwm_period	= traject_temp.sect_temp[sect_idx].pwm.pwm_period;
			peltctrl.pwm.pwm_high 		= traject_temp.sect_temp[sect_idx].pwm.pwm_high;
			peltctrl.pwm.pwm_low 		= traject_temp.sect_temp[sect_idx].pwm.pwm_low;
		}
		else
		{
			ret = FALSE;
		}
	}
	else 		/* COOL */
	{
		for(i = 0; i < 3; i++)
		{
			if( (curr_temp <= traject_temp.sect_temp[i].temp_section_start) && (curr_temp >  traject_temp.sect_temp[i].temp_section_end) )
			{
				sect_idx = i;
				break;
			}
		}

		if( (sect_idx >= 0) && ( sect_idx < 3))
		{
			ret = TRUE;
			peltctrl.pwm.pwm_period	= traject_temp.sect_temp[sect_idx].pwm.pwm_period;
			peltctrl.pwm.pwm_high 		= traject_temp.sect_temp[sect_idx].pwm.pwm_high;
			peltctrl.pwm.pwm_low 		= traject_temp.sect_temp[sect_idx].pwm.pwm_low;
		}
		else
		{
			ret = FALSE;
		}

#if 1
		if( (sect_idx < 3) && ( heat_or_cool == COOL) )		/* add 0917 */
		{
			peltctrl.fan_ctrl_flag = TRUE;
		}
		else
		{
			peltctrl.fan_ctrl_flag = FALSE;
		}
#endif		
	}

	return ret;
}
#endif

uint8_t peltier_ctrl_maintain_heat(float target_temp, float measure_temp)
{
//	float pid_val;
	uint8_t ret = FALSE;
//	float move_point;
//	static uint16_t cnt = 0;
//	static uint16_t cnt2 = 0;
	float delta_t = 0;
	
	delta_t  = measure_temp - peltctrl.mea_temp_prev;

#if 0
	if( peltctrl.routine_id == COOLING_ROUTINE)
	{
		peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
		peltctrl.pwm.pwm_high 		=  10;//0;
		peltctrl.pwm.pwm_low 		= 90;//100 ;
		peltctrl.fan_ctrl_flag 			= FALSE; //200514
	}
#endif
	if( peltctrl.routine_id == COOLING_ROUTINE)		//200908 test
	{
		if( measure_temp <= (target_temp - 2.0))	/* Under Temperature : Heating  */
		{
			if( delta_t < 0) /* falling */
			{
				peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
				peltctrl.pwm.pwm_high 		=  45;//50;//45;//35;
				peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
				//printf(" maintain_heat:95F:  4\n");
			}
			else /* Rising */
			{
				peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
				peltctrl.pwm.pwm_high 		=  40;//45;//40;//30;
				peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
				//printf(" maintain_heat:95R:  4\n");
			}
		}
		else if( measure_temp <= (target_temp - 1.0))	/* Under Temperature : Heating  */
		{
			if( delta_t < 0) /* falling */
			{
				peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
				peltctrl.pwm.pwm_high 		=  35;//40;//35;//25;
				peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
				//printf(" maintain_heat:95F:  4\n");
			}
			else /* Rising */
			{
				peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
				peltctrl.pwm.pwm_high 		=  30;//35;//30;//20;
				peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
				//printf(" maintain_heat:95R:  4\n");
			}
		}
		else
		{
			peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
			peltctrl.pwm.pwm_high 		=  20;//30;//20;//15;
			peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
		}
		peltctrl.fan_ctrl_flag 			= FALSE; //200514
	}
	else
	{
		
		if( target_temp >= 90) /* 95d */
		{
			if( measure_temp <= (target_temp - 5.0))	/* Under Temperature : Heating  */
			{
				if( delta_t < 0) /* falling */
				{
					peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
					peltctrl.pwm.pwm_high 		=  85;//65;//80;//75;//100;//80;
					peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
				//	printf(" maintain_heat:95F:  5\n");
				}
				else /* Rising */
				{
					peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
					peltctrl.pwm.pwm_high 		=  80;//60;//75;//70;//95;//70;
					peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
				//	printf(" maintain_heat:95R:  5\n");
				}
			}
			else if( measure_temp <= (target_temp - 3.0))	/* Under Temperature : Heating  */
			{
				if( delta_t < 0) /* falling */
				{
					peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
					peltctrl.pwm.pwm_high 		=  75;//45;//70;//90;//80;//70;
					peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
					//printf(" maintain_heat:95F:  4\n");
				}
				else /* Rising */
				{
					peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
					peltctrl.pwm.pwm_high 		=  70;//40;//65;//85;//75;//60;
					peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
					//printf(" maintain_heat:95R:  4\n");
				}
			}
			else if( measure_temp <= (target_temp - 2.0))
			{
				if( delta_t < 0) /* falling */
				{
					peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
					peltctrl.pwm.pwm_high 		= 65;//35;//45;//60;//80;//70;//60;
					peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
					//printf(" maintain_heat:95F:  3\n");
				}
				else
				{
					peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
					peltctrl.pwm.pwm_high 		= 60;//30;//40;//55;//75;//65;//50;
					peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
					//printf(" maintain_heat:95R:  3\n");
				}

			}
			else if( measure_temp <= (target_temp - 1.0))
			{
				if( delta_t < 0) /* falling */
				{
					peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
					peltctrl.pwm.pwm_high 		= 55;//25;//35;//50;//70;//55;//60;//50;
					peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
					//printf(" maintain_heat:95F:  2\n");
				}
				else
				{

					peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
					peltctrl.pwm.pwm_high 		= 50;//20;//30;//45;//65;//50;//55;//40;
					peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
					//printf(" maintain_heat:95R:  2\n");
				}

			}
			else if( measure_temp <= g_off_threshold)
			{
				if( delta_t < 0) /* falling */
				{
					peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
					peltctrl.pwm.pwm_high 		= 40;//45;//15;//25;//35;//40;//40;//30;
					peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
					//printf(" maintain_heat:95F:  1\n");
				}
				else
				{
					peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
					peltctrl.pwm.pwm_high 		= 35;//40;//10;//20;//30;//35;//30;//20;
					peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
					//printf(" maintain_heat:95R:  1\n");
				}

			}
			else
			{
			 	//printf(" maintain_heat:95:  0\n");
			}				
		}
		else 							/* 65d */
		{
			if( measure_temp <= (target_temp - 5.0))	/* Under Temperature : Heating  */
			{
				if( delta_t < 0) /* falling */
				{
					peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
					peltctrl.pwm.pwm_high 		=  50;//60;	//50;
					peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
					//printf(" maintain_heat:60F:  4\n");
				}
				else /* Rising */
				{
					peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
					peltctrl.pwm.pwm_high 		=  45;	//30;
					peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
					//printf(" maintain_heat:60R:  4\n");
				}
			}
			else if(measure_temp <= (target_temp - 3.0))
			{
				if( delta_t < 0) /* falling */
				{
					peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
					peltctrl.pwm.pwm_high 		=  45;//50;	//40;
					peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
					//printf(" maintain_heat:60F:  4\n");
				}
				else /* Rising */
				{
					peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
					peltctrl.pwm.pwm_high 		=  40;	//20;
					peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
					//printf(" maintain_heat:60R:  4\n");
				}
			}
			else if( measure_temp <= (target_temp - 2.0))
			{
				if( delta_t < 0) /* falling */
				{
					peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
					peltctrl.pwm.pwm_high 		= 35;//40;
					peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
					//printf(" maintain_heat:60F:  3\n");
				}
				else
				{
					peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
					peltctrl.pwm.pwm_high 		= 30;	//10;
					peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
					//printf(" maintain_heat:60R:  3\n");
				}

			}
			else if( measure_temp <= (target_temp - 1.0))
			{
				if( delta_t < 0) /* falling */
				{
					peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
					peltctrl.pwm.pwm_high 		= 25;//30;	//20;
					peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
					//printf(" maintain_heat:60F:  2\n");
				}
				else
				{

					peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
					peltctrl.pwm.pwm_high 		= 20;	//10;
					peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
					//printf(" maintain_heat:60R:  2\n");
				}

			}
			else if( measure_temp <= g_off_threshold)
			{
				if( delta_t < 0) /* falling */
				{
					peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
					peltctrl.pwm.pwm_high 		= 15;//20;
					peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
					//printf(" maintain_heat:60F:  1\n");
				}
				else
				{
					peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
					peltctrl.pwm.pwm_high 		= 10;
					peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
					//printf(" maintain_heat:60R:  1\n");
				}

			}			
			else
			{
			 	//printf(" maintain_heat:95:  0\n");
			}	
		}
	}

	/* Cut Off : Over Temperature(120c) */
	if( measure_temp >= HEAT_CUTOFF) 	/* Add 1118 */
	{
		peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
		peltctrl.pwm.pwm_high 		= 0; 
		peltctrl.pwm.pwm_low 		= 100;
	}
	output_pwm(HEAT);
	drv_water_fan(OFF);
	peltctrl.mea_temp_prev = measure_temp;

	return ret;
}




uint8_t peltier_ctrl_maintain_cool(float target_temp, float measure_temp)
{
//	float pid_val;
	uint8_t ret = FALSE;
//	float move_point;
//	static uint16_t cnt = 0;
//	static uint16_t cnt2 = 0;
	float delta_t = 0;
	
	delta_t  = measure_temp - peltctrl.mea_temp_prev;

	//if( (peltctrl.routine_id == HEATING_ROUTINE ) || (peltctrl.routine_id == COOLING_ROUTINE ) )	/* PELTIER OFF : Add 1108 */
	if( peltctrl.routine_id == HEATING_ROUTINE )	
	{
		if(target_temp >= 90)
		{
			peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
			peltctrl.pwm.pwm_high 		= 0;//40;//50;
			peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
			peltctrl.fan_ctrl_flag 			= FALSE;
			//printf(" peltier_ctrl_maintain_cool : pwm=0: COOL\n");
		}
		else
		{
			peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
			peltctrl.pwm.pwm_high 		= 0;
			peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
			peltctrl.fan_ctrl_flag 			= FALSE;
		}

		output_pwm(HEAT);	/* Add test 200908 */
		peltctrl.mea_temp_prev = measure_temp;
	}
	else
	{
		if( measure_temp >= (target_temp + 5.0))	/* Under Temperature : Heating  */
		{
			if( delta_t > 0)  /* Rising */	
			{
				drv_water_fan(ON);	/* 210201 test */
				peltctrl.fan_ctrl_flag 			= TRUE; //200514
				peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
				peltctrl.pwm.pwm_high 		=  0;//80;//50;
				peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
				//printf(" peltier_ctrl_maintain_cool:60R:  5\n");
			}
			else  /* falling */
			{
				drv_water_fan(ON);	/* 210201 test */
				peltctrl.fan_ctrl_flag 			= TRUE; //200514
				peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
				peltctrl.pwm.pwm_high 		=  0;//60;//40;
				peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
				//printf(" peltier_ctrl_maintain_cool:60R:  5\n");
			}
		}
		else if( measure_temp >= (target_temp + 3.0))	/* Under Temperature : Heating  */
		{
			if( delta_t > 0)  /* Rising */
			{
				drv_water_fan(ON);	/* 210201 test */
				peltctrl.fan_ctrl_flag 			= TRUE; //200514
				peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
				peltctrl.pwm.pwm_high 		= 0;//40; //70//40;
				peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
				//printf(" peltier_ctrl_maintain_cool:60R:  4\n");
			}
			else  /* falling */
			{
				drv_water_fan(ON);	/* 210201 test */
				peltctrl.fan_ctrl_flag 			= TRUE; //200514
				peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
				peltctrl.pwm.pwm_high 		= 0;//30; //50;//20;
				peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
				//printf(" peltier_ctrl_maintain_cool:60F:  4\n");
			}
		}
		else if( measure_temp >= (target_temp + 2.0))
		{
			if( delta_t > 0) /* Rising */
			{
				drv_water_fan(OFF);	/* 210201 test */
				peltctrl.fan_ctrl_flag 			= FALSE; //200514
				peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
				peltctrl.pwm.pwm_high 		= 0;//20;//30;//40;
				peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
				//printf(" peltier_ctrl_maintain_cool:60R:  3\n");
			}
			else /* falling */
			{
				drv_water_fan(OFF);	/* 210201 test */
				peltctrl.fan_ctrl_flag 			= FALSE; //200514
				peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
				peltctrl.pwm.pwm_high	= 0;//11;
				peltctrl.pwm.pwm_low	= peltctrl.pwm.pwm_period - peltctrl.pwm.pwm_high;
			}

		}
		else if( measure_temp >= (target_temp + 1.0))
		{
			if( delta_t >0) /* Rising */
			{
				drv_water_fan(OFF);	/* 210201 test */
				peltctrl.fan_ctrl_flag 			= FALSE; //200909
				peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
				peltctrl.pwm.pwm_high	= 0;//11;
				peltctrl.pwm.pwm_low	= peltctrl.pwm.pwm_period - peltctrl.pwm.pwm_high;
			}
			else /* falling */
			{
				drv_water_fan(OFF);	/* 210201 test */
				peltctrl.fan_ctrl_flag 			= FALSE; //200909
				peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
				peltctrl.pwm.pwm_high	= 0;//11;
				peltctrl.pwm.pwm_low	= peltctrl.pwm.pwm_period - peltctrl.pwm.pwm_high;
			}

		}
		else if( measure_temp >= g_on_threshold)
		{
			if( delta_t > 0)  /* Rising */
			{
				drv_water_fan(OFF);	/* 210201 test */
				peltctrl.fan_ctrl_flag 			= FALSE; //200514
				peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
				peltctrl.pwm.pwm_high 		= 0;//40;
				peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
				//printf(" maintain_heat:60R:  1\n");
			}
			else /* falling */
			{
				drv_water_fan(OFF);	/* 210201 test */
				peltctrl.fan_ctrl_flag 			= FALSE; //200514
				peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
				peltctrl.pwm.pwm_high 		= 0;
				peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;
				//printf(" peltier_ctrl_maintain_cool:60F:  1\n");
			}

		}			
		else
		{
			peltctrl.fan_ctrl_flag 			= FALSE; //200514
		 	//printf(" peltier_ctrl_maintain_cool:60:  0\n");
		}

		output_pwm(COOL);	/* Add test 200908 */
//		drv_water_fan(ON);	/* 210201 test */
		peltctrl.mea_temp_prev = measure_temp;
	}


//	output_pwm(COOL);	/* Add 0918 */
//	peltctrl.mea_temp_prev = measure_temp;

	return ret;
}




void peltier_variable_init(void)
{
	peltctrl.fan_ctrl_flag		= FALSE;
	peltctrl.fan_state		= OFF;
	peltctrl.heat_or_cool_flag	= NONE;
	peltctrl.keep_time_count	= 0;
	peltctrl.keep_time_expired_flag	= FALSE;
	peltctrl.keep_time_flag	= 0;
	peltctrl.mea_temp_prev	= 0;
	peltctrl.pid_val			= 0;
	peltctrl.pid_val_prev		= 0;
	peltctrl.pwm.pwm_high	= 0;
	peltctrl.pwm.pwm_high_out_flag	= FALSE;
	peltctrl.pwm.pwm_low	= 0;
	peltctrl.pwm.pwm_low_out_flag	= FALSE;
	peltctrl.pwm.pwm_on_flag		= FALSE;
	peltctrl.pwm.pwm_period		= 0;
	peltctrl.pwm.pwm_time_count		= 0;
	peltctrl.setpoint		= 0;
	peltctrl.stabilize_time_count	= 0;
	peltctrl.stabilize_time_expired_flag	= 0;
	peltctrl.stabilize_time_flag	= FALSE;
	peltctrl.trend				= OSC_NONE;
	peltctrl.ctrl_kind		= PRECOND_60;
	peltctrl.routine_id = HEATING_ROUTINE;

	// peltctrl.routine_id = 0;

	g_heat_state		= OFF;
	g_cool_state		= OFF;

	g_heat_traject_flag	= FALSE;
	g_cool_traject_flag	= FALSE;
	g_cool_traject_flag2	= FALSE;

	/* Add 1111 */
	g_temp_buf_idx 		= 0;
	memset(g_temp_buf, 0, AVG_CNT);
	g_temp_flt_avg_prev  	= 0;
	g_delta_t_prev 			= 0;
	
	g_temp_buf_idx2 		= 0;
	memset(g_temp_buf2, 0, AVG_CNT);
	g_temp_flt_avg_prev2  	= 0;
	g_delta_t_prev2 		= 0;

	g_overshoot_first_flag	= TRUE; 	/* Add 1128 */
	g_time_check_flag		= FALSE;	/* Add 1128 */
	overshoot_flag 		= FALSE;	/* Add 1128 */
	overshoot_cnt			= 0; 		/* Add 1128 */
	g_delay_cnt			= 0;		/* Add 1128 */

	g_plt_heat_rtn_flag		= FALSE;	/* Add 1129 */
	keep_time_counter= 0;
	stabilize_time_counter = 0;

}


uint8_t peltier_ctrl_heating_routine(float setpoint, uint32_t keep_time)
{
	static float  measure_temp = 0;
//	static float measure_temp2 = 0;
//	static float  target_temp;
	uint8_t	ret = FALSE;
	static uint32_t cnt = 0;
//	float mod_setpoint = 0;
	/* 200914 test */
	static uint8_t temp_error_check_flag = FALSE;
	volatile uint32_t time = get_time_ms_cnt();
	static uint32_t since_time;
	
	peltctrl.setpoint = setpoint;
	
	if(heating_routine_flag == FALSE) /* Add 1024 : Time check */
	{
		pelt_timer_flag = TRUE;
		pelt_ms_cnt = 0;
		heating_routine_flag = TRUE;
//		g_plt_heat_rtn_flag = TRUE;
		peltctrl.routine_id = HEATING_ROUTINE; /* Add 1108 */
		//printf("peltctrl.routine_id = HEATING_ROUTINE\n");
	}
	
	if( peltctrl.keep_time_expired_flag  == TRUE)	
	{
		peltier_switch_alloff();
		peltier_variable_init();
		cnt = 0; 		/* Add 1105 */
		heating_routine_flag = FALSE;

		return TRUE;
	}

	measure_temp = peltier_get_tempearture();
	//printf("%d\n", (int)measure_temp);
	/* I2C2 for chamber temperature measure */ /* Add 1029 */
#if 0	
	measure_temp2 = peltier_get_tempearture_2();

	if( measure_temp2 <= 0) /* test Add 1106 */
	{
		measure_temp2 = 0;
		g_error_cnt++;
	}
#endif
	if( (cnt % SAMPLE_RATE) == 0)	/* Add 1101 */
	{
 		//peltier_data( /*TRUE*/g_temp_data_flag, measure_temp, measure_temp2);  /* TEST DATA 1010*/
		// printf("[TEMP] = %d\n", (int)measure_temp);
	}

//  	mod_setpoint =  peltier_overshoot_setpoint( setpoint + 15,  setpoint,  keep_time/3,  5); /* Add 1128 */

//	if( mod_setpoint == 0)
//	{
//		mod_setpoint = setpoint;
//	}
	

	/* 2. Maintain the Setpoint Temperature Routine */
	if( (peltctrl.keep_time_flag	== TRUE)  || (peltctrl.stabilize_time_flag	== TRUE)  )   /* Mod 1120 */
	{	
	
		//printf("heating_routine: setpoint = %d\n", (int)setpoint);

	
		if((setpoint == g_COOL_SETPOINT))			// 60 degree
		{
			g_on_threshold 	= ON_THRESHOLD_60;
			g_off_threshold	= OFF_THRESHOLD_60;
			//printf("1:g_on_threshold =%d,g_off_threshold = %d\n", (int)g_on_threshold, (int)g_off_threshold);
		}
		else if(setpoint == g_PRE_COND_SETPOINT)  	//95 degree
		{
			g_on_threshold 	= ON_THRESHOLD_95;
			g_off_threshold	= OFF_THRESHOLD_95;
			//printf("2:g_on_threshold =%d,g_off_threshold = %d\n", (int)g_on_threshold, (int)g_off_threshold);
		}
		else
		{
			// TODO: need work
			g_on_threshold = setpoint - 0.5;
			g_off_threshold = setpoint + 0.5;
		}


		if( measure_temp > g_off_threshold)
		{
			lock();	
			heat_control(OFF);		// add test 200908
			
//			peltier_ctrl_maintain_cool( mod_setpoint, measure_temp);
			peltier_ctrl_maintain_cool( setpoint, measure_temp);
			unlock();
			//printf("heating_routine:1:( measure_temp > g_off_threshold) =%d, %d\n", (int)measure_temp, (int)g_off_threshold);
		}
		else
		{
			lock();
			cool_control(OFF);

			peltier_ctrl_maintain_heat( setpoint, measure_temp);			
			unlock();
			//peltier_ctrl_maintain_heat( mod_setpoint, measure_temp);
			//printf("heating_routine:2:( measure_temp <= g_off_threshold) =%d, %d\n", (int)measure_temp, (int)g_off_threshold);
		}

	}
	else		
	{
#if 0
		if(temp_error_check_flag == FALSE)
		{
			temp_error_check_flag = TRUE;
			static float pre_measure_temp = measure_temp;
			since_time = time;
		}
		if(temp_error_check_flag == TRUE)
		{
			if(time - since_time >= SECOND_UNIT * 3)
			{
				if(pre_measure_temp >= measure_temp)
				{

				}
			}
		}
#endif

		/* 1. Raise up Temperature Routine */
		if( peltier_ctrl_heating( setpoint, measure_temp) == TRUE)  	
		//if( peltier_ctrl_heating( mod_setpoint, measure_temp) == TRUE)  		
		{
			pel_smooth_reset();		/* 0402 add */
			if(peltctrl.keep_time_expired_flag != TRUE)
			{
				pelt_timer_flag = FALSE;  	/* Add 1024 : Time check */
				pelt_ms_cnt; 			/* Add 1024 : Time check */
#if 0		//delete?
				if( setpoint < 90)	/* Add 1120 */
				{
					peltctrl.ctrl_kind = PRECOND_60;
				}
				else
				{
					peltctrl.ctrl_kind = PRECOND_95;
				}
#endif
				peltier_set_timer(keep_time);  
			}
		}
		 ret = FALSE;
	}
	
	cnt++;
	
	return ret;
}



uint8_t peltier_ctrl_cooling_routine(float setpoint, uint32_t keep_time)
{
	static float  measure_temp = 0;
//	static float measure_temp2 = 0;
//	static float  target_temp;
	uint8_t	ret = FALSE;
	// static uint8_t traject_flag = FALSE;
	static uint32_t cnt = 0;
	
	peltctrl.setpoint = setpoint;

	if(cooling_routine_flag == FALSE) /* Add 1024 : Time check */
	{
		pelt_timer_flag = TRUE;
		pelt_ms_cnt = 0;
		cooling_routine_flag = TRUE;
		peltctrl.routine_id = COOLING_ROUTINE; /* Add 1108 */
		printf("peltctrl.routine_id = COOLING_ROUTINE\n");
	}
		
	if( peltctrl.keep_time_expired_flag  == TRUE)	
	{
		peltier_switch_alloff();
		peltier_variable_init();
		cooling_routine_flag = FALSE;
		cnt = 0; 		/* Add 1105 */
		
		return TRUE;
	}

	measure_temp = peltier_get_tempearture();
#if 0	
	/* I2C2 for chamber temperature measure */ 	/* Add 1029 */
	measure_temp2 = peltier_get_tempearture_2();
	
	if( measure_temp2 <= 0) /* test Add 1106 */
	{
		measure_temp2 = 0;
		g_error_cnt++;
	}
#endif
	if( (cnt % SAMPLE_RATE) == 0)	/* Add 1101 */
	{
		//peltier_data( g_temp_data_flag, measure_temp, measure_temp2 ); /* TEST DATA 1010*/
		// printf("[TEMP] = %d\n", (int)measure_temp);
	}
	
	/* 2. Maintain the Setpoint Temperature Routine */
	if( (peltctrl.keep_time_flag	== TRUE)  || (peltctrl.stabilize_time_flag	== TRUE)  )   /* Mod 1120 */
	{	
#if 1	 // Block 1128
		//  g_Delay_time_Before_Opting_Runing(15)
		/*
		if((peltctrl.keep_time_flag == TRUE) && (keep_time_counter >=(30*SECOND_UNIT)) && (g_rountine_cnt == MEA_ROUTINE_NO))
		{
			peltier_event_gen();
		}
		*/
		if(  (peltctrl.keep_time_flag == TRUE) && (keep_time_counter >=(g_Delay_time_Before_Opting_Runing*SECOND_UNIT))) 		/* Event Generate */ // keep_time_counter >= 28(first) -> 20(optic simultaneous) -> 15 (optic delay)
		{																						// @@@ Keep_time_counter: 형광 광학계가 동작하는 사이클에서 2nd step 유지시작 몇 초 후에 광학계가 동작할 것인지 설정하는 변수
			peltier_event_gen();
		}
		else
		{
			;
		}

#endif		
		if( setpoint == g_COOL_SETPOINT) 			// 60 degree
		{
			g_on_threshold 	= ON_ROUTINETHRESHOLD_60;
			g_off_threshold	= OFF_ROUTINETHRESHOLD_60;
			//printf("peltier_ctrl_cooling_routine:1:g_on_threshold =%d,g_off_threshold = %d\n", (int)g_on_threshold, (int)g_off_threshold);
		}
//		else if (setpoint == READY_SETPOINT)
//		{
//			g_on_threshold 	= ON_THRESHOLD_60;
//			g_off_threshold	= OFF_THRESHOLD_60;
//		}
		else if(setpoint == g_PRE_COND_SETPOINT)  	//95 degree
		{
			g_on_threshold 	= ON_THRESHOLD_95;
			g_off_threshold	= OFF_THRESHOLD_95;
			//printf("peltier_ctrl_cooling_routine:2:g_on_threshold =%d,g_off_threshold = %d\n", (int)g_on_threshold, (int)g_off_threshold);
		}
		else
		{
			g_on_threshold 	= setpoint + 0.5;
			g_off_threshold	= setpoint - 0.5;
		}
		
		if( measure_temp < g_on_threshold)
		{
			lock();
			cool_control(OFF);
			peltier_ctrl_maintain_heat( setpoint, measure_temp);
			unlock();
			//printf("peltier_ctrl_cooling_routine:1:( measure_temp < g_on_threshold) =%d, %d\n", (int)measure_temp, (int)g_on_threshold);
		}
		else
		{
			lock();
			heat_control(OFF);
			peltier_ctrl_maintain_cool( setpoint, measure_temp);
			unlock();

			//printf("peltier_ctrl_cooling_routine:2:( measure_temp >= g_on_threshold) =%d, %d\n", (int)measure_temp, (int)g_on_threshold);
		}
		
		ret = FALSE;
	}
	else
	{	
		if( peltier_ctrl_cooling( setpoint, measure_temp) == TRUE)
		{
			pel_smooth_reset();		/* 0402 add */
			if(peltctrl.keep_time_expired_flag != TRUE)		/* in case the Timer expire after checking this flag */
			{
				pelt_timer_flag = FALSE;  /* Add 1024 : Time check */
				pelt_ms_cnt; 		/* Add 1024 : Time check */
				
				peltctrl.fan_ctrl_flag	= FALSE;	/* Add 1108 */
				
				peltier_set_timer(keep_time);  
			}
		}
		ret = FALSE;
	}
	
	cnt++;
	
	return ret;
}




void peltier_pwm_irq(void)
{
//	static uint32_t keep_time_counter = 0;
	
	/* sustain time check */ 
	if( peltctrl.keep_time_flag == TRUE)
	{
		keep_time_counter++;
	
		if( peltctrl.keep_time_count <= keep_time_counter) 		/* the set-time expired */
		{
			//printf(" Time Expired!: keep_time_counter = %lu\n", keep_time_counter); /* Add 1108 */
			
			peltctrl.keep_time_flag 			= FALSE;
			peltctrl.keep_time_expired_flag 	= TRUE;		/* * */
			peltctrl.pwm.pwm_on_flag 		= FALSE;
			
			drv_peltier_heating(OFF);
			drv_peltier_cooling(OFF);
			drv_peltier_dir(OFF);
			drv_peltier_fan_blow(OFF);  			/* Add 0918 */
			
			g_heat_state 	= OFF;
			g_cool_state 	= OFF;
			peltctrl.fan_state = OFF;
			
			keep_time_counter = 0;
		}
	}

	/* stabilize time check */
	if( peltctrl.stabilize_time_flag == TRUE)
	{
		//peltctrl.stabilize_time_count--;
		stabilize_time_counter++;
		
		if( peltctrl.stabilize_time_count <= stabilize_time_counter)
		{
			peltctrl.stabilize_time_expired_flag 	= TRUE;
			peltctrl.stabilize_time_flag 		= FALSE;
			peltctrl.keep_time_flag 			= TRUE;
			stabilize_time_counter 			= 0;
		}
	}

	/* make peltier switch on/off like PWM wave */
	if( peltctrl.pwm.pwm_on_flag == TRUE)
	{
		peltctrl.pwm.pwm_time_count++;
		
		if( peltctrl.fan_ctrl_flag == ON) /* add 0917 */
		{
			drv_peltier_dir(OFF);	/* 210115 test */
			fan_control(ON);
		}
		else
		{
			drv_peltier_dir(ON);	/* 210115 test */
			fan_control(OFF);
		}

		if( peltctrl.pwm.pwm_time_count <= peltctrl.pwm.pwm_high)
		{
			if( peltctrl.heat_or_cool_flag == HEAT )
			{
				heat_control(ON);
			}
			else
			{
				cool_control(ON);
			}
		}
		else if( peltctrl.pwm.pwm_time_count < peltctrl.pwm.pwm_period)
		{
			if( peltctrl.heat_or_cool_flag == HEAT )
			{
				heat_control(OFF);
			}
			else
			{
				cool_control(OFF);
			}
			peltctrl.pwm.pwm_low_out_flag = TRUE;
		}
		else
		{
			// peltctrl.pwm.pwm_time_count 	= 0;		// TODO: reset from not interrupt routine
			peltctrl.pwm.pwm_high_out_flag 	= FALSE;
			peltctrl.pwm.pwm_low_out_flag 	= FALSE;
		}
	}
}


void peltier_stop(void) 		/* Add 2019.10.07 */
{
	drv_peltier_stop();
	peltier_variable_init();
	drv_water_fan(OFF);	/* 210114 test */
	printf("peltier stop\n");
}

#if 0
/* Add 1028 */
float calc_temp_pid(float Setpoint, float Input )
{
	static float Error = 0;
	static float prevError = 0;
	static float prevInput = 0;
	float PTerm;
	float ITerm;
	float DTerm;
	float dError;
	float dInput; 
	const float Kp;
	const float Ki;
	const float Kd;
	const float dt = 0.1; 	/* 100ms*/
	float temp_output;
	static uint8_t first_time_flag = FALSE;

	if( first_time_flag == FALSE)
	{
		prevInput = Input;
		first_time_flag = TRUE;
	}

	Error 	= 	Setpoint - Input;
	PTerm 	= 	Kp * Error;
	ITerm 	+= 	Ki * Error * dt;
	/*
	dError 	= 	Error -prevError;
	DTerm 	= 	Kd * (dError / dt);
	*/
	dInput 	= 	Input - prevInput;
	DTerm 	= 	-Kd * (dInput / dt);
	
	temp_output 	= 	PTerm + ITerm + DTerm;
	prevInput 		= 	Input;
	
	return temp_output;
}
#endif

/* Block 1029
void convert_temp_to_pwm( float delta_temp_pid )
{
 	pwm_val = search_temp2pwm_tbl(0, 100, delta_temp_pid);
}

uint8_t search_temp2pwm_tbl(uint8_t left_idx, uint8_t right_idx, float val)
{
	float diff1 = 0;
	float diff2 = 0;
	uint8_t mid_idx = 0;
	
	if( left_idx + 1 == right_idx )
	{
		diff1 = abs( val - Temp_Pwm_Tbl[ left_idx ]);
		diff2 = abs( val - Temp_Pwm_Tbl[ right_idx ]);

		if( diff1 < diff2 )
		{
			return left_idx;
		}
		else
		{
			return right_idx;
		}
	}
	else
	{
		mid_idx = (uint8_t)((left_idx + right_idx)/2);

		if( val > mid_idx )
		{
			return search_temp2pwm_tbl( mid_idx,  right_idx,  val);
		}
		else
		{
			return search_temp2pwm_tbl( left_idx,  mid_idx,  val);
		}
	}
}
*/


/* I2C2 for chamber temperature measure */
/* Add 1029 */
float peltier_get_tempearture_2(void)	
{
	float temp_flt;
	uint8_t buf[4] = {0};

	drv_mcp9600_2_read_multiple(ADDR_STATUS_REG, 1, buf);

	if( (buf[0] & UPDATE_MASK) == UPDATE_MASK)
	{
		drv_mcp9600_2_set_config(ADDR_STATUS_REG, buf[0] &(~UPDATE_MASK)); /* Add 1113 */
		temp_flt = drv_mcp9600_2_read_temp( ADDR_HOT_JUNCTION_TEMP_REG );
		thermo_temp[1] = temp_flt;
		return temp_flt;
	}
	else
	{
		temp_err_flag = TRUE;	/* 210204 add */
		printf("err temp2\n");
		return 0;
	}
}

uint16_t peltier_data(uint8_t flag, float f_temp, float f_temp2)
{
	//if( flag == TRUE)
	{
		if(temp_idx < TEMP_DATA_MAX)
		{
			temp_data[ temp_idx ] 		= f_temp;  	/* TEST DATA 1010*/
			temp_data_water[ temp_idx ] 	= f_temp2; 	/* Add 1101 */
			temp_idx++;
		}
		else
		{
			temp_idx--; 							/* TEST 1106 */
		}
	}
	return temp_idx;
}

#if 0
void peltier_print(uint8_t choice)
{
	uint16_t i ; 
	uint16_t temp_uint = 0;
	uint16_t temp_1und = 0;
	uint16_t temp_uint2 = 0;
	uint16_t temp_1und2 = 0;
	

	if( temp_idx > 200)
	{
		g_print_cnt =  200;
	}
	else
	{
		g_print_cnt = temp_idx;
	}
	
	if( (choice == 1) || (choice == 3) )
	{
		printf("\ntemp_data : \n");
		printf("\ntemp_idx =%u : error count = %u, sample rate = %u\n", temp_idx, g_error_cnt, (uint8_t)SAMPLE_RATE);


		for(i = 0;  i < g_print_cnt; i++)
		{
			printf("%d," , (int)temp_data[i]);
		}
		printf("End!: \n");
	}


	if( (choice == 2) || (choice == 3) )
	{	
		printf("\ntemp_data_water : \n");
		printf("\ntemp_idx =%u : error count = %u, sample rate = %u\n", temp_idx, g_error_cnt, (uint8_t)SAMPLE_RATE);

		
		for(i = 0;  i < g_print_cnt; i++)
		{
			printf("%d," , (int)temp_data_water[i]);
		} 
		printf("End!: \n");
	}
}
#endif

#if 0
void peltier_print2(uint8_t choice)
{
	uint16_t i ; 
	uint16_t temp_uint = 0;
	uint16_t temp_1und = 0;
	uint16_t temp_uint2 = 0;
	uint16_t temp_1und2 = 0;
	
	
	if( temp_idx > 200)
	{
		g_print_cnt =  200;
	}
	else
	{
		g_print_cnt = temp_idx;
	}
	
	if( (choice == 1) || (choice == 3) )
	{
		printf("\ntemp_data : \n");
		printf("\ntemp_idx =%u : error count = %u, sample rate = %u\n", temp_idx, g_error_cnt, (uint8_t)SAMPLE_RATE);


		for(i = 0;  i < g_print_cnt; i += 10)
		{
			printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d," , (int)temp_data[i], (int)temp_data[i+1],(int)temp_data[i+2],(int)temp_data[i+3],
				(int)temp_data[i+4],(int)temp_data[i+5],(int)temp_data[i+6],(int)temp_data[i+7],(int)temp_data[i+8],(int)temp_data[i+9]);
		}
		printf("End!: \n");
	}


	if( (choice == 2) || (choice == 3) )
	{	
		printf("\ntemp_data_water : \n");
		printf("\ntemp_idx =%u : error count = %u, sample rate = %u\n", temp_idx, g_error_cnt, (uint8_t)SAMPLE_RATE);

		
		for(i = 0;  i < g_print_cnt; i += 10)
		{
			printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d," , (int)temp_data_water[i], (int)temp_data_water[i+1],(int)temp_data_water[i+2],(int)temp_data_water[i+3],
				(int)temp_data_water[i+4],(int)temp_data_water[i+5],(int)temp_data_water[i+6],(int)temp_data_water[i+7],(int)temp_data_water[i+8],(int)temp_data_water[i+9]);
		}
		printf("End!: \n");
	}
}
#endif

void peltier_data_init(void)
{
	memset(temp_data, 0, temp_idx);
	memset(temp_data_water, 0, temp_idx); 
	
	temp_idx = 0;
}


uint8_t peltier_pwm_test(uint16_t pwm_rate, uint32_t keep_time)   /* Add 1119 */
{
	static uint8_t time_flag = FALSE;
	static uint8_t time_flag2 = FALSE;
	static float  measure_temp = 0;
	static float measure_temp2 = 0;
	static uint32_t cnt = 0;
	uint32_t ms_cnt1 = 0;
	uint32_t ms_cnt2 = 0;
	
	if( time_flag== FALSE)
	{
		peltier_set_timer(keep_time);  
		time_flag = TRUE;
		ms_cnt1 = get_time_ms_cnt();
	}
	
	if( peltctrl.keep_time_expired_flag  == TRUE)	
	{
		peltier_variable_init();

		return TRUE;
	}
	
	measure_temp 	= peltier_get_tempearture();
	measure_temp2 	= peltier_get_tempearture_2();


	if( measure_temp2 <= 0) /* test Add 1106 */
	{
		measure_temp2 = 0;
		g_error_cnt++;
	}

	if( (cnt % SAMPLE_RATE) == 0)	/* Add 1101 */
	{
		
 		peltier_data( /*TRUE*/g_temp_data_flag, measure_temp, measure_temp2);  /* TEST DATA 1010*/
	}
	
	peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
	peltctrl.pwm.pwm_high 		=  pwm_rate;
	peltctrl.pwm.pwm_low 		= peltctrl.pwm.pwm_period -peltctrl.pwm.pwm_high ;

	if( measure_temp >= 125)
	{
		peltctrl.pwm.pwm_period	= PELTIER_PWM_PERIOD;
		peltctrl.pwm.pwm_high 		=  0;
		peltctrl.pwm.pwm_low 		= 100; 
	}

	output_pwm(HEAT);

	cnt++;

	if( time_flag2== FALSE)
	{
		time_flag2 = TRUE;
		ms_cnt2 = get_time_ms_cnt();
		printf(" ms_cnt2 - ms_cnt1 = %u\n", ms_cnt2 - ms_cnt1);
	}	
	return FALSE;
}


void peltier_event_gen(void) // @@@ 증폭단계에서 사이클 수 계산하는 변수. 해당 변수가 5, 6, 7 사이클과 마지막 사이클에 형광 광학계가 동작하게 설정
							 // @@@ Line :1913 의 peltier_event_gen 함수에 광학계 동작 사이클 설정
{

	if( g_rountine_cnt != g_rountine_cnt_prev)
	{
		//printf("peltier_event_gen : if : g_rountine_cnt = %u, g_rountine_cnt_prev = %u\n", g_rountine_cnt, g_rountine_cnt_prev);
	
		if(g_rountine_cnt==3)//if(g_rountine_cnt==5) // @@@ instead of 5, replace below flag arrays.
		{
			set_event( MEASURE_OP_EVENT );
			set_event( ACTION_TRIGGER_EVENT );
			//printf("peltier_event_gen : g_rountine_cnt = 5, MEASURE_OP_EVENT, ACTION_TRIGGER_EVENT\n");
		}
		else if(g_rountine_cnt==6 || g_rountine_cnt==7 || Optic_Measure_Index_Flag[g_rountine_cnt]==1) // 배정국주임수정
		{
			set_event( ACTION_TRIGGER_EVENT );
//			printf("peltier_event_gen : g_rountine_cnt = %u,ACTION_TRIGGER_EVENT\n", g_rountine_cnt);
		}
		else if(g_rountine_cnt==MEA_ROUTINE_NO)
		{
			set_event( ACTION_TRIGGER_EVENT );
//			printf("peltier_event_gen : g_rountine_cnt = %u,ACTION_TRIGGER_EVENT\n", g_rountine_cnt);
		}

		g_rountine_cnt_prev = g_rountine_cnt;
	}
	else
	{
		;//printf("peltier_event_gen : else: g_rountine_cnt = %u, g_rountine_cnt_prev = %u\n", g_rountine_cnt, g_rountine_cnt_prev);
	}
}

 void peltier_switch_alloff(void)
 {
	drv_peltier_heating(OFF);
	drv_peltier_cooling(OFF);
	drv_peltier_dir(OFF);
	drv_peltier_fan_blow(OFF);

	g_heat_state = OFF;
	g_cool_state = OFF;
	peltctrl.fan_state = OFF;
 }

static uint8_t delayed_flag = FALSE;

float peltier_delayed_SetPoint(float targetPoint, float offset, uint32_t delay_time)
{
	volatile uint32_t time = get_time_ms_cnt();
	float temp;
	static uint32_t since_time;

	if(delayed_flag == FALSE)
	{
		delayed_flag = TRUE;
		since_time = time;
	}
	if(targetPoint >= 90)
	{
		if((time - since_time) <= delay_time - ((delay_time/5)*4))
		{
			temp = (targetPoint + offset);
		}
#if 1
		else if(((time - since_time) > delay_time - ((delay_time/5)*4)) && ((time - since_time) <= delay_time - ((delay_time/5)*3)))
		{
			temp = (targetPoint + offset - (offset/5));
		}
		else if(((time - since_time) > delay_time - ((delay_time/5)*3)) && ((time - since_time) <= delay_time - ((delay_time/5)*2)))
		{
			temp = (targetPoint + offset - (offset/5)*2);
		}
		else if(((time - since_time) > delay_time - ((delay_time/5)*2)) && ((time - since_time) <= delay_time - (delay_time/5)))
		{
			temp = (targetPoint + offset - (offset/5)*3);
		}
		else if((time - since_time) <= delay_time)
		{
			temp = (targetPoint + offset - (offset/5)*4);
		}
#endif
		else
		{
			temp = targetPoint;
		}
	}
#if 0
	else if(targetPoint == 65)
	{
		if((time - since_time) <= delay_time)
		{
			temp = targetPoint + offset;
		}
		else
		{
			temp = targetPoint;
		}
	}
#endif
	else
	{
		if(time - since_time <= delay_time)
		{
			temp = targetPoint + offset;
		}
		else
		{
			temp = targetPoint;
		}
	}
	// printf("%d\n", (int)temp);
	return temp;
}

void peltier_delayed_reset(void)
{
	delayed_flag = FALSE;
}

void pel_smooth_reset(void)		/* 0402 add */
{
	pel_smooth_flag = FALSE;
}
#if 0
float peltier_overshoot_setpoint(float over_setpoint, float orig_setpoint, uint32_t set_time, uint16_t set_num)
 {
	static float sp_val_buf[20];
	uint8_t i;
	float ret;
	uint32_t curr_time;
	uint32_t cur_idx;
	float delta_t = 0;
	static uint32_t delta_tm_sec = 0;
	static uint32_t cnt = 0;
	

	if( g_overshoot_first_flag == TRUE)
	{
		g_overshoot_first_flag = FALSE;
			
		printf(" peltier_overshoot_setpoint:\n");

		delta_t = (over_setpoint - orig_setpoint)/(float)set_num;
		delta_tm_sec = set_time/set_num;
		printf("delta_t = %d , delta_tm_sec = %d\n", (int)delta_t, (int)delta_tm_sec);

		sp_val_buf[0] = over_setpoint;
		printf(" sp_val_buf[0] = %d\n", (int)sp_val_buf[0]);
		
		for( i = 1; i <= set_num -1; i++)
		{
			sp_val_buf[i] = sp_val_buf[i -1]  - delta_t;
			printf(" sp_val_buf[%u] = %d\n", i, (int)sp_val_buf[i]);
		}
		sp_val_buf[set_num ] = orig_setpoint;
		printf(" sp_val_buf[%u] = %d\n", set_num , (int)sp_val_buf[set_num ]);
	
	}


	if( g_time_check_flag == FALSE)
	{
		if( (peltctrl.keep_time_flag == TRUE)  || (peltctrl.stabilize_time_flag == TRUE))
		{
			set_overshoot_time();

			g_time_check_flag	= TRUE;
			//printf("set_overshoot_time()\n");
		}
	}
	

	if( g_time_check_flag == TRUE)	
	{
		g_delay_cnt++;

		if(g_delay_cnt  > 20) /* 2sec later */
		{
			curr_time = get_overshoot_time();
			
			cur_idx = (uint8_t)(curr_time/(uint32_t)delta_tm_sec);
			if( cur_idx >= set_num)
			{
				cur_idx = set_num;
			}
			ret = sp_val_buf[ cur_idx ];
			
			//printf(" curr_time = %u, cur_idx = %u, sp_val_buf[ %u ] = %d\n", curr_time, cur_idx,  cur_idx, (int)sp_val_buf[ cur_idx ]);
		}
	}
	else
	{
		ret = 0;
	}
	
	return ret;
 }
#endif

