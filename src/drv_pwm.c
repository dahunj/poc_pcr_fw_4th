/*
 * drv_pwm.c
 *
 *  Created on: 2019. 7. 9.
 *      Author: jk.choi
 */

#include "drv_pwm.h"

#define SYSCLK 72000000
#define PRESCALER 72


#define TRUE	1
#define FALSE 0

// smooth pwm
#define MAX_UPDOWN_COUNT 		100
#define MIN_SMOOTHING_PERIOD	10000
#define UPDOWN_ARRAY_SIZE		250
#define PERIOD_CONV     1000000			// 1usec
#define float_t float


uint8_t tim3_init_flag = FALSE;
uint8_t tim4_init_flag = FALSE;

uint16_t g_period_tim4 = 0;
uint16_t g_period_tim3 = 0;

uint16_t g_pwm_tim3_cnt = 0;
uint16_t g_pwm_tim4_cnt = 0;
uint16_t g_tim3_set_pulse_num = 0;
uint16_t g_tim4_set_pulse_num = 0;

uint16_t g_curr_tim3_channel = 1;
uint16_t g_curr_tim4_channel = 1;

stPWM_Info tim4_pwm_info[4];
omPWM_Info tim3_pwm_info[2];
static uint16_t smoothUpDownPeriod[2][UPDOWN_ARRAY_SIZE];
static stSmoothPWM smoothPwm;
static struct
{
	uint16_t max_up_cnt;
	uint16_t max_down_cnt;
	uint16_t min_period;
}smoothPwm_control;

extern uint8_t MotorZeroInitDone_flag;

extern int om_final_count;		// bjk 191029
uint16_t om_move_count = 0;		// bjk 191029

extern uint8_t g_chamber_check_flag;	/* test add 200925 */

static void prepare_smoothPwm(stSmoothPWM *s, uint16_t total_step_count, uint16_t target_period, uint8_t isFixed);
static void prepare_linear_smoothPwm(stSmoothPWM *s, uint16_t total_step_count, uint16_t target_period);
static void prepare_fixed_smoothPwm(stSmoothPWM *s, uint16_t total_step_count, uint16_t target_period);
static inline uint16_t get_arr_smoothPwm(stSmoothPWM *s);
static void counting_step_smoothPwm(stSmoothPWM *s);


void drv_pwm_set_pulse_freq(TIM_TypeDef* tim_x, uint16_t channel_x, uint16_t pulse_count, uint16_t pulse_period, uint8_t isFixed)
{
	uint16_t tmp;

	if(!((tim_x == TIM3) && (channel_x == 1)))
	{
		prepare_smoothPwm(&smoothPwm, pulse_count, pulse_period, isFixed);
	}

	if(tim_x == TIM3)
	{
		TIM_SetCounter(TIM3, 0);
		TIM_ClearITPendingBit(TIM3 ,TIM_IT_Update);

		switch(channel_x)
		{
			// ORIENTAL_MOTOR
			case 1:
				tim_x->CCR1 = pulse_period/3;
				tim_x->ARR = pulse_period;

				g_tim3_set_pulse_num = pulse_count;
				g_curr_tim3_channel = TIM_Channel_1;
				TIM_CCxCmd(TIM3,  TIM_Channel_1, ENABLE);
				break;
				
			// STEP_MOTOR_5
			case 2:
				tmp = get_arr_smoothPwm(&smoothPwm);
				counting_step_smoothPwm(&smoothPwm);
				tim_x->CCR2 = tmp /3;
				tim_x->ARR = tmp;
				smoothPwm.motor_num = 5;

				g_tim3_set_pulse_num = pulse_count;
				g_curr_tim3_channel = TIM_Channel_2;
				TIM_CCxCmd(TIM3,  TIM_Channel_2, ENABLE);
				break;
				
			default:
				break;
		}
		TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
		TIM_Cmd(TIM3, ENABLE);
	}	
	else if(   tim_x ==  TIM4)
	{
		TIM_SetCounter(TIM4, 0);
		TIM_ClearITPendingBit(TIM4 ,TIM_IT_Update);		
		smoothPwm.motor_num = channel_x;

		switch(channel_x)
		{
			case 1:
				tmp = get_arr_smoothPwm(&smoothPwm);
				counting_step_smoothPwm(&smoothPwm);
				tim_x->CCR1 = tmp /3;
				tim_x->ARR = tmp;

				g_tim4_set_pulse_num = pulse_count;
				g_curr_tim4_channel = TIM_Channel_1;
				TIM_CCxCmd(TIM4,  TIM_Channel_1, ENABLE);
				break;
				
			case 2:
				tmp = get_arr_smoothPwm(&smoothPwm);
				counting_step_smoothPwm(&smoothPwm);
				tim_x->CCR2 = tmp /3;
				tim_x->ARR = tmp;

				g_tim4_set_pulse_num = pulse_count;
				g_curr_tim4_channel = TIM_Channel_2;
				TIM_CCxCmd(TIM4, TIM_Channel_2, ENABLE);
				break;
				
			case 3:
				tmp = get_arr_smoothPwm(&smoothPwm);
				counting_step_smoothPwm(&smoothPwm);
				tim_x->CCR3 = tmp /3;
				tim_x->ARR = tmp;

				g_tim4_set_pulse_num = pulse_count;
				g_curr_tim4_channel = TIM_Channel_3;
				TIM_CCxCmd(TIM4,  TIM_Channel_3, ENABLE);
				break;
				
			case 4:
				tmp = get_arr_smoothPwm(&smoothPwm);
				counting_step_smoothPwm(&smoothPwm);
				tim_x->CCR4 = tmp /3;
				tim_x->ARR = tmp;

				g_tim4_set_pulse_num = pulse_count;
				g_curr_tim4_channel = TIM_Channel_4;
				TIM_CCxCmd(TIM4,  TIM_Channel_4, ENABLE);
				break;

			default:
				break;
		}
		TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
		TIM_Cmd(TIM4, ENABLE);
	}
	else
	{}

}

static void prepare_smoothPwm(stSmoothPWM *s, uint16_t total_step_count, uint16_t target_period, uint8_t isFixed)
{
	if(isFixed != FALSE)
	{
		prepare_fixed_smoothPwm(s, total_step_count, target_period);
	}
	else
	{
		prepare_linear_smoothPwm(s, total_step_count, target_period);
	}
}

static void prepare_linear_smoothPwm(stSmoothPWM *s, uint16_t total_step_count, uint16_t target_period)
{
	uint32_t init_pps = PERIOD_CONV/smoothPwm_control.min_period;
    float_t pps = init_pps;
    uint32_t target_pps = PERIOD_CONV/target_period;
	float_t incUp_pps = (float_t)(target_pps - init_pps) / smoothPwm_control.max_up_cnt;
	float_t decDown_pps = (float_t)(target_pps - init_pps)  / smoothPwm_control.max_down_cnt;

#if 0
	printf("init= %d tar= %d ", (int)init_pps, (int)target_pps);
	printf("incUp= %d.%s ", (int)incUp_pps, sub_three(incUp_pps));
	printf("decDown= %d.%s\n", (int)decDown_pps, sub_three(decDown_pps));
#endif

	if(target_period >= MIN_SMOOTHING_PERIOD)
	{
		printf("[Warning] target_period is greater than MIN_SMOOTHING_PERIOD\n");
	}

	// init value
	s->smoothing_Phase = PWMSPEED_UP;
	s->step_cnt = 0;
	s->normal_period = target_period;
	s->end_flag = 0;

	// calculate steps
	uint16_t u,n,d;
	int total_upDown_cnt = smoothPwm_control.max_up_cnt + smoothPwm_control.max_down_cnt;

	if(total_step_count >= total_upDown_cnt)
	{
		u = smoothPwm_control.max_up_cnt;
		d = smoothPwm_control.max_down_cnt;
		n = total_step_count - (u + d);
	}
	else
	{
		n = 0;
		u = (1000 * total_step_count * smoothPwm_control.max_up_cnt +500)/ total_upDown_cnt / 1000;
		d = total_step_count - u;
	}

	s->up_end_step = u;
	s->nor_end_step = n;
	s->down_end_step = d;

	// make array
	uint16_t period = smoothPwm_control.min_period;
	for(int i=0;i<s->up_end_step;i++)
	{
		smoothUpDownPeriod[0][i] = period;
        pps += incUp_pps;
		period = PERIOD_CONV/ pps;
	}

	period = smoothPwm_control.min_period;
    pps = init_pps;
	for(int i=s->down_end_step -1;i>=0;i--)
	{	
		smoothUpDownPeriod[1][i] = period;
		pps += decDown_pps;
		period = PERIOD_CONV/ pps;
	}
}

static void prepare_fixed_smoothPwm(stSmoothPWM *s, uint16_t total_step_count, uint16_t target_period)
{
	/* init value */
	s->smoothing_Phase = PWMSPEED_FIXED;
	s->step_cnt = 0;
	s->normal_period = target_period;
	s->end_flag = 0;
}

static inline uint16_t get_arr_smoothPwm(stSmoothPWM *s)
{
	uint16_t arr;

	if(s->smoothing_Phase == PWMSPEED_NORMAL)
	{
		arr = s->normal_period;
	}
	else if(s->smoothing_Phase == PWMSPEED_FIXED)
	{
		arr = s->normal_period;
	}
	else
	{
		arr = smoothUpDownPeriod[s->smoothing_Phase][s->step_cnt];
	}

	return arr;
}


static void counting_step_smoothPwm(stSmoothPWM *s)
{
	if(s->smoothing_Phase == PWMSPEED_UP)
	{
		s->step_cnt++;
		if(s->step_cnt < s->up_end_step)
		{

		}
		else
		{
			s->step_cnt = 0;
			if(s->nor_end_step > 0)
			{
				s->smoothing_Phase = PWMSPEED_NORMAL;
			}
			else
			{
				s->smoothing_Phase = PWMSPEED_DOWN;
			}
		}
	}
	else if(s->smoothing_Phase == PWMSPEED_NORMAL)
	{
		s->step_cnt++;
		if(s->step_cnt < s->nor_end_step)
		{

		}
		else
		{
			s->step_cnt = 0;
			s->smoothing_Phase = PWMSPEED_DOWN;
		}
	}
	else if(s->smoothing_Phase == PWMSPEED_DOWN)
	{
		s->step_cnt++;
		if(s->step_cnt < s->down_end_step)
		{
			
		}
		else
		{
			s->step_cnt = s->down_end_step-1;	// prevent overflow
			
			if(s->end_flag == 0)
			{
				s->end_flag = 1;		
			}
			else
			{
				printf("[Error] pwm STEP: %d\n", smoothPwm.motor_num);
			}
		}
	}
	else
	{}
}


void drv_pwm_tim3_ch1_init(uint16_t period , uint16_t pulse)
{

	GPIO_InitTypeDef port;
	TIM_TimeBaseInitTypeDef timer;
	TIM_OCInitTypeDef timerPWM;
	NVIC_InitTypeDef 	NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 
		
	/* Enable the TIM1 Pins Software Remapping */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	GPIO_StructInit(&port);
	port.GPIO_Mode 		= GPIO_Mode_AF_PP;
	port.GPIO_Pin 		= GPIO_Pin_6 |GPIO_Pin_7; /* PWM ch1: PA6  ch2: PA7 */
	port.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,  &port);

	TIM_TimeBaseStructInit(&timer);
	timer.TIM_Prescaler 		= 72 -1;
	timer.TIM_Period 			= period;
	timer.TIM_ClockDivision 	= 0;
	timer.TIM_CounterMode 		= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &timer);
		
	TIM_OCStructInit(&timerPWM);
	timerPWM.TIM_Pulse 			= pulse;
	timerPWM.TIM_OCMode 		= TIM_OCMode_PWM2;
	timerPWM.TIM_OutputState 	= TIM_OutputState_Disable;
	timerPWM.TIM_OCPolarity 	= TIM_OCPolarity_High;

	TIM_OC1Init(TIM3, &timerPWM);
	TIM_OC2Init(TIM3, &timerPWM);

	    	/* NVIC Configuration */
    	/* Enable the TIM3_IRQn Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel 						= TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd 					= ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	 
	TIM_CCxCmd(TIM3,  TIM_Channel_1, DISABLE); 
	TIM_CCxCmd(TIM3,  TIM_Channel_2, DISABLE); 
	TIM_Cmd(TIM3, DISABLE); 

	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}


void drv_pwm_tim4_channel_init(uint16_t period , uint16_t pulse)
{
	GPIO_InitTypeDef port;
	TIM_TimeBaseInitTypeDef timer;
	TIM_OCInitTypeDef timerPWM;
	NVIC_InitTypeDef 	NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* Enable the TIM1 Pins Software Remapping */
	GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	

	GPIO_StructInit(&port);
	port.GPIO_Mode 		= GPIO_Mode_AF_PP;
	port.GPIO_Pin 		= GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	port.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,  &port);

	TIM_TimeBaseStructInit(&timer);
	timer.TIM_Prescaler 		= 72-1; 
	timer.TIM_Period 			=  period;
	timer.TIM_ClockDivision 	= 0;
	timer.TIM_CounterMode 		= TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &timer);

	TIM_OCStructInit(&timerPWM);
	timerPWM.TIM_Pulse 			= pulse;
	timerPWM.TIM_OCMode 		= TIM_OCMode_PWM2;
	timerPWM.TIM_OutputState 	= TIM_OutputState_Disable;
	timerPWM.TIM_OCPolarity 	= TIM_OCPolarity_High;
	TIM_OC1Init(TIM4, &timerPWM);
	TIM_OC2Init(TIM4, &timerPWM);
	TIM_OC3Init(TIM4, &timerPWM);
	TIM_OC4Init(TIM4, &timerPWM);

	/* NVIC Configuration */
	/* Enable the TIM4_IRQn Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel 						= TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 			= 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd 					= ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_CCxCmd(TIM4,  TIM_Channel_1, DISABLE);
	TIM_CCxCmd(TIM4,  TIM_Channel_2, DISABLE); 
	TIM_CCxCmd(TIM4,  TIM_Channel_3, DISABLE);
	TIM_CCxCmd(TIM4,  TIM_Channel_4, DISABLE);
	TIM_Cmd(TIM4, DISABLE);

	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
}

void TIM3_IRQHandler(void)
{
	static uint16_t ch_no;
	static uint8_t i;
	static uint16_t tmp;
	
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) 
	{
		for(i=0; i<2; i++)
		{
			if(tim3_pwm_info[i].set_flag ==TRUE)
			{
				tim3_pwm_info[i].g_pwm_cnt++;
				if(i==0)
				{
					om_move_count++;	// bjk 191029
				}
				
				if(tim3_pwm_info[i].g_pwm_cnt >= tim3_pwm_info[i].g_set_pulse_num)
				{
					TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);

					switch(i+1)
					{
						case 1:
							ch_no = TIM_Channel_1;
							break;

						case 2:
							ch_no = TIM_Channel_2;
							drv_limit_sensor_reset_interrupt(STEP_MOTOR_5);
							break;

						default:
							break;
					}

					TIM_CCxCmd(TIM3,  ch_no, DISABLE);
					TIM_Cmd(TIM3, DISABLE);
					tim3_pwm_info[i].set_flag = FALSE;
					tim3_pwm_info[i].g_pwm_cnt = 0;
					tim3_pwm_info[i].g_set_pulse_num = 0;
#if 0	//cartridge check
					/* test 200924 */
					if((g_chamber_check_flag == TRUE) && (tube_sensor != CONTACTED))
					{
						drv_orientalmotor_output_pulse(ORIENTAL_MOTOR, 1, 50, 200);
						/*
						if(tube_sensor == CONTACTED)
						{
							tube_sensor = DETATCHED;
							printf("tube_sensor contacted\n");
						}
						*/
					}
					else if((g_chamber_check_flag == TRUE) && (basket_sensor != CONTACTED))
					{
						drv_orientalmotor_output_pulse(ORIENTAL_MOTOR, 0, 50, 200);
						/*
						if(basket_sensor == CONTACTED)
						{
							basket_sensor = DETATCHED;
							printf("basket_sensor contacted\n");
						}
						*/
					}
					else
					{}
#endif
					om_final_count = om_move_count;
					om_move_count = 0;

					if(MotorZeroInitDone_flag == TRUE)
					{
						if(ch_no == TIM_Channel_1)
						{
							update_om_status(JOB_DONE);
						}
						else
						{
							update_sm_status(5,JOB_DONE);					
						}
					}
				}
				else
				{
					if(i == 1)
					{
						tmp = get_arr_smoothPwm(&smoothPwm);
						TIM3->ARR =	tmp;
						TIM3->CCR2 = tmp/3;
						counting_step_smoothPwm(&smoothPwm);
					}
				}
			}
		}
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

void TIM4_IRQHandler(void)
{
	static uint16_t ch_no;
	static uint8_t i;
	static uint16_t tmp;	
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		for(i=0; i < 4; i++)
		{
			if(tim4_pwm_info[i].set_flag == TRUE)
			{
				tim4_pwm_info[i].g_pwm_cnt++;
		
				if( tim4_pwm_info[i].g_pwm_cnt  >= tim4_pwm_info[i].g_set_pulse_num)
				{
					TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
					switch(i+1)
					{
						case 1:
							ch_no = TIM_Channel_1;
							drv_limit_sensor_reset_interrupt(STEP_MOTOR_1);
							break;

						case 2:
							ch_no = TIM_Channel_2;
							drv_limit_sensor_reset_interrupt(STEP_MOTOR_2);
							break;

						case 3:
							ch_no = TIM_Channel_3;
							drv_limit_sensor_reset_interrupt(STEP_MOTOR_3);
							break;
							
						case 4:
							ch_no = TIM_Channel_4;
							drv_limit_sensor_reset_interrupt(STEP_MOTOR_4);
							break;
						default:
							break;
					}

					TIM_CCxCmd(TIM4,  ch_no, DISABLE);
					TIM_Cmd(TIM4, DISABLE);
					tim4_pwm_info[i].set_flag = FALSE;
					tim4_pwm_info[i].g_pwm_cnt = 0;
					tim4_pwm_info[i].g_set_pulse_num = 0;
					
					if(MotorZeroInitDone_flag == TRUE)
					{
						update_sm_status(i+1,JOB_DONE);	
					}
				}								
				else
				{
					tmp = get_arr_smoothPwm(&smoothPwm);
					TIM4->ARR =	tmp;
					switch(i)
					{
						case 0:
							TIM4->CCR1 = tmp/3;
							break;

						case 1:
							TIM4->CCR2 = tmp/3;
							break;

						case 2:
							TIM4->CCR3 = tmp/3;
							break;
							
						case 3:
							TIM4->CCR4 = tmp/3;
							break;
						default:
							break;
					}
					counting_step_smoothPwm(&smoothPwm);
				}
			}
		}
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}	
}

void drv_pwm_all_stop(void)
{

	TIM_CCxCmd(TIM4,  TIM_Channel_1, DISABLE);
	TIM_CCxCmd(TIM4,  TIM_Channel_2, DISABLE);
	TIM_CCxCmd(TIM4,  TIM_Channel_3, DISABLE);
	TIM_CCxCmd(TIM4,  TIM_Channel_4, DISABLE);

	TIM_CCxCmd(TIM3,  TIM_Channel_1, DISABLE);
	TIM_CCxCmd(TIM3,  TIM_Channel_2, DISABLE);
	
	TIM_Cmd(TIM3, DISABLE);
	TIM_Cmd(TIM4, DISABLE);

	TIM_ITConfig(TIM4, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_Update, DISABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_Update, DISABLE);


	drv_pwm_variable_init();
	printf("Pwm all stop\n");
}

void drv_pwm_Motor_reset(uint16_t tim, uint16_t channel_num)
{
	uint16_t idx = 0;
	// printf("Reset %d, %d\n", tim, channel_num);
	if(tim == 3)
	{
		switch (channel_num)
		{
		case 1:
			idx = 0;
			if(MotorZeroInitDone_flag == TRUE)
			{
				update_om_status(JOB_DONE);
			}
			break;

		case 2:
			idx = 1;
			if(MotorZeroInitDone_flag == TRUE)
			{
				update_sm_status(5,JOB_DONE);
			}
			break;
		
		default:
			break;
		}
		tim3_pwm_info[idx].g_pwm_cnt = 0;
		tim3_pwm_info[idx].set_flag = 0;
		tim3_pwm_info[idx].g_set_pulse_num = 0;
	}
	else if (tim == 4)
	{
		switch (channel_num)
		{
		case 1:
			idx = 0;
			break;

		case 2:
			idx = 1;
			break;

		case 3:
			idx = 2;
			break;

		case 4:
			idx = 3;
			break;

		default:
			break;
		}
		tim4_pwm_info[idx].g_pwm_cnt = 0;
		tim4_pwm_info[idx].set_flag = 0;
		tim4_pwm_info[idx].g_set_pulse_num = 0;

		if(MotorZeroInitDone_flag == TRUE)
		{
			update_sm_status(channel_num,JOB_DONE);
		}
	}
	else
	{}
}

void drv_pwm_smoothPwm_control(uint16_t max_up_cnt, uint16_t max_down_cnt, uint16_t min_period)
{
	if( (max_up_cnt > UPDOWN_ARRAY_SIZE) || (max_down_cnt > UPDOWN_ARRAY_SIZE ))
	{
		printf("[Error] overflow\n");
		return;
	}

	if((max_up_cnt == 0) || (max_down_cnt == 0))
	{
		printf("[Error] Zero is not allowed\n");
		return;
	}

	smoothPwm_control.max_down_cnt = max_down_cnt;
	smoothPwm_control.max_up_cnt = max_up_cnt;
	smoothPwm_control.min_period = min_period;
}

void drv_pwm_variable_init(void)
{
	uint8_t i;

	for( i = 0; i < 4; i++)
	{
		tim4_pwm_info[i].set_flag 		= FALSE;
		tim4_pwm_info[i].g_set_pulse_num 	= 0;
		tim4_pwm_info[i].g_pwm_cnt 		= 0;
	}

	for( i = 0; i < 2; i++)
	{
		tim3_pwm_info[i].set_flag 		= FALSE;
		tim3_pwm_info[i].g_set_pulse_num 	= 0;
		tim3_pwm_info[i].g_pwm_cnt 		= 0;
	}

	 tim3_init_flag = FALSE;
	 tim4_init_flag = FALSE;

	 g_period_tim4 = 0;
	 g_period_tim3 = 0;

	 g_pwm_tim3_cnt = 0;
	 g_pwm_tim4_cnt = 0;
	 g_tim3_set_pulse_num = 0;
	 g_tim4_set_pulse_num = 0;

	 g_curr_tim3_channel = 1;
	 g_curr_tim4_channel = 1;
	 
	 //smooth pwm
	smoothPwm_control.max_down_cnt = MAX_UPDOWN_COUNT;
	smoothPwm_control.max_up_cnt = MAX_UPDOWN_COUNT;
	smoothPwm_control.min_period = MIN_SMOOTHING_PERIOD;

	// drv_pwm_smoothPwm_control(200, 100, 6000);	// TEST
}
