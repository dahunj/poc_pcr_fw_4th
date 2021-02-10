
#ifndef GLOB_VAR_IF_H_
#define GLOB_VAR_IF_H_

#include "stm32f10x.h"
#include "stm3210b_eval.h"
#include  "stm32f10x_gpio.h"
#include "drv_timer.h"
#include "task.h"


extern int g_ROUTINE_CYCLE_MAX; //40 [1,100]  == MEA_ROUTINE_NO
extern uint8_t Optic_Measure_Index_Flag[101];

extern int g_Cooling_Sec; // 15; // 臾쇱쓽 ��寃� �삩�룄源뚯� 鍮⑤━ �룄�떖�븯湲� �쐞�빐�꽌 �떇�엳湲� �쐞�븳 �떆媛�. Default value = 15.  [0, 60]
extern int g_Optic_Operation_Keeping_Temp_Sec; // 39 [0,60]
extern int g_Optic_No_Operation_Keeping_Temp_Sec; // 30 [0,60]
extern int g_Delay_time_Before_Opting_Runing; // 15 [0,60]
extern int g_Keeping_Minute_Peltier_Temperature; // 10 [0,60]
extern int g_Keeping_time_for_High_Temperature; // 10 [0,60]

extern float g_HEAT_SETPOINT;  // 96,  [80, 120]
extern float g_COOL_SETPOINT; // 60, [0,120]
extern int g_Keeping_Minute_Peltier_Temperature; // 10 [0,60] minutes.
extern float g_PRE_COND_SETPOINT;   // 96 [0,120]

extern int g_RT_Keeping_Minute_Peltier_Temperature;
extern float g_RT_PRE_COND_SETPOINT;

void Check_Glob_Var(char *, int szmsg);
void Print_Glob_Vars();
void Filter_Input(char *msg, int szmsg);

#endif //GLOB_VAR_IF_H_
