/* Includes */
#include <stddef.h>
#include "stm32f10x.h"

#include "task.h"
#include "hw_config.h"
#include "stdio.h"
#include "stdlib.h"

#include "Glob_Var_IF.h"

extern  stTaskType	task_desc[12];

// 아래 변수는 운전 중에는 변경 못하게 블록킹 필요.

int g_ROUTINE_CYCLE_MAX = 45;

int g_Cooling_Sec = 15; // come to target water temperature, keeping time.  Default value = 15.  [0, 60]
int g_Optic_Operation_Keeping_Temp_Sec = 50;//40; //38 [0,60]
int g_Optic_No_Operation_Keeping_Temp_Sec = 40;//30; //30 [0,60]

int g_Delay_time_Before_Opting_Runing = 35;//25; //15 [0,60]
int g_Keeping_time_for_High_Temperature = 6; // 10 [0,60]
float g_HEAT_SETPOINT = 95;  // 96,  [80, 120]
float g_COOL_SETPOINT = 65; // 60, [0,120]
int g_Keeping_Minute_Peltier_Temperature = 1; // 10 [0,60] minutes.

float g_PRE_COND_SETPOINT = 95.0;   // 96 [0,120]

int g_RT_Keeping_Minute_Peltier_Temperature = 5;
float g_RT_PRE_COND_SETPOINT = 50.0; // 50 1cycle


uint8_t Optic_Measure_Index_Flag[101] =
{
		// 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
		   0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // 9
		   0, 0, 0, 0, 0, 1, 1, 1, 1, 1,  // 19
		   1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 29
		   1, 1, 1, 1, 1, 1, 1, 1, 1, 1,  // 39
		   1, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // 49
		   0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // 59
		   0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // 69
		   0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // 79
		   0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // 89
		   0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  // 99
		   0
};
// g_rountine_cnt ?이 특정 값일 때, 동작.
// 기존에는 switch( g_rountine_cnt )  case 5,6,7,MEA_ROUTINE_NO  <==================== 검토 요
// switch( g_rountine_cnt ) 구문 참조

/*
 *
====================================
PC->MCU 명령어
:[변수명] [변수값]

SHOW_GLOB_VARS

ROUTINE_CYCLE_MAX 40
COOLING_SEC 15
OPTIC_OPERATION_KEEPING_TEMP_SEC 39
OPTIC_NO_OPERATION_KEEPING_TEMP_SEC 30
DELAY_TIME_BEFORE_OPTING_RUNING 15                           <--- 스펠링 오류 무시
KEEPING_MINUTE_PELTIER_TEMPERATURE 10
KEEPING_TIME_FOR_HIGH_TEMPERATURE 10
HEAT_SETPOINT 96
COOL_SETPOINT 60
PRE_COND_SETPOINT 96

MEASURE_INDEX  0 4 7 9     <=== 최대 변수값 개수는 ROUTINE_CYCLE_MAX 미만
====================================

 */

void Filter_Input(char *msg, int szmsg)
{
	//Check_Glob_Var(msg, szmsg);

}

void Check_Glob_Var(char *msg, int szmsg)
{
	//Print_Glob_Vars();

	if(strncmp(msg, "stop", 5)==0)
	{

	}
	if(strncmp(msg, "start", 6)==0)
	{

	}
	if(strncmp(msg, "optic1", 7)==0)
	{

	}
	if(strncmp(msg, "optic2", 7)==0)
	{

	}
	if(strncmp(msg, "optRead1",9)==0)
	{

	}
	if(strncmp(msg, "optRead2",9)==0)
	{

	}
	if(strncmp(msg, "forceFan",9)==0)
	{

	}
	if(strncmp(msg, "FanOff",7)==0)
	{

	}
	if(strncmp(msg, "readTemp",9)==0)
	{

	}
	if(strncmp(msg, "SelfTest485",12)==0)
	{

	}
	//////////////////////////////////////////////////////////////////////////////////
	// set internal global variables
	if(strncmp(msg, "FINAL_CYCLE", 11)==0) // dflt 40 [1,100]
	{
		int vv;
		sscanf(msg+12, "%d", &vv);
		if(vv>=1 && vv<=100)
		{
			g_ROUTINE_CYCLE_MAX = vv;
			printf("g_FINAL_CYCLE = %d\n", g_ROUTINE_CYCLE_MAX);
		}
	}
	if(strncmp(msg, "COOLING_SEC", 11)==0) // dflt 15 [0,60]
	{
		int vv;
		sscanf(msg+12, "%d", &vv);
		if(vv>=0 && vv<=60)
		{
			g_Cooling_Sec = vv;
			printf("g_COOLING_SEC = %d, input=%d\n", g_Cooling_Sec, vv);
		}
	}
	if(strncmp(msg, "OPTIC_OPERATION_KEEPING_TEMP_SEC", 32)==0) // 39 [0,60]
	{
		int vv;
		sscanf(msg+33, "%d", &vv);
		if(vv>=0 && vv<=60)
		{
			g_Optic_Operation_Keeping_Temp_Sec = vv;
			printf("g_OPTIC_OPERATION_KEEPING_TEMP_SEC = %d\n", g_Optic_Operation_Keeping_Temp_Sec);
		}
	}
	if(strncmp(msg, "2nd_STEP_KEEPING_TIME_SEC", 25)==0) // 30 [0,60]
	{
		int vv;
		sscanf(msg+26, "%d", &vv);
		if(vv>=0 && vv<=60)
		{
			g_Optic_No_Operation_Keeping_Temp_Sec = vv;
			printf("g_2nd_STEP_KEEPING_TIME_SEC = %d\n", g_Optic_No_Operation_Keeping_Temp_Sec);
		}
	}
	if(strncmp(msg, "DELAY_TIME_BEFORE_OPTING_RUNING", 31)==0) // 15 [0,60]                           <--- 스펠링 오류 무시
	{
		int vv;
		sscanf(msg+32, "%d", &vv);
		if(vv>=0 && vv<=60)
		{
			g_Delay_time_Before_Opting_Runing = vv;
			printf("g_DELAY_TIME_BEFORE_OPTING_RUNING = %d\n", g_Delay_time_Before_Opting_Runing);
		}
	}
	if(strncmp(msg, "PRECOND_KEEPING_TIME_MIN", 24)==0) // 10 [0,60]
	{
		int vv;
		sscanf(msg+25, "%d", &vv);
		if(vv>=0 && vv<=60)
		{
			g_Keeping_Minute_Peltier_Temperature = vv;
			printf("g_PRECOND_KEEPING_TIME_MIN = %d\n", g_Keeping_Minute_Peltier_Temperature);
		}
	}
	if(strncmp(msg, "1st_STEP_KEEPING_TIME_SEC", 25)==0) // 10 [0, 60]
	{
		int vv;
		sscanf(msg+26, "%d", &vv);
		if(vv>=0 && vv<=60)
		{
			g_Keeping_time_for_High_Temperature = vv;
			printf("g_1st_STEP_KEEPING_TIME_SEC = %d\n", g_Keeping_time_for_High_Temperature);
		}
	}
	if(strncmp(msg, "1st_STEP_SETPOINT", 17)==0) // 96 [80,120]
	{
		int vv;
		sscanf(msg+18, "%d", &vv); // should check float operation valid. C/C++ Build ==> Settings ==> C Linker ==> Miscellaneous -u _printf_float
		if(vv>=0 && vv<=120)
		{
			g_HEAT_SETPOINT = vv;
			printf("g_1st_STEP_SETPOINT = %d\n", (int)g_HEAT_SETPOINT);
		}
	}
	if(strncmp(msg, "2nd_STEP_SETPOINT", 17)==0) // 60 [0,120]
	{
		int vv;
		sscanf(msg+18, "%d", &vv);
		if(vv>=0 && vv<=120)
		{
			g_COOL_SETPOINT = vv;
			printf("g_2nd_STEP_SETPOINT = %d\n", (int)g_COOL_SETPOINT);
		}
	}

	if(strncmp(msg, "PRE_COND_SETPOINT", 17)==0) // 96 [0, 120]
	{
		int vv;
		sscanf(msg+18, "%d", &vv);
		if(vv>=0 && vv<=120)
		{
			g_PRE_COND_SETPOINT = vv;
			printf("g_PRE_COND_SETPOINT = %d\n", (int)g_PRE_COND_SETPOINT);
		}
	}
	//RT PRE COND SETPOINT
	if(strncmp(msg, "RT_PRE_COND_SETPOINT", 20)==0) // 96 [0, 120]
	{
		int vv;
		sscanf(msg+21, "%d", &vv);
		if(vv>=0 && vv<=120)
		{
			g_RT_PRE_COND_SETPOINT = vv;//g_PRE_COND_SETPOINT = vv;
			printf("g_RT_PRE_COND_SETPOINT = %d\n", (int)g_RT_PRE_COND_SETPOINT);
		}
	}
	//RT_PRECOND_KEEPING_TIME_MIN added by dahunj
	if(strncmp(msg, "RT_PRECOND_KEEPING_TIME_MIN", 27)==0) // 10 [0,60]
	{
		int vv;
		sscanf(msg+28, "%d", &vv);
		if(vv>=0 && vv<=60)
		{
			g_RT_Keeping_Minute_Peltier_Temperature = vv;
			printf("g_RT_PRECOND_KEEPING_TIME_MIN = %d\n", g_RT_Keeping_Minute_Peltier_Temperature);
		}
	}


	if(strncmp(msg, "MEASURE_INDEX_SET", 17)==0) //   0 4 7 9     <=== 최대 변수값 개수는 ROUTINE_CYCLE_MAX 미만
	{										  // 내부 수신 버퍼가 100 byte !!!
		//Optic_Measure_Index_Flag[100];
		// refer to peltier_ctrl.c :: if(g_rountine_cnt==5) // @@@ Optic_Measure_Index_Flag
		int vv;
		sscanf(msg+18, "%d", &vv);
		if(vv>=0 && vv<=100)
		{
			Optic_Measure_Index_Flag[vv] = 1;
			printf("Measure Index %d : %d\n", vv, Optic_Measure_Index_Flag[vv]);
		}
	}
	if(strncmp(msg, "MEASURE_INDEX_RESET", 19)==0) //   0 4 7 9     <=== 최대 변수값 개수는 ROUTINE_CYCLE_MAX 미만
	{										  // 내부 수신 버퍼가 100 byte !!!
		//Optic_Measure_Index_Flag[100];
		// refer to peltier_ctrl.c :: if(g_rountine_cnt==5) // @@@ Optic_Measure_Index_Flag
		int vv;
		sscanf(msg+20, "%d", &vv);
		if(vv>=0 && vv<=100)
		{
			Optic_Measure_Index_Flag[vv] = 0;
			printf("Measure Index %d : %d\n", vv, Optic_Measure_Index_Flag[vv]);
		}
	}
	if(strncmp(msg, "FAN2_ON", 7)==0)
	{
//		GPIO_WriteBit( GPIOB, GPIO_Pin_15, 1);
		drv_inside_LED(ON);
//		drv_water_fan(ON);
		printf("FAN2_ON\n");
	}
	if(strncmp(msg, "FAN2_OFF", 8)==0)
	{
//		GPIO_WriteBit( GPIOB, GPIO_Pin_15, 0);
		drv_inside_LED(OFF);
//		drv_water_fan(OFF);
		printf("FAN2_OFF\n");
	}
	if(strncmp(msg, "DOOR_LOCK", 9)==0)		//DOOR_OPEN
	{
		door_lock_open(OFF);
		printf("DOOR_CLOSE\n");
	}
	if(strncmp(msg, "DOOR_UNLOCK", 11)==0)		//DOOR_CLOSE
	{
		door_lock_open(ON);
		printf("DOOR_OPEN\n");
	}
	if(strncmp(msg, "FELTIER_TEST", 12)==0)		//bjk 200902 add
	{
		printf("FELTIER_TEST\n");
		drv_self_check();
	}
	if(strncmp(msg, "check_start", 11)==0)		//bjk 200907 add
	{
		uint8_t state;
		state = door_sensor_check();
		if(state == OPEN)
		{
			//printf("g_check_door\n");
			printf("g_ok_door\n");
		}
		else if(state == CLOSE)
		{
			printf("g_ok_door\n");
		}
		else
		{}
	}
	if(strncmp(msg, "UV_C_START", 10)==0)		//bjk 201021 test
	{

	}


	if(strncmp(msg, "SHOW_GLOB_VARS", 14)==0)
	{
		Print_Glob_Vars();
	}
}
void Print_Glob_Vars()
{
   printf("<<<<<<<<<<<<<<<<<<  Glob. Variables >>>>>>>>>>>>>>>>>>>>>\n");
   for(int i = 0; i < 10000; i++)
   {}
   printf("g_FINAL_CYCLE = %d\n", g_ROUTINE_CYCLE_MAX);
   for(int i = 0; i < 10000; i++)
   {}
//   printf("COOLING_SEC = %d\n", g_Cooling_Sec);
   printf("g_OPTIC_OPERATION_KEEPING_TEMP_SEC = %d\n", g_Optic_Operation_Keeping_Temp_Sec);
   for(int i = 0; i < 10000; i++)
   {}
   printf("g_DELAY_TIME_BEFORE_OPTING_RUNING = %d\n", g_Delay_time_Before_Opting_Runing);
   for(int i = 0; i < 10000; i++)
   {}
   printf("g_1st_STEP_KEEPING_TIME_SEC = %d\n", g_Keeping_time_for_High_Temperature);
   for(int i = 0; i < 10000; i++)
   {}
   printf("g_1st_STEP_SETPOINT = %d\n", (int)g_HEAT_SETPOINT);
   for(int i = 0; i < 10000; i++)
   {}
   printf("g_2nd_STEP_KEEPING_TIME_SEC = %d\n", g_Optic_No_Operation_Keeping_Temp_Sec);
   for(int i = 0; i < 10000; i++)
   {}
   printf("g_2nd_STEP_SETPOINT = %d\n", (int)g_COOL_SETPOINT);
   for(int i = 0; i < 10000; i++)
   {}
   printf("g_PRECOND_KEEPING_TIME_MIN = %d\n", g_Keeping_Minute_Peltier_Temperature);
   for(int i = 0; i < 10000; i++)
   {}
   printf("g_PRE_COND_SETPOINT = %d\n", (int)g_PRE_COND_SETPOINT);
   for(int i = 0; i < 10000; i++)
   {}
    printf("g_RT_PRECOND_KEEPING_TIME_MIN = %d\n", g_RT_Keeping_Minute_Peltier_Temperature);
   for(int i = 0; i < 10000; i++)
   {}
   printf("g_RT_PRE_COND_SETPOINT = %d\n", (int)g_RT_PRE_COND_SETPOINT);
   for(int i = 0; i < 10000; i++)
   {}


   printf("---------------------------------------------------------\n");
   /*
   printf("<<<<<<<<<<<<<<<<<<  Glob. Variables >>>>>>>>>>>>>>>>>>>>>\n");
   printf("g_ROUTINE_CYCLE_MAX = %d\n", g_ROUTINE_CYCLE_MAX);
   printf("g_Cooling_Sec = %d\n", g_Cooling_Sec);
   printf("g_Optic_Operation_Keeping_Temp_Sec = %d\n", g_Optic_Operation_Keeping_Temp_Sec);
   printf("g_Optic_No_Operation_Keeping_Temp_Sec = %d\n", g_Optic_No_Operation_Keeping_Temp_Sec);
   printf("g_Delay_time_Before_Opting_Runing = %d\n", g_Delay_time_Before_Opting_Runing);
   printf("g_Keeping_Minute_Peltier_Temperature = %d\n", g_Keeping_Minute_Peltier_Temperature);
   printf("g_Keeping_time_for_High_Temperature = %d\n", g_Keeping_time_for_High_Temperature);
   printf("g_HEAT_SETPOINT = %d\n", (int)g_HEAT_SETPOINT);
   printf("g_COOL_SETPOINT = %d\n", (int)g_COOL_SETPOINT);
   printf("g_PRE_COND_SETPOINT = %d\n", (int)g_PRE_COND_SETPOINT);
   printf("---------------------------------------------------------\n");
   printf("MEASURE INDEX LIST\n");
   for(int i=0; i<101; i++)
   {
      if(Optic_Measure_Index_Flag[i]!=0)
      {
         printf("  Index[%d] = %d\n", Optic_Measure_Index_Flag[i]);
      }
   }
   printf("---------------------------------------------------------\n");
   */
}


