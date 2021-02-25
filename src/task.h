/*
 * task.h
 *
 *  Created on: 2019. 6. 18.
 *      Author: jk.choi
 */

#ifndef TASK_H_
#define TASK_H_


#include "stm32f10x.h"
#include "hw_config.h"
//#include "optic_measure.h"
#include "drv_adafruit_mcp9600.h"
#include "peltier_ctrl.h"
#include "drv_peltier.h"


#if 0
#define 	SEQ_DESC_IDX_MAX 	300
#else
#define 	SEQ_DESC_IDX_MAX 	1100
#define 	MEA_SEQ_DESC_IDX_MAX 	400//150//100//60 //50

#endif

#define 	Q_BUF_MAX 			300
#define 	JOB_MSG_IDX		3
#define	RECV_BUF_MAX 		50

#define	DEBUG 
#define	SM_OPS_MAX	5
#define MEASURE_OP_EVENT		0x3E
#define ACTION_DONE_EVENT	0x3D
#define ACTION_TRIGGER_EVENT	0x3C

#define E1D1	1
#define E1D2	2
#define E2D2	3

#define CONTINUE 	0
#define DIS_CONT 	1

#define CHAM_1	1
#define CHAM_2	2
#define CHAM_3	3
#define CHAM_4	4

/*
chamber number description
				  -----
				  | 2 |
				  |   |
			------     ------
opt_1 ->	| 1           3 |	  <- opt_2
			------     ------
				  |   |
				  | 4 |
				  -----
				    ก่
				  front

*/
#if 0	//Ver1.0 chamber
#define CHAM_1_3	1
#define CHAM_2_4	2
#define CHAM_3_1	3
#define CHAM_4_2	4
#endif
#if 1	//Ver2.0 chamber
#define CHAM_1_2	1
#define CHAM_2_3	2
#define CHAM_3_4	3
#define CHAM_4_1	4
#endif

/*****************************/
typedef unsigned long	uint32_t;
typedef unsigned short	uint16_t;
typedef unsigned char	uint8_t;

/****************************/
typedef struct
{
	uint32_t 	cycle_time;
	uint32_t 	offset_time;
	void 		(*func)();
	uint8_t		task_max;
	uint8_t		task_id;

}stTaskType;


typedef enum
{
	TASKID_NONE = 0,
	TASKID_COM_PC = 1,
	TASKID_Q ,
	TASKID_SM1,
	TASKID_SM2,
	TASKID_SM3,
	TASKID_SM4,
	TASKID_SM5,
	TASKID_OM,
	TASKID_PEL,
	TASKID_OPT,
	TASKID_MNG,
	TASKID_MAX //12
}E_TASK_ID;


typedef struct
{
	uint8_t			task_id;
	uint32_t 		job_gap_time;
	uint32_t 		goal_value;		// optic: chamber number 1~4
	uint16_t 		move_speed; 	// 
	uint32_t 		config;			// optic: CONTINUE, DIS_CONT
	uint8_t 		direction;
//	uint32_t 		keep_time;
}  stSeqDesc;


typedef enum {
   JOB_NONE = 0,
   JOB_READY = 1,
   JOB_RUN = 2,
   JOB_DONE = 3
} eJobStatus;


typedef struct
{
	uint8_t		task_id; 		
	uint16_t		task_cycle;
	uint8_t		job_index;
    	eJobStatus 	job_status; 	
	uint32_t		goal_value;
	uint32_t		curr_value;
	uint8_t 		direction;
	uint32_t		keep_time;
	uint8_t		init_flag;
	uint8_t		job_completed_flag;

}  stTaskOp;


typedef struct
{
	stSeqDesc 	*seq_ptr_head;
	stSeqDesc 	*seq_ptr_curr;
	stTaskOp		*task_ptr;
	uint32_t		act_desc_idx;
	uint8_t		task_id;
	uint8_t 		task_max;
	uint8_t     	regist_task_cnt;
	uint8_t		seq_compld_flag;
	uint8_t		seqact_compl_flags[SEQ_DESC_IDX_MAX];
	uint32_t		prev_start_time;
	uint8_t		job_assigned_flag;
	void			*p_other_opr_ptr;  	// Add 10.22
	uint16_t		seq_desc_cnt;		/* Add 1121 */

} stOprInfo;

typedef struct
{
	uint8_t		head_ptr;
	uint8_t		tail_ptr;
	uint8_t 		q_buf[Q_BUF_MAX];
}stQueue;

/* Add 1205 */
typedef enum {
	STOP_POCPCR = 0,
	RUN_POCPCR = 1
} ePocPcr_State;

//ePocPcr_State g_pocpcr_state = STOP_POCPCR; /* Add 1205 */


void task_regist(E_TASK_ID task_id, uint32_t cycle_time, uint32_t offset_time, void (*task_func)());
void task_run(void);
void task_init(void);
void task_q(void);
void task_com_pc(void);
void task_sm1(void);
void task_sm2(void);
void task_sm3(void);
void task_sm4(void);
void task_sm5(void);
void task_om(void);
void task_pel(void);
void task_opt(void);
void task_allstop(void);
void task_manage(void);

stTaskType* get_taskDesc(E_TASK_ID task_id);
stSeqDesc* get_seqDesc(uint8_t job_id);

uint8_t check_seq_compl(void);
void enqueue_job(uint8_t enq_data);
uint8_t dequeue_job(void);

void queue_init(void);
uint8_t msg_parse(void);
eJobStatus update_sm_status(uint8_t ch, eJobStatus status);
eJobStatus update_om_status(eJobStatus status);
void set_event(uint8_t evt);
void delay(u16 x);
//void callback_opt_status(uint8_t status);
void callback_opt_status(void);
uint16_t get_seq_cnt(stSeqDesc* p_seq_desc);
void opr_ptr_init(stOprInfo* ptr);

uint8_t door_sensor_check(void);
void drv_UV_LED(uint8_t on_off);

#endif /* TASK_H_ */
