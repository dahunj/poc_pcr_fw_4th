#include "modbus_hal.h"
#include "optic_qiagen.h"
#include "optic_service.h"

extern qia_mangType qia_mang[2];

qia_taskType task_mang[2] ={
    {.opt_mang = &qia_mang[0], .task_step = QIA_TASK_READY},
    {.opt_mang = &qia_mang[1], .task_step = QIA_TASK_READY},
};

extern uint8_t g_rountine_cnt;
extern uint8_t g_optic_test1_on_flag;
extern uint8_t g_optic_test2_on_flag;
extern uint8_t g_op_mea_exe_flag;


void optic_qiagen_task_init(void)
{
    task_mang[MODBUS_PORT1].task_step = QIA_TASK_READY;

    for(int i=0; i< CHAM_MAX; i++)
    {
        task_mang[MODBUS_PORT1].base_e1d1_meas_idx[i] = 0;
        task_mang[MODBUS_PORT1].base_e2d2_meas_idx[i] = 0;
        task_mang[MODBUS_PORT1].last_e1d1_meas[i] = 0;
        task_mang[MODBUS_PORT1].last_e2d2_meas[i] = 0;
        //200728 test
        task_mang[MODBUS_PORT1].cycle_e1d1_meas[i] = 0;
        task_mang[MODBUS_PORT1].cycle_e2d2_meas[i] = 0;
    }

    memset(task_mang[MODBUS_PORT1].base_e1d1_meas, 0, sizeof(task_mang[MODBUS_PORT1].base_e1d1_meas));
    memset(task_mang[MODBUS_PORT1].base_e2d2_meas, 0, sizeof(task_mang[MODBUS_PORT1].base_e2d2_meas));


    task_mang[MODBUS_PORT2].task_step = QIA_TASK_READY;

    for(int i=0; i< CHAM_MAX; i++)
    {
        task_mang[MODBUS_PORT2].base_e1d1_meas_idx[i] = 0;
        task_mang[MODBUS_PORT2].base_e2d2_meas_idx[i] = 0;
        task_mang[MODBUS_PORT2].last_e1d1_meas[i] = 0;
        task_mang[MODBUS_PORT2].last_e2d2_meas[i] = 0;
        //200728 teset
        task_mang[MODBUS_PORT2].cycle_e1d1_meas[i] = 0;
        task_mang[MODBUS_PORT2].cycle_e2d2_meas[i] = 0;
    }

    memset(task_mang[MODBUS_PORT2].base_e1d1_meas, 0, sizeof(task_mang[MODBUS_PORT2].base_e1d1_meas));
    memset(task_mang[MODBUS_PORT2].base_e2d2_meas, 0, sizeof(task_mang[MODBUS_PORT2].base_e2d2_meas));

    optic_qiagen_reset();

    printf("opt> task init\n");
}

//200728 test
void optic_cycle_data_init(void)
{
    for(int i=0; i< CHAM_MAX; i++)
    {
        //200728 test
        task_mang[MODBUS_PORT1].cycle_e1d1_meas[i] = 0;
        task_mang[MODBUS_PORT1].cycle_e2d2_meas[i] = 0;
    }
    for(int i=0; i< CHAM_MAX; i++)
    {
        //200728 teset
        task_mang[MODBUS_PORT2].cycle_e1d1_meas[i] = 0;
        task_mang[MODBUS_PORT2].cycle_e2d2_meas[i] = 0;
    }
    printf("optic_cycle_data_init\n");
}

static void send_to_pc(uint8_t item_code, uint8_t sub_item_code, uint32_t value)
{
    uint8_t tmp_buf[4];

    tmp_buf[0] = (value  >> 24) & 0xFF;
    tmp_buf[1] = (value  >> 16) & 0xFF;
    tmp_buf[2] = (value  >> 8) & 0xFF;
    tmp_buf[3] = value  & 0xFF;

    modbus_pack_pc(item_code , sub_item_code, tmp_buf, 4 );
}

uint8_t optic_qiagen_check_msg(void)
{
	uint8_t ret;

	if(g_rountine_cnt == BASE_MEA_ROUTINE_NO)
	{
		ret = BASE_OPT_MEA;
	}
	else if( g_rountine_cnt == MEA_ROUTINE_NO)
	{
		ret = OPT_MEA;
	}
	else if(Optic_Measure_Index_Flag[g_rountine_cnt]==1)
	{
		ret = CYCLE_OPT_MEA;
	}
	else
	{
		ret = FALSE;
	}

	return ret;
}


void send_measure_msg(qia_taskType *mang, uint8_t msg_id, uint8_t ed_mode, uint8_t chamber)
{
    uint8_t cham_idx =  chamber -1;

    if(cham_idx >= CHAM_MAX)
    {
        printf("opt> error send_measure_msg cham_idx= %d\n", cham_idx);
        return;
    }

//    printf("opt[%d]> send msg id= %d, chamber= %d, ed mode= %d\n", mang->opt_mang->port, msg_id, chamber, ed_mode);

    if( msg_id == BASE_OPT_MEA )
	{
		if( ed_mode == QIA_E1D1_MDOE)
		{
            // calculate average
            uint32_t tmp = 0;

            for(int i=0;i<mang->base_e1d1_meas_idx[cham_idx];i++)
            {
                tmp += mang->base_e1d1_meas[cham_idx][i];
            }

            if(mang->base_e1d1_meas_idx[cham_idx] > 0)
            {
                tmp = tmp / (mang->base_e1d1_meas_idx[cham_idx]);
            }

//            printf("opt[%d]> mode= %d avg= %d idx= %d\n", mang->opt_mang->port, ed_mode, tmp, mang->base_e1d1_meas_idx[cham_idx]);

            if(mang->opt_mang->port == MODBUS_PORT1)
            {
                if( chamber == 1)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT_BASE_1CH_E1D1, tmp);
                }
                else if( chamber == 2)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT_BASE_2CH_E1D1, tmp);
                }
                else if( chamber == 3)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT_BASE_3CH_E1D1, tmp);
                }
                else if( chamber == 4)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT_BASE_4CH_E1D1, tmp);
                }
                else
                {}
            }
            else if(mang->opt_mang->port == MODBUS_PORT2)
            {
                if( chamber == 1)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT2_BASE_1CH_E1D1, tmp);
                }
                else if( chamber == 2)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT2_BASE_2CH_E1D1, tmp);
                }
                else if( chamber == 3)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT2_BASE_3CH_E1D1, tmp);
                }
                else if( chamber == 4)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT2_BASE_4CH_E1D1, tmp);
                }
                else
                {}
            }
            else
            {}
        }
        else if(ed_mode == QIA_E2D2_MDOE)   /* E2D2: ROX */
        {
            // calculate average
            uint32_t tmp = 0;

            for(int i=0;i<mang->base_e2d2_meas_idx[cham_idx];i++)
            {
                tmp += mang->base_e2d2_meas[cham_idx][i];
            }

            if(mang->base_e2d2_meas_idx[cham_idx] > 0)
            {
                tmp = tmp / (mang->base_e2d2_meas_idx[cham_idx]);
            }

//           printf("opt[%d]> mode= %d avg= %d idx= %d\n", mang->opt_mang->port, ed_mode, tmp, mang->base_e2d2_meas_idx[cham_idx]);
            
            if(mang->opt_mang->port == MODBUS_PORT1)
            {
                if( chamber == 1)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT_BASE_1CH_E2D2, tmp);
                }
                else if( chamber == 2)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT_BASE_2CH_E2D2, tmp);
                }
                else if( chamber == 3)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT_BASE_3CH_E2D2, tmp);
                }
                else if( chamber == 4)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT_BASE_4CH_E2D2, tmp);
                }
                else
                {}
            }
            else if(mang->opt_mang->port == MODBUS_PORT2)
            {
                if( chamber == 1)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT2_BASE_1CH_E2D2, tmp);
                }
                else if( chamber == 2)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT2_BASE_2CH_E2D2, tmp);
                }
                else if( chamber == 3)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT2_BASE_3CH_E2D2, tmp);
                }
                else if( chamber == 4)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT2_BASE_4CH_E2D2, tmp);
                }
                else
                {}
            }
            else
            {}
        }
        else
        {}
    }
    else if(msg_id == OPT_MEA)
    {
        if(ed_mode == QIA_E1D1_MDOE)
        {
            if(mang->opt_mang->port == MODBUS_PORT1)
            {
                if(chamber == 1)    /* 1st : TB */
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT_VAL_1CH_E1D1, mang->last_e1d1_meas[cham_idx]);
                }
                else if(chamber == 2)       /* 2nd :  NTM */
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT_VAL_2CH_E1D1, mang->last_e1d1_meas[cham_idx]);
                }
                else if(chamber == 3)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT_VAL_3CH_E1D1, mang->last_e1d1_meas[cham_idx]);
                }
                else if(chamber == 4)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT_VAL_4CH_E1D1, mang->last_e1d1_meas[cham_idx]);
                }
                else
                {}
            }
            else if(mang->opt_mang->port == MODBUS_PORT2)
            {
                if(chamber == 1)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT2_VAL_1CH_E1D1, mang->last_e1d1_meas[cham_idx]);
                }
                else if(chamber == 2)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT2_VAL_2CH_E1D1, mang->last_e1d1_meas[cham_idx]);
                }
                else if(chamber == 3)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT2_VAL_3CH_E1D1, mang->last_e1d1_meas[cham_idx]);
                }
                else if(chamber == 4)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT2_VAL_4CH_E1D1, mang->last_e1d1_meas[cham_idx]);
                }
                else
                {}
            }
            else
            {}
        }
        else if(ed_mode == QIA_E2D2_MDOE)
        {
            if(mang->opt_mang->port == MODBUS_PORT1)
            {
                if(chamber == 1)    /* 1st: IC of channel 1 */
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT_VAL_1CH_E2D2, mang->last_e2d2_meas[cham_idx]);
                }
                else if(chamber == 2)       /* 2nd: IC of channel 2 */
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT_VAL_2CH_E2D2, mang->last_e2d2_meas[cham_idx]);
                }
                else if(chamber == 3)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT_VAL_3CH_E2D2, mang->last_e2d2_meas[cham_idx]);
                }
                else if(chamber == 4)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT_VAL_4CH_E2D2, mang->last_e2d2_meas[cham_idx]);
                }
                else
                {}
            }
            else if(mang->opt_mang->port == MODBUS_PORT2)
            {
                if(chamber == 1)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT2_VAL_1CH_E2D2, mang->last_e2d2_meas[cham_idx]);
                }
                else if(chamber == 2)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT2_VAL_2CH_E2D2, mang->last_e2d2_meas[cham_idx]);
                }
                else if(chamber == 3)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT2_VAL_3CH_E2D2, mang->last_e2d2_meas[cham_idx]);
                }
                else if(chamber == 4)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT2_VAL_4CH_E2D2, mang->last_e2d2_meas[cham_idx]);
                }
                else
                {}
            }
            else
            {}
        }
        else
        {}
    }
    //200728 test
    else if(msg_id == CYCLE_OPT_MEA) // @@@@
    {
        if(ed_mode == QIA_E1D1_MDOE)
        {
            if(mang->opt_mang->port == MODBUS_PORT1)
            {
                if(chamber == 1)    /* 1st : TB */
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT_VAL_1CH_E1D1, mang->cycle_e1d1_meas[cham_idx]);
                }
                else if(chamber == 2)       /* 2nd :  NTM */
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT_VAL_2CH_E1D1, mang->cycle_e1d1_meas[cham_idx]);
                }
                else if(chamber == 3)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT_VAL_3CH_E1D1, mang->cycle_e1d1_meas[cham_idx]);
                }
                else if(chamber == 4)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT_VAL_4CH_E1D1, mang->cycle_e1d1_meas[cham_idx]);
                }
                else
                {}
            }
            else if(mang->opt_mang->port == MODBUS_PORT2)
            {
                if(chamber == 1)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT2_VAL_1CH_E1D1, mang->cycle_e1d1_meas[cham_idx]);
                }
                else if(chamber == 2)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT2_VAL_2CH_E1D1, mang->cycle_e1d1_meas[cham_idx]);
                }
                else if(chamber == 3)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT2_VAL_3CH_E1D1, mang->cycle_e1d1_meas[cham_idx]);
                }
                else if(chamber == 4)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT2_VAL_4CH_E1D1, mang->cycle_e1d1_meas[cham_idx]);
                }
                else
                {}
            }
            else
            {}
        }
        else if(ed_mode == QIA_E2D2_MDOE)
        {
            if(mang->opt_mang->port == MODBUS_PORT1)
            {
                if(chamber == 1)    /* 1st: IC of channel 1 */
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT_VAL_1CH_E2D2, mang->cycle_e2d2_meas[cham_idx]);
                }
                else if(chamber == 2)       /* 2nd: IC of channel 2 */
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT_VAL_2CH_E2D2, mang->cycle_e2d2_meas[cham_idx]);
                }
                else if(chamber == 3)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT_VAL_3CH_E2D2, mang->cycle_e2d2_meas[cham_idx]);
                }
                else if(chamber == 4)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT_VAL_4CH_E2D2, mang->cycle_e2d2_meas[cham_idx]);
                }
                else
                {}
            }
            else if(mang->opt_mang->port == MODBUS_PORT2)
            {
                if(chamber == 1)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT2_VAL_1CH_E2D2, mang->cycle_e2d2_meas[cham_idx]);
                }
                else if(chamber == 2)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT2_VAL_2CH_E2D2, mang->cycle_e2d2_meas[cham_idx]);
                }
                else if(chamber == 3)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT2_VAL_3CH_E2D2, mang->cycle_e2d2_meas[cham_idx]);
                }
                else if(chamber == 4)
                {
                    send_to_pc(FC_OPT_VAL , SFC_OPT2_VAL_4CH_E2D2, mang->cycle_e2d2_meas[cham_idx]);
                }
                else
                {}
            }
            else
            {}
        }
        else
        {}
    }
    else
    {}
}

void optic_qiagen_send_msg(uint8_t port, uint8_t msg_id, uint8_t ed_mode, uint8_t chamber)
{
    qia_taskType *mang;

    if((port == MODBUS_PORT1) || (port == MODBUS_PORT2))
    {
        mang = &task_mang[port];
    }
    else
    {
        printf("opt[%d]> error port\n", port);

        return;
    }

    if(mang->opt_mang->init_done_flag == FALSE) // optic module have not been initialised.
    {
        return;
    }

    send_measure_msg(mang, msg_id, ed_mode, chamber);
}

// TODO: check state of result.
void optic_qiagen_callback(uint8_t port)
{
    if((port == MODBUS_PORT1) || (port == MODBUS_PORT2))
    {
        task_mang[port].task_step = QIA_TASK_DONE;
    }

    // printf("opt[%d]> callback\n", port);
}

static void save_measure(qia_taskType *mang, uint32_t chamber, uint32_t e1d1_val, uint32_t e2d2_val)
{
    uint8_t cham_idx =  chamber -1;

    if(cham_idx < CHAM_MAX)
    {
//        printf("opt[%d]> save chamber= %d\n", mang->opt_mang->port, chamber);

        if(mang->base_e1d1_meas_idx[cham_idx] < BASE_MEAS_MAX)
        {
//            printf("opt[%d]> base save, e1d1 idx= %d\n", mang->opt_mang->port, mang->base_e1d1_meas_idx[cham_idx]);
            mang->base_e1d1_meas[cham_idx][mang->base_e1d1_meas_idx[cham_idx]] = e1d1_val;
            mang->base_e1d1_meas_idx[cham_idx]++;
        }

        if(mang->base_e2d2_meas_idx[cham_idx] < BASE_MEAS_MAX)
        {
//            printf("opt[%d]> base save, e2d2 idx= %d\n", mang->opt_mang->port, mang->base_e2d2_meas_idx[cham_idx]);
            mang->base_e2d2_meas[cham_idx][mang->base_e2d2_meas_idx[cham_idx]] = e2d2_val;
            mang->base_e2d2_meas_idx[cham_idx]++;
        }

        mang->last_e1d1_meas[cham_idx] = e1d1_val;
        mang->last_e2d2_meas[cham_idx] = e2d2_val;
        //200728 test
        mang->cycle_e1d1_meas[cham_idx] = e1d1_val;
        mang->cycle_e2d2_meas[cham_idx] = e2d2_val;
	}
}

static uint8_t measure_process(qia_taskType *mang, uint32_t chamber, uint16_t delay_ms)
{
    uint8_t rslt = FALSE;

    uint32_t check_time = 0;		//201203 test
//    uint8_t cham_idx =  chamber -1;

    // if(cham_idx >= CHAM_MAX)
    // {
    //     printf("opt> error optic_qiagen_measure_process\n");
    //     return FALSE;
    // }

    if(mang->task_step == QIA_TASK_READY)
    {
        if(delay_ms >0)
        {
            mang->ready_time_tick = get_time_ms_cnt();
            mang->task_step = QIA_TASK_DELAY;
        }
        else
        {
            mang->task_step = QIA_TASK_START;
            check_time = get_time_ms_cnt();		//201203 test
//            printf("OPTIC action start: %d\n", check_time);
            optic_qiagen_request_measure(mang->opt_mang->port, optic_qiagen_callback);
        }
    }
    else if(mang->task_step == QIA_TASK_DELAY)
    {
        uint32_t time = get_time_ms_cnt() - mang->ready_time_tick;

        if(time >= delay_ms)
        {
            mang->task_step = QIA_TASK_START;
            optic_qiagen_request_measure(mang->opt_mang->port, optic_qiagen_callback);
//            printf("opt[%d]> delay_ms= %d\n", mang->opt_mang->port, time);
        }
    }
    else if(mang->task_step == QIA_TASK_DONE)
    {
        uint32_t e1d1_val, e2d2_val;

        optic_qiagen_get_measure(mang->opt_mang->port, &e1d1_val, &e2d2_val);

        if((g_optic_test1_on_flag != TRUE) && (g_optic_test2_on_flag != TRUE) && (g_op_mea_exe_flag == TRUE))
        {
        	check_time = get_time_ms_cnt();		//201203 test
//        	printf("OPTIC action finish: %d\n", check_time);
            save_measure(mang, chamber, e1d1_val, e2d2_val);
        }

        rslt = TRUE;
        mang->task_step = QIA_TASK_NONE;

        if(g_optic_test1_on_flag == TRUE)
        {
            send_to_pc(FC_OPT_VAL, SFC_OPT_VAL_1CH_TB, e1d1_val);
            send_to_pc(FC_OPT_VAL, SFC_OPT_VAL_1CH_IC, e2d2_val);
            // g_optic_test1_on_flag = FALSE;
        }
        
        if(g_optic_test2_on_flag == TRUE)
        {
            // send_to_pc(FC_OPT_VAL, SFC_OPT_VAL_1CH_IC, e2d2_val);
            send_to_pc(FC_OPT_VAL, SFC_OPT2_VAL_3CH_E1D1, e1d1_val);
            send_to_pc(FC_OPT_VAL, SFC_OPT2_VAL_3CH_E2D2, e2d2_val);

            // g_optic_test2_on_flag = FALSE;
        }
    }
    else if(mang->task_step == QIA_TASK_NONE)
    {
        rslt = TRUE;
    }
    else
    {}

    return rslt;
}


uint8_t optic_qiagen_measure_process(uint8_t port, uint32_t chamber, uint16_t delay_ms)
{
    qia_taskType *mang;
    uint8_t rslt;

    if((port == MODBUS_PORT1) || (port == MODBUS_PORT2))
    {
        mang = &task_mang[port];
    }
    else
    {
        printf("opt[%d]> error port\n", port);

        return FALSE;
    }

    if(mang->opt_mang->init_done_flag == FALSE) // optic module have not been initialised.
    {
        return TRUE;
    }

    rslt = measure_process(mang, chamber, delay_ms);

    return rslt;
}

static void measure_ready(uint8_t port)
{
    qia_taskType *mang;

    if((port == MODBUS_PORT1) || (port == MODBUS_PORT2))
    {
        mang = &task_mang[port];
    }
    else
    {
        printf("opt[%d]> error port\n", port);

        return;
    }

    mang->task_step = QIA_TASK_READY;
}

void optic_qiagen_measure_ready(void)
{
    measure_ready(MODBUS_PORT1);
    measure_ready(MODBUS_PORT2);
}
