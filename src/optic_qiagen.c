#include "modbus_hal.h"
#include "optic_qiagen.h"

#define AGEING_TEST 0
uint8_t infinity_opt_flag = FALSE;
extern uint8_t infinity_start_stop_flag_opt;

extern uint8_t self_check_flag;

qia_mangType qia_mang[2];


static void optic_qiagen_routine(qia_mangType *mang)
{
    if(mang->job_state == QIA_JOB_RUN)
    {
        switch(mang->optic_state)
        {
            case OPTIC_ST_RESET:
                mod_trigger_ctrl(mang->port, ON);
                mod_reset_ctrl(mang->port, ON);

                mang->job_time_since = get_time_ms_cnt();
                mang->job_state = QIA_JOB_WAIT;
                // init variables
                mang->init_idx = 0;
                mang->init_done_flag = FALSE;
                printf("opt[%d]> qiagen reset\n", mang->port);
            break;

            case OPTIC_ST_INIT_READ:
                mod_send(mang->port, FUNC_READING,  mang->setting[mang->init_idx].startRegister, mang->setting[mang->init_idx].bytes);
                mang->job_time_since = get_time_ms_cnt();
                mang->job_state = QIA_JOB_WAIT;
            break;

            case OPTIC_ST_INIT_WRITE:
                mod_send(mang->port, FUNC_WRITING,  mang->setting[mang->init_idx].startRegister, mang->setting[mang->init_idx].value);
                mang->job_time_since = get_time_ms_cnt();
                mang->job_state = QIA_JOB_WAIT;
            break;

            case OPTIC_ST_INIT_SAVE:
                mod_send(mang->port, FUNC_WRITING,  ADD_REG_SAVE_ACTUAL_OR_DEFAULT_PARAMETERS, 0);  /* 0: Save actual, 1: Default parameters */
                mang->job_time_since = get_time_ms_cnt();
                mang->job_state = QIA_JOB_WAIT;
            break;

            case OPTIC_ST_READ_ALL:
                mod_send(mang->port, FUNC_READING,  mang->setting[mang->init_idx].startRegister, mang->setting[mang->init_idx].bytes);
                mang->job_time_since = get_time_ms_cnt();
                mang->job_state = QIA_JOB_WAIT;
            break;

            case OPTIC_ST_NORMAL:
                switch(mang->meas_state)
                {
                    case QIA_MEAS_NONE:
                    break;

                    case QIA_MEAS_START:
                        mod_send(mang->port, FUNC_WRITING, ADD_REG_START_METHOD_COMMAND, 1);
                        mang->job_time_since = get_time_ms_cnt();
                        mang->job_state = QIA_JOB_WAIT;
                        mang->first_ticket_flag = TRUE;
                    break;

                    case QIA_MEAS_CHCK:
                        mod_send(mang->port, FUNC_READING, ADD_REG_TICKET, 2);
                        mang->job_time_since = get_time_ms_cnt();
                        mang->job_state = QIA_JOB_WAIT;
                    break;

                    case QIA_MEAS_STOP:
                        mod_send(mang->port, FUNC_WRITING, ADD_REG_STOP_METHOD_COMMAND, 1);
                        mang->job_time_since = get_time_ms_cnt();
                        mang->job_state = QIA_JOB_WAIT;
                        mang->meas_value_idx = 0;
                    break;

                    case QIA_MEAS_GET:
                        if(mang->meas_value_idx == 0)
                        {
                            mod_send(mang->port, FUNC_READING, ADD_REG_ON_VALUE_1, 2);
                            mang->job_time_since = get_time_ms_cnt();
                            mang->job_state = QIA_JOB_WAIT;
                        }
                        else if(mang->meas_value_idx == 1)
                        {
                            mod_send(mang->port, FUNC_READING, ADD_REG_ON_VALUE_3, 2);
                            mang->job_time_since = get_time_ms_cnt();
                            mang->job_state = QIA_JOB_WAIT;
                        }
                        else if(mang->meas_value_idx == 2)
                        {
                            mod_send(mang->port, FUNC_READING, ADD_REG_OFF_VALUE_1, 2);
                            mang->job_time_since = get_time_ms_cnt();
                            mang->job_state = QIA_JOB_WAIT;
                        }
                        else if(mang->meas_value_idx == 3)
                        {
                            mod_send(mang->port, FUNC_READING, ADD_REG_OFF_VALUE_3, 2);
                            mang->job_time_since = get_time_ms_cnt();
                            mang->job_state = QIA_JOB_WAIT;
                        }
                        else
                        {}
                        
                    break;

                    default:
                    break;
                }
            break;

            default:
            break;
        }
    }
    else if(mang->job_state == QIA_JOB_WAIT)
    {
        uint32_t time;
        uint8_t val;

        switch(mang->optic_state)
        {
            case OPTIC_ST_RESET:
                time = get_time_ms_cnt() - mang->job_time_since;
                if(time >= 2000)
                {
                    mang->job_state = QIA_JOB_DONE;
                    // printf("opt[%d]> reset done\n", mang->port);
                }
            break;

            case OPTIC_ST_INIT_READ:
                val = mod_rx_waiting(mang->port);

                if(val == MOD_ST_RX_GOOD)
                {
                    uint16_t cnt = mang->setting_cnt;
                    mang->init_idx++;
                    if(mang->init_idx <cnt)
                    {
                        mang->job_state = QIA_JOB_RUN;
                    }
                    else
                    {
                        mang->job_state = QIA_JOB_DONE;
                    }
                }
                else if(val == MOD_ST_RX_TIMEOUT)
                {
                    printf("opt[%d]> init failed\n", mang->port);

                    if(self_check_flag == TRUE)		//test 210121
                    {
                    	printf("opt[%d] FAULT\n", mang->port);
                    }
                    mang->job_state = QIA_JOB_RUN;
                    // TODO: Is it right that normal state?
                    mang->optic_state = OPTIC_ST_NORMAL;
                    mang->init_idx = 0;
                }
                else if(val > 0 )
                {
                    // If failed, goto write state.
                    printf("opt[%d]> try reg write\n", mang->port);
                    mang->job_state = QIA_JOB_RUN;
                    mang->optic_state = OPTIC_ST_INIT_WRITE;
                    mang->init_idx = 0;
                }
                else
                {}
            break;

            case OPTIC_ST_INIT_WRITE:
                val = mod_rx_waiting(mang->port);
                // TODO: timeout?
                if(val == MOD_ST_RX_GOOD)
                {
                    uint16_t cnt = mang->setting_cnt;
                    mang->init_idx++;
                    if(mang->init_idx <cnt)
                    {
                        mang->job_state = QIA_JOB_RUN;
                    }
                    else
                    {
                        mang->job_state = QIA_JOB_DONE;
                    }
                }
                else if(val > 0 )
                {
                    // TODO: test me
                    printf("opt[%d]> write failed\n", mang->port);
                    mang->job_state = QIA_JOB_RUN;
                    mang->optic_state = OPTIC_ST_NORMAL;
                }
                else
                {}

            break;

            case OPTIC_ST_INIT_SAVE:
                val = mod_rx_waiting(mang->port);

                if(val == MOD_ST_RX_GOOD)
                {
                    mang->job_state = QIA_JOB_DONE;
                    printf("opt[%d]> OPTIC_ST_INIT_SAVE= done\n", mang->port);
                }
                else if(val >0)
                {
                    printf("opt[%d]> OPTIC_ST_INIT_SAVE= not good\n", mang->port);
                    mang->job_state = QIA_JOB_DONE;     // TODO: is failed?
                }
                else
                {}
            break;

            case OPTIC_ST_READ_ALL:
                val = mod_rx_waiting(mang->port);

                if(val == MOD_ST_RX_GOOD)
                {
                    mod_rxMsgType * rxMsg;
                    rxMsg = mod_get_rxMxg(mang->port);

                    printf("opt[%d] reg= %d data= %d\n", mang->port, rxMsg->startRegister, rxMsg->u32Data);

                    uint16_t cnt = mang->setting_cnt;
                    mang->init_idx++;
                    if(mang->init_idx <cnt)
                    {
                        mang->job_state = QIA_JOB_RUN;
                    }
                    else
                    {
                        mang->job_state = QIA_JOB_DONE;
                    }
                }
                else if(val == MOD_ST_RX_TIMEOUT)
                {
                    printf("opt[%d]> read all, timeout\n", mang->port);
                    mang->job_state = QIA_JOB_RUN;
                    mang->optic_state = OPTIC_ST_NORMAL;
                    mang->init_idx = 0;
                }
                else if(val > 0 )
                {
                    printf("opt[%d]> read all, bad\n", mang->port);
                    mang->job_state = QIA_JOB_RUN;
                    mang->optic_state = OPTIC_ST_NORMAL;
                    mang->init_idx = 0;
                }
                else
                {}
            break;

            case OPTIC_ST_NORMAL:
                switch(mang->meas_state)
                {
                    case QIA_MEAS_START:
                        val = mod_rx_waiting(mang->port);

                        if(val == MOD_ST_RX_GOOD)
                        {
                            mang->job_state = QIA_JOB_DONE;
                            // printf("opt[%d]> QIA_MEAS_START\n", mang->port);
                        }
                    break;

                    case QIA_MEAS_CHCK:
                        val = mod_rx_waiting(mang->port);

                        if(val == MOD_ST_RX_GOOD)
                        {
                            if(mang->first_ticket_flag)
                            {
                                mang->ticket = mod_get_rxMsg_data(mang->port);
                                mang->first_ticket_flag = FALSE;
                                mang->job_state = QIA_JOB_RUN;
                            }
                            else
                            {
                                uint32_t ticket = mod_get_rxMsg_data(mang->port);
                                if(mang->ticket != ticket)
                                {
                                    // printf("opt[%d]> QIA_MEAS_CHCK\n", mang->port);
                                    mang->job_state = QIA_JOB_DONE;
                                }
                                else
                                {
                                    mang->job_state = QIA_JOB_RUN;
                                }
                            }
                        }

                    break;

                    case QIA_MEAS_STOP:
                        val = mod_rx_waiting(mang->port);

                        if(val == MOD_ST_RX_GOOD)
                        {
                            mang->job_state = QIA_JOB_DONE;
                            // printf("opt[%d]> QIA_MEAS_STOP\n", mang->port);
                        }
                    break;

                    case QIA_MEAS_GET:
                        val = mod_rx_waiting(mang->port);

                        if(val == MOD_ST_RX_GOOD)
                        {
                            uint32_t meas_val = mod_get_rxMsg_data(mang->port);

                            switch(mang->meas_value_idx)
                            {
                                case 0:
                                    mang->e1d1_onOff_raw[0] = meas_val;
                                break;

                                case 1:
                                    mang->e2d2_onOff_raw[0] = meas_val;
                                break;

                                case 2:
                                    mang->e1d1_onOff_raw[1] = meas_val;
                                break;

                                case 3:
                                    mang->e2d2_onOff_raw[1] = meas_val;
                                break;

                                default:
                                break;
                            }

                            mang->job_state = QIA_JOB_RUN;
                            mang->meas_value_idx++;

                            if(mang->meas_value_idx <4)
                            {
                                // printf("opt> led%d= %u\n", mang->meas_value_idx, meas_val);
                            }
                            else
                            {
                                mang->job_state = QIA_JOB_DONE;
                            }
                        }
                    break;

                    default:
                    break;
                }
            break;

				default:
					break;
		}
	}
	else if(mang->job_state == QIA_JOB_DONE)
	{
		switch(mang->optic_state)
		{
			case OPTIC_ST_RESET:
				mang->optic_state = OPTIC_ST_INIT_READ;
				mang->job_state = QIA_JOB_RUN;
				break;

			case OPTIC_ST_INIT_READ:
				mang->optic_state = OPTIC_ST_NORMAL;
				mang->job_state = QIA_JOB_RUN;
				mang->init_done_flag = TRUE;
				printf("opt[%d]> init read done\n", mang->port);
				break;

			case OPTIC_ST_INIT_WRITE:
				#if OPTIC_SAVE_EN
				mang->optic_state = OPTIC_ST_INIT_SAVE;
				#else
				mang->optic_state = OPTIC_ST_NORMAL;
				#endif
				mang->init_done_flag = TRUE;
				printf("opt[%d]> init write done\n", mang->port);

				if(self_check_flag == TRUE)		//test 210121
				{
					printf("opt[d%] OK\n", mang->port);
				}
				mang->job_state = QIA_JOB_RUN;
#if AGEING_TEST
                if(infinity_start_stop_flag_opt == TRUE)		/*test 0324*/
                {
                	infinity_start_stop_flag_opt = FALSE;
                	infinity_opt_flag = TRUE;
                }
#endif
				break;

            case OPTIC_ST_INIT_SAVE:
                mang->optic_state = OPTIC_ST_NORMAL;
                mang->job_state  = QIA_JOB_RUN;
            break;

            case OPTIC_ST_READ_ALL:
                mang->optic_state = OPTIC_ST_NORMAL;
                mang->job_state  = QIA_JOB_RUN;
            break;

            case OPTIC_ST_NORMAL:
                switch(mang->meas_state)
                {
                    case QIA_MEAS_START:
                        mang->meas_state = QIA_MEAS_CHCK;
                        mang->job_state  = QIA_JOB_RUN;
                    break;

                    case QIA_MEAS_CHCK:
                        mang->meas_state = QIA_MEAS_STOP;
                        mang->job_state  = QIA_JOB_RUN;
                    break;

                    case QIA_MEAS_STOP:
                        mang->meas_state = QIA_MEAS_GET;
                        mang->job_state  = QIA_JOB_RUN;
                    break;

                    case QIA_MEAS_GET:
                        mang->meas_state = QIA_MEAS_NONE;
                        mang->job_state  = QIA_JOB_RUN;
                        
                        mang->e1d1_meas = mang->e1d1_onOff_raw[0] - mang->e1d1_onOff_raw[1];
//                       mang->e2d2_meas = mang->e2d2_onOff_raw[0] - mang->e2d2_onOff_raw[1];

                        if(mang->e2d2_onOff_raw[0] >= mang->e2d2_onOff_raw[1])	/* 20.06.30 test */
                        {
                        	mang->e2d2_meas = mang->e2d2_onOff_raw[0] - mang->e2d2_onOff_raw[1];
                        }
                        else
                        {
                        	mang->e2d2_meas = 0;
                        }
//                        printf("opt[%d]> e1d1= %u e2d2= %u\n", mang->port, mang->e1d1_meas, mang->e2d2_meas);

                        {
                            float tmp;
                            tmp = (float) mang->e1d1_meas*OPT_MULT_FACTOR;
                            printf("e1d1[%d]= %d.", mang->port, (int)tmp);
                            printf("%s mV\n",sub_three(tmp));

                            tmp = (float) mang->e2d2_meas*OPT_MULT_FACTOR;
                            printf("e2d2[%d]= %d.", mang->port, (int)tmp);
                            printf("%s mV\n",sub_three(tmp));
                        }

                        if(mang->end_callback != NULL)
                        {
                            mang->end_callback(mang->port);                            
                            mang->end_callback = NULL;
                        }
                    break;

                    default:
                    break;
                }
            break;

            default:
            break;
        }
    }
    else
    {}
}

static void request_measure(qia_mangType *mang, optic_FuncType callback)
{
    mang->meas_state = QIA_MEAS_START;
    mang->end_callback = callback;
}

static void get_measure(qia_mangType *mang, uint32_t *e1d1_val, uint32_t *e2d2_val)
{
    if(e1d1_val != NULL)
    {
        *e1d1_val = mang->e1d1_meas;
    }

    if(e2d2_val != NULL)
    {
        *e2d2_val = mang->e2d2_meas;
    }
}

//------------------------------------------------------------------------------------
// interface
//------------------------------------------------------------------------------------

void optic_qiagen_request_measure(uint8_t port, optic_FuncType callback)
{
    if(port == MODBUS_PORT1)
    {
        request_measure(&qia_mang[0], callback);
    }
    else if(port == MODBUS_PORT2)
    {
        request_measure(&qia_mang[1], callback);
    }
    else
    {}
}

void optic_qiagen_get_measure(uint8_t port, uint32_t *e1d1_val, uint32_t *e2d2_val)
{
    if(port == MODBUS_PORT1)
    {
        get_measure(&qia_mang[0], e1d1_val, e2d2_val);
    }
    else if(port == MODBUS_PORT2)
    {
        get_measure(&qia_mang[1], e1d1_val, e2d2_val);
    }
    else
    {}
}

float optic_qiagen_convert_raw_to_mV(uint32_t raw_input)
{
    return ((float)raw_input * OPT_MULT_FACTOR);
}

void optic_qiagen_test(void)
{
//     mod_msg_proc(MODBUS_PORT1);
//     optic_qiagen_routine(&qia_mang[MODBUS_PORT1]);

    // optic_qiagen_request_measure(MODBUS_PORT1, NULL);
    // optic_qiagen_send_msg(MODBUS_PORT1, BASE_OPT_MEA, QIA_E1D1_MDOE, 1);

    optic_qiagen_read_all(MODBUS_PORT1);
}

void optic_qiagen_msg_process(void)
{
    mod_msg_proc(MODBUS_PORT1);
    optic_qiagen_routine(&qia_mang[MODBUS_PORT1]);

    mod_msg_proc(MODBUS_PORT2);
    optic_qiagen_routine(&qia_mang[MODBUS_PORT2]);
}

void optic_qiagen_init(void)
{
    uint16_t cnt;

    qia_mang[0].job_state = QIA_JOB_RUN;
    qia_mang[0].optic_state = OPTIC_ST_RESET;
    qia_mang[0].meas_state = QIA_MEAS_NONE;
    qia_mang[0].port = MODBUS_PORT1;
    qia_mang[0].end_callback = NULL;
    mod_get_setting(MODBUS_PORT1, &qia_mang[0].setting, &cnt);
    qia_mang[0].setting_cnt = cnt;


    // printf("sizeof(q1_model[2]])= %d\n", sizeof(q1_model[2]));
    // printf("sizeof(Optic_settingType)= %d\n", sizeof(Optic_settingType));
    // printf("setting_cnt = %d\n", qia_mang[0].setting_cnt);
    
    qia_mang[1].job_state = QIA_JOB_RUN;
    qia_mang[1].optic_state = OPTIC_ST_RESET;
    qia_mang[1].meas_state = QIA_MEAS_NONE;
    qia_mang[1].port = MODBUS_PORT2;
    qia_mang[1].end_callback = NULL;
    mod_get_setting(MODBUS_PORT2, &qia_mang[1].setting, &cnt);
    qia_mang[1].setting_cnt = cnt;

    // qia_mang[1].setting = &q1_model[1][0];
    // qia_mang[1].setting_cnt = sizeof(q1_model[2]) / sizeof(Optic_settingType);
}

void optic_qiagen_reset(void)
{
    optic_qiagen_init();
}

void optic_qiagen_read_all(uint8_t port)
{
    qia_mangType *mang;

    if(port == MODBUS_PORT1)
    {
        mang = &qia_mang[0];
    }
    else if(port == MODBUS_PORT2)
    {
        mang = &qia_mang[1];
    }
    else
    {
        return;
    }

    if((mang->optic_state == OPTIC_ST_NORMAL) && (mang->job_state == QIA_JOB_RUN))
    {
        mang->optic_state = OPTIC_ST_READ_ALL;
        mang->init_idx = 0;
    }
}
