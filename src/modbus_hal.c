#include "modbus_hal.h"

#define RETRY_WAIT_TIME_MS 800

static stTrxData txBuffer[2];
static stTrxData rxBuffer[2];
static mod_ManageType mod_mang[2];
static uint8_t rxData[2][TRX_BUF_MAX];
static MOD_COMM_ST_E mod_state[2];

extern uint16_t g_optic_current_FAM;
extern uint16_t g_optic_current_ROX;
extern uint16_t g_optic_current_HEX;
extern uint16_t g_optic_current_CY5;



#if 1
// note: If 'bytes' is 1, high byte of value is used.
 Optic_settingType q1_model[2][29] = {
		{
				{ADD_REG_CYCLES,				2, 1},
				{ADD_REG_CYCLE_TIME,			2, 1},
				{ADD_REG_START_MODE,			1, STARTMODE_0_CMD},
				{ADD_REG_METHOD_TYPE,			1, METHOD_TYPE_5_E1D1_E2D2},
				{ADD_REG_DARK_SIGNAL_TYPE,		1, DARK_SIGNAL_TYPE_2_DARK_EVERY},
				{ADD_REG_AVERAGE,				1, AVERAGE_3},
				{ADD_REG_LED_MODE,				1, LED_MODE_0_TOGGLE},
				{ADD_REG_TRIGGER_DELAY,			1, 0},
				{ADD_REG_E1D1_FACTOR,			2, 0x0000},
				{ADD_REG_E1D1_FACTOR+1,			2, 0x0001},
				{ADD_REG_E2D2_FACTOR,			2, 0x0000},
				{ADD_REG_E2D2_FACTOR+1,			2, 0x0001},
				{ADD_REG_E1D1_OFFSET,			2, 0x0000},
				{ADD_REG_E1D1_OFFSET+1,			2, 0x0000},
				{ADD_REG_E2D2_OFFSET,			2, 0x0000},
				{ADD_REG_E2D2_OFFSET+1,			2, 0x0000},
				{ADD_REG_ON_DELAY_LED1,			2, 100},	/* 300 */
				{ADD_REG_ON_DELAY_LED2,			2, 100},
				{ADD_REG_OFF_DELAY_LED1,		2, 100},
				{ADD_REG_OFF_DELAY_LED2,		2, 100},
				{ADD_REG_LED1_CURRENT,			1, LED1_CURRENT_20},	/* HEX */
				{ADD_REG_LED2_CURRENT,			1, LED2_CURRENT_10},	/* CY5 */
				{ADD_REG_LED1_CURRENT_DEFAULT,	1, LED1_CURRENT_100},
				{ADD_REG_LED2_CURRENT_DEFAULT,	1, LED2_CURRENT_100},
				{ADD_REG_LED1_CURRENT_MAX,		1, LED1_CURRENT_250},
				{ADD_REG_LED2_CURRENT_MAX,		1, LED2_CURRENT_250},
				{ADD_REG_LED1_CURRENT_MIN,		1, LED1_CURRENT_00},
				{ADD_REG_LED2_CURRENT_MIN,		1, LED2_CURRENT_00},
				{ADD_REG_ADC_SAMPLING,			2, 100},
		},
		{
		        {ADD_REG_CYCLES,                2, 1},
		        {ADD_REG_CYCLE_TIME,            2, 1},
		        {ADD_REG_START_MODE,            1, STARTMODE_0_CMD},
		        {ADD_REG_METHOD_TYPE,           1, METHOD_TYPE_5_E1D1_E2D2},
		        {ADD_REG_DARK_SIGNAL_TYPE,      1, DARK_SIGNAL_TYPE_2_DARK_EVERY},
		        {ADD_REG_AVERAGE,               1, AVERAGE_3},
		        {ADD_REG_LED_MODE,              1, LED_MODE_0_TOGGLE},
		        {ADD_REG_TRIGGER_DELAY,         1, 0},
		        {ADD_REG_E1D1_FACTOR,           2, 0x0000},
		        {ADD_REG_E1D1_FACTOR+1,         2, 0x0001},
		        {ADD_REG_E2D2_FACTOR,           2, 0x0000},
		        {ADD_REG_E2D2_FACTOR+1,         2, 0x0001},
		        {ADD_REG_E1D1_OFFSET,           2, 0x0000},
		        {ADD_REG_E1D1_OFFSET+1,         2, 0x0000},
		        {ADD_REG_E2D2_OFFSET,           2, 0x0000},
		        {ADD_REG_E2D2_OFFSET+1,         2, 0x0000},
		        {ADD_REG_ON_DELAY_LED1,         2, 100},	/* 300 */
		        {ADD_REG_ON_DELAY_LED2,         2, 100},
		        {ADD_REG_OFF_DELAY_LED1,        2, 100},
		        {ADD_REG_OFF_DELAY_LED2,        2, 100},
		        {ADD_REG_LED1_CURRENT,          1, LED1_CURRENT_20},	/* FAM */
		        {ADD_REG_LED2_CURRENT,          1, LED2_CURRENT_20},	/* HEX filter */
		        {ADD_REG_LED1_CURRENT_DEFAULT,  1, LED1_CURRENT_100},
		        {ADD_REG_LED2_CURRENT_DEFAULT,  1, LED2_CURRENT_100},
		        {ADD_REG_LED1_CURRENT_MAX,      1, LED1_CURRENT_250},
		        {ADD_REG_LED2_CURRENT_MAX,      1, LED2_CURRENT_250},
		        {ADD_REG_LED1_CURRENT_MIN,      1, LED1_CURRENT_00},
		        {ADD_REG_LED2_CURRENT_MIN,      1, LED2_CURRENT_00},
		        {ADD_REG_ADC_SAMPLING,          2, 100},
		},
};
#else
// TEST TEST
const Optic_settingType q1_model[2][16] = {
    {
        {ADD_REG_CYCLES,                2, 1},
        {ADD_REG_CYCLE_TIME,            2, 1},
        {ADD_REG_START_MODE,            1, STARTMODE_0_CMD},
        {ADD_REG_METHOD_TYPE,           1, METHOD_TYPE_5_E1D1_E2D2},
        {ADD_REG_DARK_SIGNAL_TYPE,      1, DARK_SIGNAL_TYPE_2_DARK_EVERY},
        {ADD_REG_AVERAGE,               1, AVERAGE_1},
        {ADD_REG_LED_MODE,              1, LED_MODE_0_TOGGLE},
        {ADD_REG_TRIGGER_DELAY,         1, 0},
        {ADD_REG_LED1_CURRENT,          1, LED1_CURRENT_100},
        {ADD_REG_LED2_CURRENT,          1, LED1_CURRENT_100},
        {ADD_REG_LED1_CURRENT_DEFAULT,  1, LED1_CURRENT_100},
        {ADD_REG_LED2_CURRENT_DEFAULT,  1, LED1_CURRENT_100},
        {ADD_REG_LED1_CURRENT_MAX,      1, LED1_CURRENT_130},
        {ADD_REG_LED2_CURRENT_MAX,      1, LED1_CURRENT_130},
        {ADD_REG_LED1_CURRENT_MIN,      1, LED1_CURRENT_80},
        {ADD_REG_LED2_CURRENT_MIN,      1, LED1_CURRENT_80},
    },
    {
        {ADD_REG_CYCLES,                2, 1},
        {ADD_REG_CYCLE_TIME,            2, 1},
        {ADD_REG_START_MODE,            1, STARTMODE_0_CMD},
        {ADD_REG_METHOD_TYPE,           1, METHOD_TYPE_5_E1D1_E2D2},
        {ADD_REG_DARK_SIGNAL_TYPE,      1, DARK_SIGNAL_TYPE_2_DARK_EVERY},
        {ADD_REG_AVERAGE,               1, AVERAGE_1},
        {ADD_REG_LED_MODE,              1, LED_MODE_0_TOGGLE},
        {ADD_REG_TRIGGER_DELAY,         1, 0},
        {ADD_REG_LED1_CURRENT,          1, LED1_CURRENT_100},
        {ADD_REG_LED2_CURRENT,          1, LED1_CURRENT_100},
        {ADD_REG_LED1_CURRENT_DEFAULT,  1, LED1_CURRENT_100},
        {ADD_REG_LED2_CURRENT_DEFAULT,  1, LED1_CURRENT_100},
        {ADD_REG_LED1_CURRENT_MAX,      1, LED1_CURRENT_130},
        {ADD_REG_LED2_CURRENT_MAX,      1, LED1_CURRENT_130},
        {ADD_REG_LED1_CURRENT_MIN,      1, LED1_CURRENT_80},
        {ADD_REG_LED2_CURRENT_MIN,      1, LED1_CURRENT_80},
    },
};
#endif



// start[1] | address[2] | function[2] | value[8] | LRC[2] | end[2]
static uint16_t convert_msg_to_buffer(mod_txMsgType *packet, uint8_t *bf)
{
    int idx = 0;
    uint32_t lrc;

    bf[idx++] = 0x3a;

    /* Slave Address : 2 */	
    btoah(&packet->address, bf+idx, 1);
    idx+=2;
    lrc = packet->address;

    /* Function Code : 2 */
    btoah(&packet->function, bf+idx, 1);
    idx+=2;
    lrc += packet->function;

    /* Start Register : 4 */
    /* Number or bytes or register value : 4 */
    uint8_t tmp[4];
    tmp[3] = packet->Value.u8[0];
    tmp[2] = packet->Value.u8[1];
    tmp[1] = packet->Value.u8[2];
    tmp[0] = packet->Value.u8[3];
    for(int i=0;i<4;i++)
    {
        lrc+=tmp[i];
    }

    btoah(tmp, bf+idx, 4);
    idx+=8;

    uint8_t chk[1];
    chk[0] = -(lrc & 0xff);
    btoah(chk, bf+idx, 1);
    idx+=2;

    bf[idx++] = 0x0d;
    bf[idx++] = 0x0a;
    bf[idx++] = '\0';       // This character is inserted that is not included protocol but for transmit.

    return idx;
}

void mod_1_trigger_ctrl(uint8_t onoff)
{
	uint32_t volatile i = 0;

	if( onoff == ON)
	{
		GPIO_WriteBit( FLUOSENS_TRIGGER_PG, FLUOSENS_TRIGGER_PIN, OFF);

		for( i = 0 ; i < 1000; i++)
		{
			;
		}
		
		GPIO_WriteBit( FLUOSENS_TRIGGER_PG, FLUOSENS_TRIGGER_PIN, ON);
	}
	else
	{	
		GPIO_WriteBit( FLUOSENS_TRIGGER_PG, FLUOSENS_TRIGGER_PIN, OFF);
	}
}

void mod_1_reset_ctrl(uint8_t onoff)
{
	uint32_t volatile i = 0;

	if( onoff == ON)
	{
		GPIO_WriteBit( FLUOSENS_RESET_PG, FLUOSENS_RESET_PIN, OFF);

		for( i = 0 ; i < 1000*500; i++)		// 1000
		{
			;
		}
		
		GPIO_WriteBit( FLUOSENS_RESET_PG, FLUOSENS_RESET_PIN, ON);
	}
	else
	{
		GPIO_WriteBit( FLUOSENS_RESET_PG, FLUOSENS_RESET_PIN, OFF);
	}	
}

void mod_2_trigger_ctrl(uint8_t onoff)
{
	uint32_t volatile i = 0;

	if( onoff == ON)
	{
		GPIO_WriteBit( GPIOA, GPIO_Pin_8, OFF);

		for( i = 0 ; i < 1000; i++)
		{
			;
		}
		
		GPIO_WriteBit( GPIOA, GPIO_Pin_8, ON);
	}
	else
	{
		GPIO_WriteBit( GPIOA, GPIO_Pin_8, OFF);
	}
	
}

void mod_2_reset_ctrl(uint8_t onoff)
{
	uint32_t volatile i = 0;

	if( onoff == ON)
	{
		GPIO_WriteBit( GPIOD, GPIO_Pin_2, OFF);

		for( i = 0 ; i < 1000*500; i++)		// 1000
		{
			;
		}
		
		GPIO_WriteBit( GPIOD, GPIO_Pin_2, ON);
	}
	else
	{
		GPIO_WriteBit( GPIOD, GPIO_Pin_2, OFF);
	}	
}

void mod_trigger_ctrl(uint8_t port, uint8_t onoff)
{
    if(port >1)
    {
        return;
    }

    if(port == MODBUS_PORT1)
    {
        mod_1_trigger_ctrl(onoff);
    }
    else if(port == MODBUS_PORT2)
    {
        mod_2_trigger_ctrl(onoff);
    }
    else
    {}
}

void mod_reset_ctrl(uint8_t port, uint8_t onoff)
{
    if(port >1)
    {
        return;
    }

    if(port == MODBUS_PORT1)
    {
        mod_1_reset_ctrl(onoff);
    }
    else if(port == MODBUS_PORT2)
    {
        mod_2_reset_ctrl(onoff);
    }
    else
    {}
}

void init_mod_hardware(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 

    // module 1
	/* /TRIGGER */
	GPIO_InitStructure.GPIO_Pin 		=  GPIO_Pin_10; 			
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_OD;	// GPIO_Mode_Out_PP GPIO_Mode_Out_OD
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* /RESET */
	GPIO_InitStructure.GPIO_Pin 		=  GPIO_Pin_4; 			
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_OD;  //  GPIO_Mode_Out_PP
	GPIO_Init(GPIOD, &GPIO_InitStructure);

    // mod_trigger_ctrl(ON);
    // mod_reset_ctrl(ON);

    // moudle 2

	/* /TRIGGER */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin 		=  GPIO_Pin_8; 			
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_OD;	// GPIO_Mode_Out_PP
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* /RESET */
	GPIO_InitStructure.GPIO_Pin 		=  GPIO_Pin_2; 			
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_OD;  //  GPIO_Mode_Out_PP
	GPIO_Init(GPIOD, &GPIO_InitStructure);

    // mod_2_trigger_ctrl(ON);
    // mod_2_reset_ctrl(ON);
}

uint8_t parsing_rx_msg(mod_rxMsgType *packet, uint8_t *bf, uint16_t bf_len)
{
    uint8_t success = FALSE;
    int idx = 1;
    uint32_t lrc = 0;
    int byteNum;

    byteNum = ahtob(&bf[idx]);
    idx+=2;

    if(byteNum >=0)
    {
        packet->address = byteNum;
        lrc += byteNum;
        success = TRUE;
    }
    else
    {
        success = FALSE;
    }

    if(success)
    {
        byteNum = ahtob(&bf[idx]);
        idx+=2;

        if(byteNum >=0)
        {
            packet->function = byteNum;
            lrc += byteNum;
        }
        else
        {
            success = FALSE;
        }
    }

    if(success)
    {
        byteNum = ahtob(&bf[idx]);
        idx+=2;

        if(byteNum >=0)
        {
            packet->data_len = byteNum;
            lrc += byteNum;
        }
        else
        {
            success = FALSE;
        }
    }
    
    if(success)
    {
        if((packet->data_len <= packet->data_maxLen) && ( (idx + (packet->data_len*2)+2)<=bf_len))
        {
            for(int i=0;i<packet->data_len;i++)
            {
                byteNum = ahtob(&bf[idx]);
                idx+=2;

                if(byteNum >=0)
                {
                    packet->data[i] = byteNum;
                    lrc += byteNum;
                }
                else
                {
                    success = FALSE;
                    break;
                }
            }
        }
        else
        {
            success = FALSE;
        }
    }
    
    if(success)
    {
        byteNum = ahtob(&bf[idx]);
        idx+=2;

        if(byteNum >=0)
        {
            lrc = (-lrc) & 0xff;
            
            if(lrc == byteNum)
            {
                success = TRUE;
            }
            else
            {
                success = FALSE;
            }
        }
        else
        {
            success = FALSE;
        }
    }

    return success;
}

uint8_t compare_tx_msg(uint8_t *tx, uint8_t tx_len, uint8_t *rx, uint8_t rx_len)
{
    uint8_t rslt = FALSE;

    for(int i=0;i<tx_len;i++)
    {
        if(i>=rx_len)
        {
            break;
        }

        if(tx[i] != rx[i])
        {
            break;
        }
    
        if(tx[i] == 0x0d)   // TODO: check message length
        {
            rslt = TRUE;
            break;
        }
    }

    return rslt;
}

void print_rxMsg(mod_rxMsgType *packet)
{
    printf("mod> rx= %d %d %d \n", packet->address, packet->function, packet->data_len);
}

void print_rxMsgData(mod_rxMsgType *packet)
{
    printf("%s\n", packet->data);
}

void print_rxPacket(stTrxData *packet)
{
    printf("mod> rxPacket= ");
    for(int i=0;i< packet->buf_idx; i++)
    {
        printf("%c", packet->buf[i]);
    }
    printf("\n");
}

static inline void mod_clear_recv_bytes(uint8_t port)
{
    if(port > 1)
    {
        return;
    }

	rxBuffer[port].buf_use_flag = FALSE;
	rxBuffer[port].buf_idx = 0;
}

void mod_send_msg(uint8_t port, mod_ManageType * mang)
{
    if(port > 1)
    {
        return;
    }

    txBuffer[port].buf_idx = convert_msg_to_buffer(&mang->txMsg, txBuffer[port].buf);
    mang->tx_time = get_time_ms_cnt();
    mang->sended_txMsag_flag = TRUE;
    mod_mang[port].tx_retry = 0;
    mod_clear_recv_bytes(port);     // TODO: test me!

    if(port == MODBUS_PORT1)
    {
        usart2_transmit_string((char*)txBuffer[port].buf); // @@@ 485, Optic Send string
        // printf("%s", (char*)txBuffer[port].buf);
    }
    else if(port == MODBUS_PORT2)
    {
        usart3_transmit_string((char*)txBuffer[port].buf);  // @@@ 485, Optic Send string
        // printf("%s", (char*)txBuffer[port].buf);
    }
    else
    {}

    mod_state[port] = MOD_ST_WAITING_RX;
}

void mod_send_retry(uint8_t port, mod_ManageType * mang)
{
    if(port > 1)
    {
        return;
    }

    if(port == MODBUS_PORT1)
    {
        usart2_transmit_string((char*)txBuffer[port].buf);
        mang->tx_retry++;
        mang->tx_time = get_time_ms_cnt();
        mang->sended_txMsag_flag = TRUE;
        printf("mod[%d]> Retry= %d\n", port, mang->tx_retry);
    }
    else if(port == MODBUS_PORT2)
    {
        usart3_transmit_string((char*)txBuffer[port].buf);
        mang->tx_retry++;
        mang->tx_time = get_time_ms_cnt();
        mang->sended_txMsag_flag = TRUE;
        printf("mod[%d]> Retry= %d\n", port, mang->tx_retry);
    }
    else
    {}
}

static inline void mod_get_msg(uint8_t port)
{
    if(port > 1)
    {
        return;
    }

    mod_mang[port].received_rxMsg_flag = TRUE;
    mod_mang[port].rx_time = get_time_ms_cnt();
}

void mod_recevie_msg(uint8_t port, uint8_t input)
{
    if(port > 1)
    {
        return;
    }

    // TODO: need block uart rx until process has been complete
    if(rxBuffer[port].buf_idx<TRX_BUF_MAX)
    {
        if(rxBuffer[port].buf_use_flag == TRUE)
        {
        	rxBuffer[port].buf[rxBuffer[port].buf_idx] = input;
            
            if( (input == 0x0A) && (rxBuffer[port].buf[rxBuffer[port].buf_idx -1] == 0x0D) )   /* CR(0x0D) LF(0x0A) =  end */
            {
               mod_get_msg(port);
//               mod_clear_recv_bytes(port);
            }
            else
            {
            	rxBuffer[port].buf_idx++;
            }
        }
        else
        {
            if(input == 0x3A) 	/* : = 0x3A  = start */
            {
                rxBuffer[port].buf_use_flag = TRUE;
                rxBuffer[port].buf[rxBuffer[port].buf_idx] = input;
                rxBuffer[port].buf_idx = 1;
            }
        }
    }
    else
    {
        rxBuffer[port].buf_use_flag = FALSE;
        rxBuffer[port].buf_idx = 0;
    }
}

uint8_t get_reg_defInfo(uint8_t port, uint16_t reg, uint32_t *val, uint8_t *len)
{
    uint8_t rslt = FALSE;

    if(port > 1)
    {
        return FALSE;
    }

    const Optic_settingType *setting = &q1_model[port][0];
    uint16_t cnt = sizeof(q1_model[0]) / sizeof(Optic_settingType);

    for(int i=0; i< cnt; i++ )
    {
        if(setting[i].startRegister == reg)
        {
            *len = setting[i].bytes;
            *val = setting[i].value;
            rslt = TRUE;
            break;
        }
    }
    
    return rslt;
}

void mod_msg_proc(uint8_t port)
{
	float tmp;
    // int32_t val;
    uint32_t data;
    uint8_t data_len;

    if(port > 1)
    {
        return;
    }

    if( (mod_mang[port].received_rxMsg_flag == TRUE) && (mod_mang[port].sended_txMsag_flag == TRUE) ) /* Sent & Received */
	{
        mod_mang[port].received_rxMsg_flag = FALSE;
        mod_mang[port].sended_txMsag_flag = FALSE;

        // uint32_t rxTime = get_time_ms_cnt();
        // printf("transive time= %dms\n", rxTime - mod_mang[port].tx_time);
        // printf("transive time= %dms\n", mod_mang[port].rx_time - mod_mang[port].tx_time);


        if(mod_mang[port].txMsg.function == FUNC_WRITING)
        {
            mod_mang[port].rxMsg.address = mod_mang[port].txMsg.address;
            mod_mang[port].rxMsg.function = mod_mang[port].txMsg.function;

            // print_rxPacket(&rxBuffer[port]);
            
            if(compare_tx_msg(txBuffer[port].buf, txBuffer[port].buf_idx, rxBuffer[port].buf, rxBuffer[port].buf_idx) == TRUE)
            {
                mod_state[port] = MOD_ST_RX_GOOD;
                // printf("mod[%d]> rx write ok\n", port);
            }
            else
            {
                // TODO: rx parsing fail. retry?
                mod_state[port] = MOD_ST_RX_BAD;
                printf("mod[%d]> rx write bad\n", port);
            }
        }
        else    // FUNC_READING
        {
            mod_mang[port].rxMsg.data = rxData[port];
            mod_mang[port].rxMsg.data_maxLen = TRX_BUF_MAX;

            // print_rxPacket(&rxBuffer[port]);

            if(parsing_rx_msg(&mod_mang[port].rxMsg, rxBuffer[port].buf, rxBuffer[port].buf_idx))
            {
                // print_rxMsg(&mod_mang[port].rxMsg);

                mod_state[port] = MOD_ST_RX_DONE;
                // check header
                if(mod_mang[port].txMsg.function == mod_mang[port].rxMsg.function)
                {
                    mod_mang[port].rxMsg.startRegister = mod_mang[port].txMsg.Value.startRegister;

                    switch(mod_mang[port].rxMsg.startRegister)
                    {
                        case ADD_REG_TEMPERATURE:
                            if(mod_mang[port].rxMsg.data_len == 4)
                            {
                                mod_mang[port].rxMsg.u32Data =  convertTo_uint32(mod_mang[port].rxMsg.data);

                                tmp = ((float)mod_mang[port].rxMsg.u32Data*OPT_MULT_FACTOR - 54.3f) / 0.205f;
                                printf("temp[%d]= %d.", port, (int)tmp);
                                printf("%s oC\n",sub_three(tmp));
                            }
                        break;
#if 0
                        case ADD_REG_LED1_CURRENT:
                            if(mod_mang[port].rxMsg.data_len == 2)
                            {
                                printf("led1 current= %d mA\n", mod_mang[port].rxMsg.data[0]);
                            }                            
                            break;

                        case ADD_REG_LED2_CURRENT:
                            if(mod_mang[port].rxMsg.data_len == 2)
                            {
                                printf("led2 current= %d mA\n", mod_mang[port].rxMsg.data[0]);
                            }                            
                            break;
#endif
                        case ADD_REG_ON_VALUE_1:
                        case ADD_REG_ON_VALUE_2:
                        case ADD_REG_ON_VALUE_3:
                        case ADD_REG_OFF_VALUE_1:
                        case ADD_REG_OFF_VALUE_2:
                        case ADD_REG_OFF_VALUE_3:
                            if(mod_mang[port].rxMsg.data_len == 4)
                            {
                                mod_mang[port].rxMsg.u32Data = convertTo_uint32(mod_mang[port].rxMsg.data);
                                // tmp = (float)mod_mang[port].rxMsg.u32Data*OPT_MULT_FACTOR;
                                // printf("mod> Meas %d= %d.", mod_mang[port].rxMsg.startRegister, (int)tmp);
                                // printf("%s mV\n",sub_three(tmp));
//                                printf("mod[%d]> Value %d= %d\n", port, mod_mang[port].rxMsg.startRegister, mod_mang[port].rxMsg.u32Data);

                                mod_state[port] = MOD_ST_RX_GOOD;
                            }

                        break;

                        case ADD_REG_TICKET:
                            if(mod_mang[port].rxMsg.data_len == 4)
                            {
                                mod_mang[port].rxMsg.u32Data = convertTo_uint32(mod_mang[port].rxMsg.data);                                        
                                // printf("mod[%d]> ticket= %d\n", port, mod_mang[port].rxMsg.u32Data);
                                mod_state[port] = MOD_ST_RX_GOOD;
                            }
                        break;

                        default:
                            if(get_reg_defInfo(port, mod_mang[port].rxMsg.startRegister, &data, &data_len) == TRUE)
                            {
                                if(data_len == 1)
                                {
                                    data = (data >> 8) & 0x00ff;
                                    mod_mang[port].rxMsg.u32Data = mod_mang[port].rxMsg.data[0];
                                }
                                else if(data_len == 2)
                                {
                                    mod_mang[port].rxMsg.u32Data = (mod_mang[port].rxMsg.data[0]<<8) | mod_mang[port].rxMsg.data[1];
                                }
                                else if(data_len == 4)
                                {
                                    mod_mang[port].rxMsg.u32Data = convertTo_uint32(mod_mang[port].rxMsg.data);
                                }
                                else
                                {}

                                if(data == mod_mang[port].rxMsg.u32Data)
                                {
                                    // printf("mod> rx ok\n");
                                    mod_state[port] = MOD_ST_RX_GOOD;
                                }
                                // printf("mod> reg= %d, data= %d\n", mod_mang[port].rxMsg.startRegister, mod_mang[port].rxMsg.u32Data);
                            }
                            else
                            {
                                printf("mod[%d]> unkown rx reg= %d\n", port, mod_mang[port].rxMsg.startRegister);
                                // TEST TEST
                                for(int i=0;i<mod_mang[port].rxMsg.data_len;i++)
                                {
                                    printf("0x%x ", mod_mang[port].rxMsg.data[i]);
                                }
                                printf("\n");
                            }
                        break;
                    }
                }
            }
            else
            {
                // TODO: rx parsing fail. retry?
                printf("mod[%d]> parsing fail\n", port);
                mod_state[port] = MOD_ST_RX_BAD;
            }
        }

        mod_clear_recv_bytes(port);
    }
    else if((mod_mang[port].sended_txMsag_flag == TRUE))
    {
        uint32_t time = get_time_ms_cnt() - mod_mang[port].tx_time;
        if(time >RETRY_WAIT_TIME_MS)
        {
            // printf("mod> retry\n");
            mod_clear_recv_bytes(port);
            if(mod_mang[port].tx_retry < 1)
            {
                mod_send_retry(port, &mod_mang[port]);
            }
            else
            {
                printf("mod[%d]> retry fail\n", port);
                // TODO: init retry variables
                mod_mang[port].sended_txMsag_flag = FALSE;
                mod_state[port] = MOD_ST_RX_TIMEOUT;
            }
        }
    }
    else
    {}
}


//------------------------------------------------------------------------------------
// Transive
//------------------------------------------------------------------------------------


void modbus_hal_test(void)
{
    mod_mang[0].txMsg.address = 0;
    mod_mang[0].txMsg.function = 3;
    // mod_mang[0].txMsg.Value.startRegister = 0x0102;
    mod_mang[0].txMsg.Value.startRegister = ADD_REG_AVERAGE;
    
    mod_mang[0].txMsg.Value.num_or_val = 0x0001;
    mod_send_msg(0, &mod_mang[0]);

}

void mod_send(uint8_t port, uint8_t func, uint16_t reg, uint16_t val)
{
    if(port>1)
    {
        return;
    }

    mod_mang[port].txMsg.address = 0;
    mod_mang[port].txMsg.function = func;
    mod_mang[port].txMsg.Value.startRegister = reg;
    mod_mang[port].txMsg.Value.num_or_val = val;

    if(port == MODBUS_PORT1)
    {
        mod_send_msg(MODBUS_PORT1, &mod_mang[port]);
    }
    else if(port == MODBUS_PORT2)
    {
        mod_send_msg(MODBUS_PORT2, &mod_mang[port]);
    }
    else
    {}
}

// return 0, or rx state
uint8_t mod_rx_waiting(uint8_t port)
{
    if(port > 1)
    {
        return FALSE;
    }

    if(mod_state[port] == MOD_ST_RX_GOOD)
    {
        mod_state[port] = MOD_ST_PROCESSING;
        return MOD_ST_RX_GOOD;
    }
    else if(mod_state[port] == MOD_ST_RX_DONE)
    {
        mod_state[port] = MOD_ST_PROCESSING;
        return MOD_ST_RX_DONE;
    }
    else if(mod_state[port] == MOD_ST_RX_BAD)
    {
        mod_state[port] = MOD_ST_PROCESSING;
        return MOD_ST_RX_BAD;
    }
    else if(mod_state[port] == MOD_ST_RX_TIMEOUT)
    {
        mod_state[port] = MOD_ST_PROCESSING;
        return MOD_ST_RX_TIMEOUT;
    }
    else
    {}

    return 0;
}

uint8_t mod_rx_idle(uint8_t port)
{
    if(port > 1)
    {
        return FALSE;
    }

    if(mod_state[port] == MOD_ST_PROCESSING)
    {
        return TRUE;
    }

    return FALSE;
}

uint8_t mod_rx_reset(uint8_t port)
{
    if(port > 1)
    {
        return FALSE;
    }

    mod_state[port] = MOD_ST_PROCESSING;
}

uint8_t mod_rx_state(uint8_t port)
{
    if(port > 1)
    {
        return FALSE;
    }

    return mod_state[port];
}

//------------------------------------------------------------------------------------
// Transive - modbus
//------------------------------------------------------------------------------------

extern stTransiveData send1_msg;

void modbus_pack_pc(uint8_t item_code, uint8_t sub_item_code, uint8_t* p_send_data, uint16_t data_len)
{
#if 0
	send1_msg.buf[0] = 0x3A;
	send1_msg.buf[1] = item_code;
	send1_msg.buf[2] = sub_item_code;
	send1_msg.buf[3] = data_len;
	
	memcpy( &send1_msg.buf[4], p_send_data, data_len);
	
	send1_msg.buf[ data_len + 4] = 0x0D;
	send1_msg.buf[ data_len + 5] = 0x0A;

	send1_msg.buf_idx = data_len + 6;
	send1_msg.buf_use_flag	= TRUE;
#else
	// direct transmit
	send1_msg.buf[0] = 0x3A;
	send1_msg.buf[1] = item_code;
	send1_msg.buf[2] = sub_item_code;
	send1_msg.buf[3] = data_len;
	
	memcpy( &send1_msg.buf[4], p_send_data, data_len);
	
	send1_msg.buf[ data_len + 4] = 0x0D;
	send1_msg.buf[ data_len + 5] = 0x0A;

	send1_msg.buf_idx = data_len + 6;
	usart1_transmit_data(send1_msg.buf, send1_msg.buf_idx);
#endif
}

//------------------------------------------------------------------------------------
// interface
//------------------------------------------------------------------------------------

uint32_t mod_get_rxMsg_data(uint8_t port)
{
    if(port > 1)
    {
        return FALSE;
    }

    return mod_mang[port].rxMsg.u32Data;
}

mod_rxMsgType * mod_get_rxMxg(uint8_t port)
{
    if(port > 1)
    {
        return FALSE;
    }

    return &mod_mang[port].rxMsg;
}

// TODO: reset variables
void mod_init_variables(void)
{
    mod_clear_recv_bytes(MODBUS_PORT1);
    mod_clear_recv_bytes(MODBUS_PORT2);
}

void mod_get_setting(uint8_t port, const Optic_settingType **setting, uint16_t *cnt)
{
    if(port > 1)
    {
        return;
    }

    if(port == MODBUS_PORT1)
    {
        q1_model[0][20].value = g_optic_current_HEX; //ADD_REG_LED1_CURRENT, LED1_CURRENT_64
        q1_model[0][21].value = g_optic_current_CY5; //ADD_REG_LED1_CURRENT, LED1_CURRENT_64
        
        *setting = &q1_model[0][0];
        *cnt = sizeof(q1_model[0]) / sizeof(Optic_settingType);
    }
    else if(port == MODBUS_PORT2)
    {
        q1_model[1][20].value = g_optic_current_FAM; //ADD_REG_LED1_CURRENT, LED1_CURRENT_64
        q1_model[1][21].value = g_optic_current_ROX; //ADD_REG_LED1_CURRENT, LED1_CURRENT_64
        *setting = &q1_model[1][0];
        *cnt = sizeof(q1_model[0]) / sizeof(Optic_settingType);
    }
    else
    {}
}

uint32_t Dec_to_Hex(int dec)
{
	uint32_t hex;
	switch(dec)
	{
		case 0:
			hex = 0x0000;
			break;
		case 1:
			hex = 0x0100;
			break;
		case 2:
			hex = 0x0200;
			break;
		case 3:
			hex = 0x0300;
			break;
		case 4:
			hex = 0x0400;
			break;
		case 5:
			hex = 0x0500;
			break;
		case 6:
			hex = 0x0600;
			break;
		case 7:
			hex = 0x0700;
			break;
		case 8:
			hex = 0x0800;
			break;
		case 9:
			hex = 0x0900;
			break;
		case 10:
			hex = 0x0A00;
			break;
		case 11:
			hex = 0x0B00;
			break;
		case 12:
			hex = 0x0C00;
			break;
		case 13:
			hex = 0x0D00;
			break;
		case 14:
			hex = 0x0E00;
			break;
		case 15:
			hex = 0x0F00;
			break;
		case 16:
			hex = 0x1000;
			break;
		case 17:
			hex = 0x1100;
			break;
		case 18:
			hex = 0x1200;
			break;
		case 19:
			hex = 0x1300;
			break;
		case 20:
			hex = 0x1400;
			break;
		case 21:
			hex = 0x1500;
			break;
		case 22:
			hex = 0x1600;
			break;
		case 23:
			hex = 0x1700;
			break;
		case 24:
			hex = 0x1800;
			break;
		case 25:
			hex = 0x1900;
			break;
		case 26:
			hex = 0x1A00;
			break;
		case 27:
			hex = 0x1B00;
			break;
		case 28:
			hex = 0x1C00;
			break;
		case 29:
			hex = 0x1D00;
			break;
		case 30:
			hex = 0x1E00;
			break;
		case 31:
			hex = 0x1F00;
			break;
		case 32:
			hex = 0x2000;
			break;
		case 33:
			hex = 0x2100;
			break;
		case 34:
			hex = 0x2200;
			break;
		case 35:
			hex = 0x2300;
			break;
		case 36:
			hex = 0x2400;
			break;
		case 37:
			hex = 0x2500;
			break;
		case 38:
			hex = 0x2600;
			break;
		case 39:
			hex = 0x2700;
			break;
		case 40:
			hex = 0x2800;
			break;
		case 41:
			hex = 0x2900;
			break;
		case 42:
			hex = 0x2A00;
			break;
		case 43:
			hex = 0x2B00;
			break;
		case 44:
			hex = 0x2C00;
			break;
		case 45:
			hex = 0x2D00;
			break;
		case 46:
			hex = 0x2E00;
			break;
		case 47:
			hex = 0x2F00;
			break;
		case 48:
			hex = 0x3000;
			break;
		case 49:
			hex = 0x3100;
			break;
		case 50:
			hex = 0x3200;
			break;
		case 51:
			hex = 0x3300;
			break;
		case 52:
			hex = 0x3400;
			break;
		case 53:
			hex = 0x3500;
			break;
		case 54:
			hex = 0x3600;
			break;
		case 55:
			hex = 0x3700;
			break;
		case 56:
			hex = 0x3800;
			break;
		case 57:
			hex = 0x3900;
			break;
		case 58:
			hex = 0x3A00;
			break;
		case 59:
			hex = 0x3B00;
			break;
		case 60:
			hex = 0x3C00;
			break;
		case 61:
			hex = 0x3D00;
			break;
		case 62:
			hex = 0x3E00;
			break;
		case 63:
			hex = 0x3F00;
			break;
		case 64:
			hex = 0x4000;
			break;
		case 65:
			hex = 0x4100;
			break;
		case 66:
			hex = 0x4200;
			break;
		case 67:
			hex = 0x4300;
			break;
		case 68:
			hex = 0x4400;
			break;
		case 69:
			hex = 0x4500;
			break;
		case 70:
			hex = 0x4600;
			break;
		case 71:
			hex = 0x4700;
			break;
		case 72:
			hex = 0x4800;
			break;
		case 73:
			hex = 0x4900;
			break;
		case 74:
			hex = 0x4A00;
			break;
		case 75:
			hex = 0x4B00;
			break;
		case 76:
			hex = 0x4C00;
			break;
		case 77:
			hex = 0x4D00;
			break;
		case 78:
			hex = 0x4E00;
			break;
		case 79:
			hex = 0x4F00;
			break;
		case 80:
			hex = 0x5000;
			break;
		case 81:
			hex = 0x5100;
			break;
		case 82:
			hex = 0x5200;
			break;
		case 83:
			hex = 0x5300;
			break;
		case 84:
			hex = 0x5400;
			break;
		case 85:
			hex = 0x5500;
			break;
		case 86:
			hex = 0x5600;
			break;
		case 87:
			hex = 0x5700;
			break;
		case 88:
			hex = 0x5800;
			break;
		case 89:
			hex = 0x5900;
			break;
		case 90:
			hex = 0x5A00;
			break;
		case 91:
			hex = 0x5B00;
			break;
		case 92:
			hex = 0x5C00;
			break;
		case 93:
			hex = 0x5D00;
			break;
		case 94:
			hex = 0x5E00;
			break;
		case 95:
			hex = 0x5F00;
			break;
		case 96:
			hex = 0x6000;
			break;
		case 97:
			hex = 0x6100;
			break;
		case 98:
			hex = 0x6200;
			break;
		case 99:
			hex = 0x6300;
			break;
		case 100:
			hex = 0x6400;
			break;
		case 101:
			hex = 0x6500;
			break;
		case 102:
			hex = 0x6600;
			break;
		case 103:
			hex = 0x6700;
			break;
		case 104:
			hex = 0x6800;
			break;
		case 105:
			hex = 0x6900;
			break;
		case 106:
			hex = 0x6A00;
			break;
		case 107:
			hex = 0x6B00;
			break;
		case 108:
			hex = 0x6C00;
			break;
		case 109:
			hex = 0x6D00;
			break;
		case 110:
			hex = 0x6E00;
			break;
		case 111:
			hex = 0x6F00;
			break;
		case 112:
			hex = 0x7000;
			break;
		case 113:
			hex = 0x7100;
			break;
		case 114:
			hex = 0x7200;
			break;
		case 115:
			hex = 0x7300;
			break;
		case 116:
			hex = 0x7400;
			break;
		case 117:
			hex = 0x7500;
			break;
		case 118:
			hex = 0x7600;
			break;
		case 119:
			hex = 0x7700;
			break;
		case 120:
			hex = 0x7800;
			break;
		case 121:
			hex = 0x7900;
			break;
		case 122:
			hex = 0x7A00;
			break;
		case 123:
			hex = 0x7B00;
			break;
		case 124:
			hex = 0x7C00;
			break;
		case 125:
			hex = 0x7D00;
			break;
		case 126:
			hex = 0x7E00;
			break;
		case 127:
			hex = 0x7F00;
			break;
		case 128:
			hex = 0x8000;
			break;
		case 129:
			hex = 0x8100;
			break;
		case 130:
			hex = 0x8200;
			break;
		case 131:
			hex = 0x8300;
			break;
		case 132:
			hex = 0x8400;
			break;
		case 133:
			hex = 0x8500;
			break;
		case 134:
			hex = 0x8600;
			break;
		case 135:
			hex = 0x8700;
			break;
		case 136:
			hex = 0x8800;
			break;
		case 137:
			hex = 0x8900;
			break;
		case 138:
			hex = 0x8A00;
			break;
		case 139:
			hex = 0x8B00;
			break;
		case 140:
			hex = 0x8C00;
			break;
		case 141:
			hex = 0x8D00;
			break;
		case 142:
			hex = 0x8E00;
			break;
		case 143:
			hex = 0x8F00;
			break;
		case 144:
			hex = 0x9000;
			break;
		case 145:
			hex = 0x9100;
			break;
		case 146:
			hex = 0x9200;
			break;
		case 147:
			hex = 0x9300;
			break;
		case 148:
			hex = 0x9400;
			break;
		case 149:
			hex = 0x9500;
			break;
		case 150:
			hex = 0x9600;
			break;
	}

	return hex;
}
