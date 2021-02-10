#pragma once


#include "stm32f10x.h"
#include "drv_usart.h"
#include "drv_timer.h"
#include <stdio.h>
#include "util.h"

#ifndef TRUE
#define TRUE 1
#define FALSE 0
#endif

#define TRX_BUF_MAX 64
// #define RX_BUF_MAX 128

#define TRANSIVE_BUF_MAX	200 // 100 <-- GD. 2020.03.30.

typedef struct
{
	uint8_t buf[TRANSIVE_BUF_MAX];
	uint8_t buf_idx; 
	uint8_t buf_use_flag;
}stTransiveData;

typedef struct{
    uint8_t address;
    uint8_t function;
    union{
        uint32_t u32;
        uint8_t  u8[4];
        struct{
            uint16_t num_or_val;
            uint16_t startRegister;
        };
    }Value;
}mod_txMsgType;

typedef struct{
    uint8_t address;
    uint8_t function;
    uint8_t data_len;
    uint8_t data_maxLen;
    uint8_t *data;

    // optional value
    uint16_t startRegister;
    uint32_t u32Data;
}mod_rxMsgType;

typedef struct
{
	uint8_t buf[TRX_BUF_MAX];
	uint8_t buf_idx;
	uint8_t buf_use_flag;
}stTrxData;

typedef struct{
    uint16_t startRegister;
    uint8_t  bytes;
    uint16_t value;
}Optic_settingType;

typedef struct{
    mod_txMsgType txMsg;
    uint8_t sended_txMsag_flag;
    uint32_t tx_time;
    uint8_t tx_retry;
    
    mod_rxMsgType rxMsg;
    uint8_t received_rxMsg_flag;
    uint32_t rx_time;

}mod_ManageType;

#define MODBUS_PORT1 0
#define MODBUS_PORT2 1

#define OPT_MULT_FACTOR 0.000298026f

#define FUNC_READING		0x03
#define FUNC_WRITING		0x06

#define ADD_REG_CYCLES			0
#define ADD_REG_CYCLE_TIME		1
#define ADD_REG_START_MODE		2
#define ADD_REG_METHOD_TYPE	3
#define ADD_REG_DARK_SIGNAL_TYPE	4
#define ADD_REG_AVERAGE		5
#define ADD_REG_LED_MODE		6
#define ADD_REG_TRIGGER_DELAY	7
#define ADD_REG_E1D1_FACTOR	8
#define ADD_REG_E1D2_FACTOR	10
#define ADD_REG_E2D2_FACTOR	12
#define ADD_REG_E1D1_OFFSET		14
#define ADD_REG_E1D2_OFFSET		16
#define ADD_REG_E2D2_OFFSET		18
#define ADD_REG_ON_DELAY_LED1	20
#define ADD_REG_ON_DELAY_LED2	21
#define ADD_REG_OFF_DELAY_LED1	22
#define ADD_REG_OFF_DELAY_LED2	23
#define ADD_REG_LED1_CURRENT	24
#define ADD_REG_LED2_CURRENT	25
#define ADD_REG_LED1_CURRENT_DEFAULT	26
#define ADD_REG_LED2_CURRENT_DEFAULT	27
#define ADD_REG_LED1_CURRENT_MAX		28
#define ADD_REG_LED2_CURRENT_MAX		29
#define ADD_REG_LED1_CURRENT_MIN		30
#define ADD_REG_LED2_CURRENT_MIN		31
#define ADD_REG_ADC_SAMPLING		32
#define ADD_REG_BOARD_NAME		128
#define ADD_REG_BOARD_SERIAL_NUMBER	144
#define ADD_REG_BOARD_ID				148
#define ADD_REG_HARDWARE_REVISION		156
#define ADD_REG_OPTIC_REVISION		160
#define ADD_REG_BOARD_TYPE			164
#define ADD_REG_MODBUS_ADDRESS	165
#define ADD_REG_BAUDRATE		166
#define ADD_REG_TICKET			256
#define ADD_REG_TEMPERATURE	258
#define ADD_REG_ON_VALUE_1		260
#define ADD_REG_ON_VALUE_2		262
#define ADD_REG_ON_VALUE_3		264
#define ADD_REG_OFF_VALUE_1		266
#define ADD_REG_OFF_VALUE_2		268
#define ADD_REG_OFF_VALUE_3		270
#define ADD_REG_SOFTWARE_REVISION			384
#define ADD_REG_USER_DEFINED_VALUE_1		400
#define ADD_REG_USER_DEFINED_VALUE_2		401
#define ADD_REG_USER_DEFINED_VALUE_111		510
#define ADD_REG_USER_DEFINED_VALUE_112		511

/* Write Access */
#define ADD_REG_START_METHOD_COMMAND		512
#define ADD_REG_STOP_METHOD_COMMAND		513
#define ADD_REG_LED1_ONOFF_COMMAND		514
#define ADD_REG_LED2_ONOFF_COMMAND		515
#define ADD_REG_START_AUTOZERO_COMMAND	516
#define ADD_REG_SAVE_ACTUAL_OR_DEFAULT_PARAMETERS		517
#define ADD_REG_SAVE_DEFAULT_PARAMETERS_TO_NVRAM		518
#define ADD_REG_SETUP_ADC		519

/* Read Access */
#define ADD_REG_NUMBER_OF_SAVED_DATAPOINTS	512
#define ADD_REG_DATAPOINTS_1		513
#define ADD_REG_DATAPOINTS_2		515
#define ADD_REG_DATAPOINTS_1500	3513

#define ADD_REG_TEMPERATURE     0x0102      // 258


//-----------------------------------------------------------------

#define FLUOSENS_RESET_PIN		GPIO_Pin_4
#define FLUOSENS_TRIGGER_PIN	GPIO_Pin_10
#define FLUOSENS_RESET_PG		GPIOD
#define FLUOSENS_TRIGGER_PG	GPIOD


#define METHOD_TYPE_1_E1D1			0x0100
#define METHOD_TYPE_2_E1D2			0x0200
#define METHOD_TYPE_3_E2D2			0x0300
#define METHOD_TYPE_4_E1D1_E1D2		0x0400
#define METHOD_TYPE_5_E1D1_E2D2		0x0500
#define METHOD_TYPE_6_E1D2_E2D2		0x0600
#define METHOD_TYPE_7_E1D1_E1D2_E2D2	0x0700
#define METHOD_TYPE_8_S_E1D1			0x0800
#define METHOD_TYPE_9_S_E1D2			0x0900
#define METHOD_TYPE_10_S_E2D2			0x0A00

#define STARTMODE_0_CMD				0x0000
#define STARTMODE_1_TRG_SNG			0x0100
#define STARTMODE_2_TRG_MEA			0x0200
#define STARTMODE_3_AUTO_CMD		0x0300
#define STARTMODE_4_AUTO_TRG_SNG		0x0400
#define STARTMODE_5_AUTO_TRG_MEA		0x0500

#define DARK_SIGNAL_TYPE_0_NO_DARK		0x0000
#define DARK_SIGNAL_TYPE_1_DARK_ONCE		0x0100
#define DARK_SIGNAL_TYPE_2_DARK_EVERY		0x0200

#define AVERAGE_1		0x0100
#define AVERAGE_2		0x0200
#define AVERAGE_3		0x0300
#define AVERAGE_4		0x0400
#define AVERAGE_5		0x0500

#define LED_MODE_0_TOGGLE			0x0000
#define LED_MODE_1_MANUAL		0x0100


#define FC_OPT_VAL			0xFA
#define FC_OPT_BASE_VAL		0xBB
#define FC_TMP_VAL			0xCA

#define SFC_OPT_TEST 			0x09
/* Final data address */
#define SFC_OPT_VAL_1CH_TB	 	0x1B
#define SFC_OPT_VAL_1CH_IC	 	0x1C
#define SFC_OPT_VAL_2CH_NTM	    0x2B
#define SFC_OPT_VAL_2CH_IC	 	0x2C

#define SFC_OPT_VAL_1CH_E1D1	SFC_OPT_VAL_1CH_TB
#define SFC_OPT_VAL_1CH_E2D2	SFC_OPT_VAL_1CH_IC
#define SFC_OPT_VAL_2CH_E1D1	SFC_OPT_VAL_2CH_NTM
#define SFC_OPT_VAL_2CH_E2D2	SFC_OPT_VAL_2CH_IC
#define SFC_OPT_VAL_3CH_E1D1 	0x5B
#define SFC_OPT_VAL_3CH_E2D2 	0x5C
#define SFC_OPT_VAL_4CH_E1D1    0x6B
#define SFC_OPT_VAL_4CH_E2D2 	0x6C

#define SFC_OPT2_VAL_1CH_E1D1	0x1E
#define SFC_OPT2_VAL_1CH_E2D2	0x1F
#define SFC_OPT2_VAL_2CH_E1D1	0x2E
#define SFC_OPT2_VAL_2CH_E2D2	0x2F
#define SFC_OPT2_VAL_3CH_E1D1 	0x3E
#define SFC_OPT2_VAL_3CH_E2D2 	0x3F
#define SFC_OPT2_VAL_4CH_E1D1   0x4E
#define SFC_OPT2_VAL_4CH_E2D2 	0x4F

/* Add 1209 Base data address */
#define SFC_OPT_BASE_1CH_TB	    0x3B
#define SFC_OPT_BASE_1CH_IC	    0x3C
#define SFC_OPT_BASE_2CH_NTM	0x4B
#define SFC_OPT_BASE_2CH_IC	    0x4C

#define SFC_OPT_BASE_1CH_E1D1   SFC_OPT_BASE_1CH_TB
#define SFC_OPT_BASE_1CH_E2D2   SFC_OPT_BASE_1CH_IC
#define SFC_OPT_BASE_2CH_E1D1	SFC_OPT_BASE_2CH_NTM
#define SFC_OPT_BASE_2CH_E2D2	SFC_OPT_BASE_2CH_IC
#define SFC_OPT_BASE_3CH_E1D1   0x7B
#define SFC_OPT_BASE_3CH_E2D2   0x7C
#define SFC_OPT_BASE_4CH_E1D1	0x8B
#define SFC_OPT_BASE_4CH_E2D2   0x8C

#define SFC_OPT2_BASE_1CH_E1D1  0x5E
#define SFC_OPT2_BASE_1CH_E2D2  0x5F
#define SFC_OPT2_BASE_2CH_E1D1  0x6E
#define SFC_OPT2_BASE_2CH_E2D2  0x6F
#define SFC_OPT2_BASE_3CH_E1D1  0x7E
#define SFC_OPT2_BASE_3CH_E2D2  0x7F
#define SFC_OPT2_BASE_4CH_E1D1  0x8E
#define SFC_OPT2_BASE_4CH_E2D2  0x8F


#define THRESHOLD_1CH_TB		10000
#define THRESHOLD_1CH_IC		10000
#define THRESHOLD_2CH_NTM	10000
#define THRESHOLD_2CH_IC		10000

#define LED1_CURRENT_250	0xFA00
#define LED1_CURRENT_200	0xC800
#define LED1_CURRENT_150	0x9600
#define LED1_CURRENT_130	0x8200
#define LED1_CURRENT_125	0x7D00
#define LED1_CURRENT_120	0x7800
#define LED1_CURRENT_115	0x7300
#define LED1_CURRENT_110	0x6E00
#define LED1_CURRENT_100	0x6400
#define LED1_CURRENT_90		0x5A00
#define LED1_CURRENT_80		0x5000
#define LED1_CURRENT_70		0x4600
#define LED1_CURRENT_64		0x4000
#define LED1_CURRENT_60		0x3C00
#define LED1_CURRENT_50		0x3200
#define LED1_CURRENT_40		0x2800
#define LED1_CURRENT_34		0x2200
#define LED1_CURRENT_32		0x2000
#define LED1_CURRENT_30		0x1E00
#define LED1_CURRENT_29		0x1D00
#define LED1_CURRENT_28		0x1C00
#define LED1_CURRENT_27		0x1B00
#define LED1_CURRENT_26		0x1A00
#define LED1_CURRENT_25		0x1900
#define LED1_CURRENT_24		0x1800
#define LED1_CURRENT_23		0x1700
#define LED1_CURRENT_22		0x1600
#define LED1_CURRENT_20		0x1400
#define LED1_CURRENT_15		0x0F00
#define LED1_CURRENT_14		0x0E00
#define LED1_CURRENT_13		0x0D00
#define LED1_CURRENT_10		0x0A00

#define LED2_CURRENT_250	0xFA00
#define LED2_CURRENT_200	0xC800
#define LED2_CURRENT_150	0x9600
#define LED2_CURRENT_130	0x8200
#define LED2_CURRENT_120	0x7800
#define LED2_CURRENT_115	0x7300
#define LED2_CURRENT_110	0x6E00
#define LED2_CURRENT_109	0x6D00
#define LED2_CURRENT_108	0x6C00
#define LED2_CURRENT_107	0x6B00
#define LED2_CURRENT_106	0x6A00
#define LED2_CURRENT_105	0x6900
#define LED2_CURRENT_100	0x6400
#define LED2_CURRENT_97		0x6100
#define LED2_CURRENT_95		0x5F00
#define LED2_CURRENT_90		0x5A00
#define LED2_CURRENT_88		0x5800
#define LED2_CURRENT_87		0x5700
#define LED2_CURRENT_85		0x5500
#define LED2_CURRENT_80		0x5000
#define LED2_CURRENT_79		0x4F00
#define LED2_CURRENT_78		0x4E00
#define LED2_CURRENT_77		0x4D00
#define LED2_CURRENT_76		0x4C00
#define LED2_CURRENT_75		0x4B00
#define LED2_CURRENT_74		0x4A00
#define LED2_CURRENT_73		0x4900
#define LED2_CURRENT_72		0x4800
#define LED2_CURRENT_71		0x4700
#define LED2_CURRENT_70		0x4600
#define LED2_CURRENT_69		0x4500
#define LED2_CURRENT_68		0x4400
#define LED2_CURRENT_65		0x4100
#define LED2_CURRENT_64		0x4000
#define LED2_CURRENT_60		0x3C00
#define LED2_CURRENT_50		0x3200
#define LED2_CURRENT_40		0x2800
#define LED2_CURRENT_34		0x2200
#define LED2_CURRENT_32		0x2000
#define LED2_CURRENT_30		0x1E00
#define LED2_CURRENT_29		0x1D00
#define LED2_CURRENT_28		0x1C00
#define LED2_CURRENT_27		0x1B00
#define LED2_CURRENT_26		0x1A00
#define LED2_CURRENT_25		0x1900
#define LED2_CURRENT_24		0x1800
#define LED2_CURRENT_23		0x1700
#define LED2_CURRENT_22		0x1600
#define LED2_CURRENT_20		0x1400
#define LED2_CURRENT_15		0x0F00
#define LED2_CURRENT_14		0x0E00
#define LED2_CURRENT_13		0x0D00
#define LED2_CURRENT_10		0x0A00

typedef enum
{
    MOD_ST_PROCESSING = 0,
    MOD_ST_WAITING_RX,
    MOD_ST_RX_DONE,
    MOD_ST_RX_GOOD,
    MOD_ST_RX_BAD,
    MOD_ST_RX_TIMEOUT,
    MOD_ST_MAX
}MOD_COMM_ST_E;


void init_mod_hardware(void);
void mod_trigger_ctrl(uint8_t port, uint8_t onoff);
void mod_reset_ctrl(uint8_t port, uint8_t onoff);

void mod_recevie_msg(uint8_t port, uint8_t input);
void modbus_hal_test(void);
void mod_msg_proc(uint8_t port);
void mod_send(uint8_t port, uint8_t func, uint16_t reg, uint16_t val);
void modbus_pack_pc(uint8_t item_code, uint8_t sub_item_code, uint8_t* p_send_data, uint16_t data_len);

uint8_t mod_rx_waiting(uint8_t port);
uint8_t mod_rx_state(uint8_t port);

uint32_t mod_get_rxMsg_data(uint8_t port);
mod_rxMsgType * mod_get_rxMxg(uint8_t port);
void mod_get_setting(uint8_t port, const Optic_settingType **setting, uint16_t *cnt);
