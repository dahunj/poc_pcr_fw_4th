#pragma once

#include "stm32f10x.h"
#include "drv_usart.h"
#include "drv_timer.h"
// #include "modbus_hal.h"
#include <stdio.h>
#include <string.h>
#include "Glob_Var_IF.h"
#define OPTIC_SAVE_EN   0

#define BASE_MEAS_MAX 3
#define CHAM_MAX 4

#define BASE_OPT_MEA 1
#define OPT_MEA 2
#define CYCLE_OPT_MEA 3

#define BASE_MEA_ROUTINE_NO	0//7
#define MEA_ROUTINE_NO	g_ROUTINE_CYCLE_MAX

typedef enum OPTIC_ST{
    OPTIC_ST_RESET =0,
    OPTIC_ST_INIT_READ,
    OPTIC_ST_INIT_WRITE,
    OPTIC_ST_INIT_SAVE,
    OPTIC_ST_READ_ALL,
    OPTIC_ST_NORMAL,
}OPTIC_ST_E;

typedef enum QIA_JOB{
    QIA_JOB_RUN = 0,
    QIA_JOB_WAIT,
    QIA_JOB_DONE,
}QIA_JOB_E;

typedef enum QIA_MEAS_STEP{
    QIA_MEAS_NONE = 0,
    QIA_MEAS_START,
    QIA_MEAS_CHCK,
    QIA_MEAS_STOP,
    QIA_MEAS_GET,
}QIA_MEAS_STEP_E;

typedef enum QIA_ED_MODE{
    QIA_E1D1_MDOE = 1,
    QIA_E1D2_MDOE = 2,
    QIA_E2D2_MDOE = 3,
}QIA_ED_MODE_E;

typedef void(*optic_FuncType)(uint8_t port);

typedef struct qia_mang{
    OPTIC_ST_E optic_state;
    QIA_JOB_E job_state;
    QIA_MEAS_STEP_E meas_state;
    uint32_t job_time_since;
    
    uint8_t reset_save_flag; // TODO: flag for save a register, Not implemented yet
    uint8_t port;
    uint16_t init_idx;
    uint8_t init_done_flag;

    uint8_t first_ticket_flag;
    uint32_t ticket;

    uint8_t meas_value_idx;
    uint32_t e1d1_onOff_raw[2];
    uint32_t e2d2_onOff_raw[2];
    uint32_t e1d1_meas;
    uint32_t e2d2_meas;

    optic_FuncType end_callback;

    const Optic_settingType *setting;
    uint16_t setting_cnt;
}qia_mangType;



void optic_qiagen_request_measure(uint8_t port, optic_FuncType callback);
void optic_qiagen_get_measure(uint8_t port, uint32_t *e1d1_val, uint32_t *e2d2_val);
float optic_qiagen_convert_raw_to_mV(uint32_t raw_input);
void optic_qiagen_reset(void);

void optic_qiagen_test(void);
void optic_qiagen_msg_process(void);
void optic_qiagen_init(void);
uint8_t optic_qiagen_check_msg(void);
void optic_qiagen_send_msg(uint8_t port, uint8_t msg_id, uint8_t ed_mode, uint8_t chamber);
void optic_qiagen_read_all(uint8_t port);

