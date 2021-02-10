#pragma once

typedef enum QIA_TASK_STEP{
    QIA_TASK_READY = 0,
    QIA_TASK_DELAY,
    QIA_TASK_START,
    QIA_TASK_DONE,
    QIA_TASK_NONE,
}QIA_TASK_STEP_E;

typedef struct qia_task{
    QIA_TASK_STEP_E task_step;
    qia_mangType * opt_mang;
    uint32_t ready_time_tick;

    // base measure values
    uint32_t base_e1d1_meas[CHAM_MAX][BASE_MEAS_MAX];
    uint32_t base_e2d2_meas[CHAM_MAX][BASE_MEAS_MAX];
    uint8_t base_e1d1_meas_idx[CHAM_MAX];
    uint8_t base_e2d2_meas_idx[CHAM_MAX];

    // measure value
    uint32_t last_e1d1_meas[CHAM_MAX];
    uint32_t last_e2d2_meas[CHAM_MAX];

    // cycle measure values //200728 test
    uint32_t cycle_e1d1_meas[CHAM_MAX];
    uint32_t cycle_e2d2_meas[CHAM_MAX];
}qia_taskType;

void optic_qiagen_task_init(void);
uint8_t optic_qiagen_measure_process(uint8_t port, uint32_t chamber, uint16_t delay_ms);
void optic_qiagen_measure_ready(void);

void optic_cycle_data_init(void);	//200728 test

