
/*
 * drv_limit_sensor.h
 *
 *  Created on: 2019. 7. 25.
 *      Author: jk.choi
 */

#ifndef DRV_LIMIT_SENSOR_H_
#define DRV_LIMIT_SENSOR_H_

#include "hw_config.h"
#include "task.h"
#include <stm32f10x.h>

typedef enum
{
	DETATCHED = 0,
	CONTACTED = 1
}e_Limit_Sens_Stat;

e_Limit_Sens_Stat lim_sens_stat_sm1;
e_Limit_Sens_Stat lim_sens_stat_sm2;
e_Limit_Sens_Stat lim_sens_stat_sm3;
e_Limit_Sens_Stat lim_sens_stat_sm4;
e_Limit_Sens_Stat lim_sens_stat_sm5;
e_Limit_Sens_Stat lim_sens_stat_om;
e_Limit_Sens_Stat tube_sensor;
e_Limit_Sens_Stat basket_sensor;

/* 0402 door safety sensor code test */
typedef enum
{
	OPEN = 1,
	CLOSE = 0
}Door_Sens_Stat;

Door_Sens_Stat door_up_sensor_flag;

void drv_limit_sensor_init(void);
void drv_limit_sensor_gpio_init(void);
void drv_limit_sensor_sm1_init(void);
void drv_limit_sensor_sm2_init(void);
void drv_limit_sensor_sm3_init(void);
void drv_limit_sensor_sm4_init(void);
void drv_limit_sensor_sm5_init(void);
void drv_limit_sensor_om_init(void);
void drv_limit_sensor_modify_interrupt(uint8_t id_motor, uint8_t  rot_dir);
void drv_limit_sensor_reset_interrupt(uint8_t id_motor);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

void drv_door_sensor_up_init(void);
void drv_tube_sensor_init(void);
void drv_basket_sensor_init(void);
void drv_zero_position_tune(void);


#endif /* DRV_LIMIT_SENSOR_H_ */
