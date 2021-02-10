/*
 * drv_motor.h
 *
 *  Created on: 2019. 7. 16.
 *      Author: jk.choi
 */

#ifndef DRV_MOTOR_H_
#define DRV_MOTOR_H_

#include "stm32f10x.h"
#include "drv_pwm.h"
#include "Drv_limit_sensor.h"


#define STEP_MOTOR_1		1
#define STEP_MOTOR_2		2
#define STEP_MOTOR_3		3
#define STEP_MOTOR_4		4
#define STEP_MOTOR_5		5
#define ORIENTAL_MOTOR        6   

#define DIR_CC	0
#define DIR_CCW	1

#define MOTOR_ENABLE	0
#define MOTOR_DISABLE	1

#define	CONFIG_PWM_SMOOTH		((uint16_t)0x0000)
#define CONFIG_PWM_FIXED		((uint16_t)0x0001)

void drv_stepmotor_port_init(void);
void drv_stepmotor_init(void);
void drv_stepmotor_output_pulse(uint8_t id_sm, uint8_t rot_dir, uint16_t step_count, uint16_t  pulse_period, uint32_t config);
void drv_stepmotor_output_enable(uint8_t id_sm,  uint8_t val);
void drv_stepmotor_output_direction(uint8_t id_sm, uint8_t rot_dir);
void drv_orientalmotor_output_pulse(uint8_t id_sm, uint8_t rot_dir, uint16_t step_count, uint16_t  pulse_period); 
void drv_orientalmotor_output_enable(uint8_t id_sm,  uint8_t val);  
void drv_orientalmotor_output_direction(uint8_t id_sm, uint8_t rot_dir);
uint8_t drv_stepmotor_check_sensor(uint8_t id_sm);
void drv_om_serch_zero_position(void);
void drv_stepmotor1_zero_position(void); 
void drv_stepmotor2_zero_position(void); 
void drv_stepmotor3_zero_position(void); 
void drv_stepmotor4_zero_position(void);
void drv_stepmotor5_zero_position(void);
void drv_motor_zero_position(void);

void door_lock_open(uint8_t state);		/* 200707 add */

#endif /* DRV_MOTOR_H_ */
