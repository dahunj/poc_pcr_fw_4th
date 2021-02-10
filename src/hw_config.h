/*
 * hw_config.h
 *
 *  Created on: 2019. 6. 20.
 *      Author: jk.choi
 */

#ifndef HW_CONFIG_H_
#define HW_CONFIG_H_

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"
#include "stdio.h"
#include "misc.h"

#include "task.h"
#include "drv_clk.h"
#include "drv_timer.h"
#include "drv_led.h"
#include "drv_motor.h"
#include "drv_limit_sensor.h"
#include "drv_adc.h"
#include "drv_spi.h"
#include "drv_usart.h"
#include "drv_i2c.h"

#include "modbus_hal.h"


#if 0
void timer4_hw_init(void);
void SetSysClockTo72(void);
void TIM4_IRQHandler(void);
#endif

void hw_init(void);
void drv_init(void);
void drv_motor_init(void);
void variable_init(void);
void drv_enable(void);
void drv_st_enable(void);




#endif /* HW_CONFIG_H_ */
