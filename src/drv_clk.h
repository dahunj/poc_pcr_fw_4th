/*
 * drv_clk.h
 *
 *  Created on: 2019. 7. 1.
 *      Author: jk.choi
 */

#ifndef DRV_CLK_H_
#define DRV_CLK_H_

#if 1
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"
#include "stdio.h"
#include "misc.h"
#else

#include "include.h"
#endif

void SetSysClockTo72(void);
void drv_clk_port_init(void);

#endif /* DRV_CLK_H_ */
