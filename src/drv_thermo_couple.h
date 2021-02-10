/*
 * drv_thermo_couple.h
 *
 *  Created on: 2019. 7. 31.
 *      Author: jk.choi
 */

#ifndef DRV_THERMO_COUPLE_H_
#define DRV_THERMO_COUPLE_H_

#include  "stm32f10x_gpio.h"
#include "drv_spi.h"

void drv_thermo_couple_init(void);
void drv_thermo_couple_on(void);
void drv_thermo_couple_off(void);
uint32_t  drv_thermo_couple_get_data(void);


#endif /* DRV_THERMO_COUPLE_H_ */
