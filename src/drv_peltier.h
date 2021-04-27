/*
 * drv_peltier.h
 *
 *  Created on: 2019. 7. 31.
 *      Author: jk.choi
 */

#ifndef DRV_PELTIER_H_
#define DRV_PELTIER_H_

#include  "stm32f10x_gpio.h"

void drv_peltier_init(void);
void drv_peltier_heating(uint8_t ctrl_onoff);
void drv_peltier_cooling(uint8_t ctrl_onoff);
void drv_peltier_fan_blow(uint8_t fan_ctrl);
void drv_peltier_stop(void);

void drv_peltier_dir(uint8_t dir_onoff);
void drv_inside_LED(uint8_t on_off);
void drv_water_fan(uint8_t on_off);
void drv_UV_LED(uint8_t on_off);

#endif /* DRV_PELTIER_H_ */
