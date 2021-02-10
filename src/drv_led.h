/*
 * drv_led.h
 *
 *  Created on: 2019. 7. 1.
 *      Author: jk.choi
 */

#ifndef DRV_LED_H_
#define DRV_LED_H_
#include "stm3210b_eval.h"

#define TRUE 1
#define FALSE 0
 
 void drv_RGB_led_init(void);
void drv_led_init_on(void);
void drv_led_init_off(void);
void drv_led_green_on(void);
void drv_led_green_off(void);
void drv_led_red_on(void);
void drv_led_red_off(void);

#endif /* DRV_LED_H_ */
