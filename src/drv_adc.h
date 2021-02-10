/*
 * drv_adc.h
 *
 *  Created on: 2019. 7. 29.
 *      Author: jk.choi
 */

#ifndef DRV_ADC_H_
#define DRV_ADC_H_

#include "stm32f10x_adc.h"


#define ADC12_IN8_PORT  	GPIOB
#define ADC12_IN8_PIN	 	GPIO_Pin_0

#define ADC12_IN9_PORT  	GPIOB
#define ADC12_IN9_PIN	 	GPIO_Pin_1

#define ADC12_IN15_PORT  	GPIOC
#define ADC12_IN15_PIN	 	GPIO_Pin_5

//void bsp_adc_gpio_init(uint8_t ADC_Channel);
//void bsp_adc_init(uint8_t ADC_Channel);

void drv_adc_gpio_init(uint8_t ADC_Channel);
void drv_adc_init(uint8_t ADC_Channel);
void ADC1_peltier_check(void);

#endif /* DRV_ADC_H_ */
