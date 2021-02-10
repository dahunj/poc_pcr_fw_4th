/*
 * drv_adc.c
 *
 *  Created on: 2019. 7. 29.
 *      Author: jk.choi
 */

#include "hw_config.h"
#include "drv_adc.h"

extern uint8_t self_check_flag;

//void bsp_adc_gpio_init(uint8_t ADC_Channel) 
void drv_adc_gpio_init(uint8_t ADC_Channel) 
{
	GPIO_InitTypeDef GPIO_InitStructure;						 				
	
	if( ADC_Channel == ADC_Channel_8 )
	{
		GPIO_InitStructure.GPIO_Pin = ADC12_IN8_PIN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(ADC12_IN8_PORT, &GPIO_InitStructure);	
	}
		
	
	if( ADC_Channel == ADC_Channel_9 )
	{
		GPIO_InitStructure.GPIO_Pin = ADC12_IN9_PIN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(ADC12_IN9_PORT, &GPIO_InitStructure);		
	}
	
	if( ADC_Channel == ADC_Channel_15 )
	{
		GPIO_InitStructure.GPIO_Pin = ADC12_IN15_PIN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(ADC12_IN15_PORT, &GPIO_InitStructure);
	}
}

//void bsp_adc_init(uint8_t ADC_Channel) 
void drv_adc_init(uint8_t ADC_Channel) 
{
	ADC_InitTypeDef ADC_InitStructure;
	
	//bsp_adc_gpio_init(ADC_Channel);
	drv_adc_gpio_init(ADC_Channel);
	
	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channel15 configuration */ 
	if( ADC_Channel == ADC_Channel_8 )
	{
		ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_55Cycles5);
	}
		
	
	if( ADC_Channel == ADC_Channel_9 )
	{
		ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_55Cycles5);	
	}
	
	if( ADC_Channel == ADC_Channel_15 )
	{
		ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_55Cycles5);
	}  
  
	ADC_Cmd(ADC1, ENABLE);
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
}

void ADC1_peltier_check(void)
{
	uint16_t value;

	drv_adc_init(ADC_Channel_8);
	GPIO_WriteBit(GPIOC, GPIO_Pin_9, 1);	//test module power enable
	for(int i = 0; i < 10 ; i++)
	{
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);
		while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

		value = ADC_GetConversionValue(ADC1);
		ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

		printf("Current low value = %d\n", value);
		for(int i = 0; i < 500000; i++)
		{

		}
	}
	GPIO_WriteBit(GPIOC, GPIO_Pin_9, 0);	//test module power disable
	printf("Test finish\n");

	self_check_flag = FALSE;
}

