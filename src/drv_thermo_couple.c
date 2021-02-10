/*
 * drv_thermo_couple.c
 *
 *  Created on: 2019. 7. 31.
 *      Author: jk.choi
 */
#include "drv_thermo_couple.h"


#define THERMO_ON 0
#define THERMO_OFF 1

#define TRUE 1
#define FALSE 0

void drv_thermo_couple_init(void)
{
	//drv_spi_init();
	bsp_touch_spi_init();
}


void drv_thermo_couple_on(void)
{
	//GPIO_WriteBit( GPIOA, GPIO_Pin_15, THERMO_ON); 	/* NSS : active low */
	GPIO_WriteBit( GPIOA, GPIO_Pin_4, THERMO_ON); 	/* NSS : active low */
}


void drv_thermo_couple_off(void)
{
	//GPIO_WriteBit( GPIOA, GPIO_Pin_15, THERMO_OFF); 	/* NSS : active low */
	GPIO_WriteBit( GPIOA, GPIO_Pin_4, THERMO_OFF); 	/* NSS : active low */
}


//uint16_t  drv_thermo_couple_get_data(void)
uint32_t  drv_thermo_couple_get_data(void)
{
	uint32_t ret_data32;
	
	drv_thermo_couple_on();
	
	while(spi_data.spi_temp_flag  != TRUE)
	{
		;
	}

        ret_data32 =  spi_data.spi_temp_u32;
	spi_data.spi_temp_flag = FALSE;
	spi_data.spi_temp_u32 = 0;
	
	return  ret_data32;
}


