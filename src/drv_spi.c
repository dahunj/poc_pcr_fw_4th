/*
 * drv_spi.c
 *
 *  Created on: 2019. 7. 29.
 *      Author: jk.choi
 */
#include  <stdio.h>
#include "stm32f10x.h"
#include <stm32f10x_spi.h>
#include "hw_config.h"
#include "drv_spi.h"

#define TRUE 1
#define FALSE 0




/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;





/*
void delay_x(u16 x)
{
	u16 i;
	for(i = 0; i < x; i++)
	{
		;
	}
}*/

void drv_spi_init(void) 		//cjk
{

	GPIO_InitTypeDef  GPIO_InitStructure; 
	SPI_InitTypeDef   SPI_InitStructure; 
 	NVIC_InitTypeDef   NVIC_InitStructure;
	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* Enable the SPI1 Pins Software Remapping */
	//GPIO_PinRemapConfig(GPIO_Remap_SPI1, ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); 

	//GPIO_PinRemapConfig(GPIO_Remap_SPI1, DISABLE);
	
#if 1
	/* SPI1 SCK(PA5)、MISO(PA6)、MOSI(PA7) */
	//GPIO_InitStructure.GPIO_Pin 		= GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	
	GPIO_InitStructure.GPIO_Pin 		=  /*GPIO_Pin_4 |*/GPIO_Pin_5 | /*GPIO_Pin_6 |*/GPIO_Pin_7 ; 	/* PA4 : NSS,   PA5: SCK,  PA6: MISO */
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  //  GPIO_InitStructure.Pull = GPIO_NoPull;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin 		=  GPIO_Pin_10; 			
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_PP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin 		=  GPIO_Pin_0 | GPIO_Pin_1; 			
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IPU;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	/* SPI1 init*/ 
	SPI_InitStructure.SPI_Direction 		= SPI_Direction_2Lines_FullDuplex; //; //SPI_Direction_1Line_Rx;    //SPI_Direction_2Lines_FullDuplex; //?
	SPI_InitStructure.SPI_Mode 		= SPI_Mode_Master;					
	SPI_InitStructure.SPI_DataSize 		= SPI_DataSize_8b; //SPI_DataSize_16b;	//SPI_DataSize_8b;				
	SPI_InitStructure.SPI_CPOL 		= SPI_CPOL_Low;						
	SPI_InitStructure.SPI_CPHA 		= SPI_CPHA_1Edge; //SPI_CPHA_1Edge;					

	/* NSS Soft */
	SPI_InitStructure.SPI_NSS 			= SPI_NSS_Soft;	
	//SPI_InitStructure.SPI_NSS 			= SPI_NSS_Hard;		
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_InitStructure.SPI_FirstBit 		= SPI_FirstBit_MSB;				   
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);
	
	#if 1
	/* SPI1 NSS */
	GPIO_InitStructure.GPIO_Pin 			= GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_Out_PP;
	//GPIO_InitStructure.GPIO_Speed 		= GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//GPIO_SetBits(GPIOA, GPIO_Pin_15);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
	#endif
#else
	/**SPI1 GPIO Configuration    
	PA4     ------> SPI1_NSS
	PA5     ------> SPI1_SCK
	PA6     ------> SPI1_MISO 
	*/
	GPIO_InitStructure.GPIO_Pin = /*GPIO_Pin_4 | */GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	//  GPIO_InitStructure.Pull = GPIO_NoPull;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* SPI1 init*/ 
	SPI_InitStructure.SPI_Direction 		= SPI_Direction_2Lines_RxOnly; //SPI_Direction_2Lines_FullDuplex; //; //SPI_Direction_1Line_Rx;    //; //?
	SPI_InitStructure.SPI_Mode 		= SPI_Mode_Master;					
	SPI_InitStructure.SPI_DataSize 		= SPI_DataSize_16b;	//SPI_DataSize_8b;				
	SPI_InitStructure.SPI_CPOL 		= SPI_CPOL_Low;						
	SPI_InitStructure.SPI_CPHA 		= SPI_CPHA_1Edge;					

	/* NSS Soft */
	SPI_InitStructure.SPI_NSS 			= SPI_NSS_Soft;	
	//SPI_InitStructure.SPI_NSS 			= SPI_NSS_Hard;		
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_InitStructure.SPI_FirstBit 		= SPI_FirstBit_MSB;				   
	SPI_InitStructure.SPI_CRCPolynomial = 10;//7;
	SPI_Init(SPI1, &SPI_InitStructure);		

#endif



#if 1//

	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
		
    	/* NVIC Configuration */
	/* Enable the TIM4_IRQn Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 		= 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd 				= ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
#endif


	/* SPI1 Enable */  
	SPI_Cmd(SPI1,ENABLE);

}





void bsp_touch_spi_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	SPI_InitTypeDef   SPI_InitStructure; 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE); 
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);


	
	/* SPI1 SCK(PA5)、MISO(PA6)、MOSI(PA7) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | /*GPIO_Pin_14 |*/ GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  //  GPIO_InitStructure.Pull = GPIO_NoPull;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin 		=  GPIO_Pin_11; 			
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin 		=  GPIO_Pin_2; 			
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN_FLOATING; //GPIO_Mode_IPU;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin 		=  GPIO_Pin_8; 			
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN_FLOATING; //GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	#if 0
	// SPI1 NSS
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // cjk
	GPIO_Init(GPIOC, &GPIO_InitStructure);
//	GPIO_SetBits(GPIOC, GPIO_Pin_10);
	#endif
	
	/* SPI1 init*/ 
	SPI_InitStructure.SPI_Direction 		= SPI_Direction_2Lines_FullDuplex; //SPI_Direction_1Line_Rx;  //;
	SPI_InitStructure.SPI_Mode 		= SPI_Mode_Master;					
	SPI_InitStructure.SPI_DataSize 		= SPI_DataSize_8b;				
	SPI_InitStructure.SPI_CPOL 		= SPI_CPOL_High; //SPI_CPOL_Low;						
	SPI_InitStructure.SPI_CPHA 		= SPI_CPHA_2Edge;					
	// NSS Soft
	SPI_InitStructure.SPI_NSS 			= SPI_NSS_Soft; //SPI_NSS_Hard; //;					
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_InitStructure.SPI_FirstBit 		= SPI_FirstBit_MSB;				   
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);
	
	/* SPI1 Enable */  
	SPI_Cmd(SPI2,ENABLE);

}

void bsp_touch_gpio_init_polling(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// Touch interrupt
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

uint8_t drv_spi_receive_data(uint32_t* pbuf)
{
	uint8_t i;
	uint8_t j;
	//uint32_t tmp;
	uint16_t tmp;



	GPIOA->BRR |= GPIO_Pin_4;
	
	
	
	while( (SPI1->SR & SPI_I2S_FLAG_RXNE) == RESET)
	{
	}
	tmp = SPI1->DR;
	GPIOA->BSRR |= GPIO_Pin_4;


#if 0
	//tmp = (uint32_t)SPI1->DR;
	tmp = SPI1->DR;
	for(i = 32, j = 0; i > 0; i--, j++)
	{
		*(pbuf + j) = tmp << i;
	}
	

	for(i = 0; i < 100; i++){}

	while( (SPI1->SR & SPI_I2S_FLAG_RXNE) == RESET)
	{
	}

	//tmp = (uint32_t)SPI1->DR;
	tmp = SPI1->DR;
	for(i = 16, j = 16; i > 0; i--, j++)
	{
		*(pbuf + j) = tmp << i;
	}
#endif

}



// SPI speed setting function
// SpeedSet:
// SPI_SPEED_2 2 frequency (SPI 36M @ sys 72M)
// SPI_SPEED_4 4 frequency (SPI 18M @ sys 72M)
// SPI_SPEED_8 8 frequency (SPI 9M @ sys 72M)
// SPI_SPEED_16 16 frequency (SPI 4.5M @ sys 72M)
// SPI_SPEED_256 256 frequency (SPI 281.25K @ sys 72M)

//void bsp_set_spi1_speed (u8 speed)
void drv_set_spi1_speed (u8 speed)
{
	SPI1->CR1 &= 0XFFC7; // Fsck = Fcpu/256
	switch (speed)
	{
		case SPI_SPEED_2: // Second division
			SPI1->CR1 |= 0<<3; // Fsck = Fpclk / 2 = 36Mhz
			break;
		case SPI_SPEED_4: // four-band
			SPI1-> CR1 |= 1<<3; // Fsck = Fpclk / 4 = 18Mhz
			break;
		case SPI_SPEED_8: // eighth of the frequency
			SPI1-> CR1 |= 2<<3; // Fsck = Fpclk / 8 = 9Mhz
			break;
		case SPI_SPEED_16: // sixteen frequency
			SPI1-> CR1 |= 3<<3; // Fsck = Fpclk/16 = 4.5Mhz
			break;
		case SPI_SPEED_256: // 256 frequency division
			SPI1-> CR1 |= 7<<3; // Fsck = Fpclk/16 = 281.25Khz
			break;
	}

	SPI1->CR1 |= 1<<6; // SPI devices enable

}

/*******************************************************************************
* Function Name: bsp_readwritebyte_spi1
* Description: SPI read and write a single byte (to return after sending the data read in this Newsletter)
* Input: u8 TxData the number to be sent
* Output: None
* Return: u8 RxData the number of received
*******************************************************************************/
//u8 bsp_readwritebyte_spi1 (u8 tx_data)
uint8_t drv_readwritebyte_spi1 (u8 tx_data)
{
	u8 retry=0;
	u16 d16 = 0;
	u16 d16_2 = 0;
	u32 d32 = 0;
	
#if 1//cjk

	//u8 retry=0;

	/* Loop while DR register in not emplty */
	while (SPI_I2S_GetFlagStatus (SPI1, SPI_I2S_FLAG_TXE) == RESET)
	{
		retry++;
		if(retry>400)
			return 0;
	}
//	SPI_SSOutputCmd(SPI1, ENABLE);
	/* Send byte through the SPI1 peripheral */
	SPI_I2S_SendData (SPI1, tx_data);
	
//	SPI_SSOutputCmd(SPI1, DISABLE);
	retry=0;

	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus (SPI1, SPI_I2S_FLAG_RXNE) == RESET)
	{
		retry++;

		if(retry>400)
			return 0;
	}

	/* Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData (SPI1);


	
#else
	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus (SPI1, SPI_I2S_FLAG_RXNE) == RESET)
	{
		retry++;

		if(retry>400)
			return 0;
	}
	delay(500);
	d16 = SPI_I2S_ReceiveData (SPI1);
	

//	d16_2 = SPI_I2S_ReceiveData (SPI1);
	
	
//	d32 = (u32)d16 << 16;
//	d32 |= (u32)d16_2;
		
	while (SPI_I2S_GetFlagStatus (SPI1, SPI_I2S_FLAG_RXNE) == RESET)
	{
		retry++;

		if(retry>400)
			return 0;
	}
	delay(1000);
	d16_2= SPI_I2S_ReceiveData (SPI1);
	
	d32 = (u32)d16 << 16;
	d32 |= (u32)d16_2;

	/* Return the byte read from the SPI bus */
	//return SPI_I2S_ReceiveData (SPI1);
	
	//return 1;
#endif		
}


// SPI speed setting function
// SpeedSet:
// SPI_SPEED_2 2 frequency (SPI 36M @ sys 72M)
// SPI_SPEED_4 4 frequency (SPI 18M @ sys 72M)
// SPI_SPEED_8 8 frequency (SPI 9M @ sys 72M)
// SPI_SPEED_16 16 frequency (SPI 4.5M @ sys 72M)
// SPI_SPEED_256 256 frequency (SPI 281.25K @ sys 72M)
//void bsp_set_spi2_speed (u8 speed)
void drv_set_spi2_speed (u8 speed)
{
	SPI2->CR1 &= 0XFFC7; // Fsck = Fcpu/256
	switch (speed)
	{
		case SPI_SPEED_2: // Second division
			SPI2->CR1 |= 0<<3; // Fsck = Fpclk / 2 = 36Mhz
			break;
		case SPI_SPEED_4: // four-band
			SPI2-> CR1 |= 1<<3; // Fsck = Fpclk / 4 = 18Mhz
			break;
		case SPI_SPEED_8: // eighth of the frequency
			SPI2-> CR1 |= 2<<3; // Fsck = Fpclk / 8 = 9Mhz
			break;
		case SPI_SPEED_16: // sixteen frequency
			SPI2-> CR1 |= 3<<3; // Fsck = Fpclk/16 = 4.5Mhz
			break;
		case SPI_SPEED_256: // 256 frequency division
			SPI2-> CR1 |= 7<<3; // Fsck = Fpclk/16 = 281.25Khz
			break;
	}

	SPI2->CR1 |= 1<<6; // SPI devices enable

}

//void bsp_set_spi2_speed_mp3(u8 SpeedSet)
void drv_set_spi2_speed_mp3(u8 SpeedSet)
{
    SPI_InitTypeDef SPI_InitStructure ;

    SPI_InitStructure.SPI_Direction=SPI_Direction_2Lines_FullDuplex ;
    SPI_InitStructure.SPI_Mode=SPI_Mode_Master ;
    SPI_InitStructure.SPI_DataSize=SPI_DataSize_8b ;
    SPI_InitStructure.SPI_CPOL=SPI_CPOL_High ;
    SPI_InitStructure.SPI_CPHA=SPI_CPHA_2Edge ;
    SPI_InitStructure.SPI_NSS=SPI_NSS_Soft ;

    if(SpeedSet==SPI_SPEED_LOW)
    {
        SPI_InitStructure.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_16;
    }
    else
    {
        SPI_InitStructure.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_4;
    }

	//moon.mp3: 4707774 Byte size buffer[512]
	//speed: 392314 Byte/S��
	//Prescaler_128, 59592 Byte/S
	//Prescaler_64, 104617 Byte/S
	//Prescaler_32, 168134 Byte/S    162337 Byte/S
	//Prescaler_16, 261543 Byte/S    247777 Byte/S
	//Prescaler_8,  313851 Byte/S    336269 Byte/S
	//Prescaler_4,  392314 Byte/S    392314 Byte/S
	//Prescaler_2,  392314 Byte/S

    SPI_InitStructure.SPI_FirstBit=SPI_FirstBit_MSB ;
    SPI_InitStructure.SPI_CRCPolynomial=7 ;
    SPI_Init(SPI2,&SPI_InitStructure);
}

/*******************************************************************************
* Function Name: bsp_readwritebyte_spi2
* Description: SPI read and write a single byte (to return after sending the data read in this Newsletter)
* Input: u8 TxData the number to be sent
* Output: None
* Return: u8 RxData the number of received
*******************************************************************************/
//u8 bsp_readwritebyte_spi2 (u8 tx_data)
uint8_t drv_readwritebyte_spi2 (u8 tx_data)
{
	u16 retry=0;

	/* Loop while DR register in not emplty */
	while (SPI_I2S_GetFlagStatus (SPI2, SPI_I2S_FLAG_TXE) == RESET)
	{
		retry++;
		if(retry>400)
			return 0;
	}
	retry=0;
	
	/* Send byte through the SPI2 peripheral */
	SPI_I2S_SendData (SPI2, tx_data);

	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus (SPI2, SPI_I2S_FLAG_RXNE) == RESET)
	{
		retry++;

		if(retry>400)
			return 0;
	}

	/* Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData (SPI2);
}




void SPI1_IRQHandler(void) 	/* cjk  0809 */
{
	static uint16_t recv_u16_data = 0;
	static uint8_t cnt = 0;
	
	if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) != RESET)
	{
		SPI_I2S_ClearITPendingBit(SPI1, SPI_I2S_IT_RXNE);
		recv_u16_data = SPI_I2S_ReceiveData(SPI1);
		
		cnt++;
		if( (cnt % 2) == 0)
		{
			spi_data.spi_temp_u32 |= ((uint32_t)recv_u16_data) << 16;
			spi_data.spi_temp_flag = TRUE;
			cnt = 0;
			recv_u16_data = 0;
		}
		else
		{
			spi_data.spi_temp_u32 |= (uint32_t)recv_u16_data;
		}
		
		
	}
}




u8 bsp_readwritebyte_spi2 (u8 tx_data)
{
	u8 retry=0;
	static u8 d8 = 0, i;
	u16 d16 = 0;
	u32 d32 = 0;

#if 1
	/* Loop while DR register in not emplty */
	while (SPI_I2S_GetFlagStatus (SPI2, SPI_I2S_FLAG_TXE) == RESET)
	{
		retry++;
		if(retry>200)
			return 0;
	}

	/* Send byte through the SPI2 peripheral */
	SPI_I2S_SendData (SPI2, tx_data);

	retry=0;
#endif

#if 1
	/* Wait to receive a byte */
	for(i = 0 ; i < 2; i++)
	{
		while (SPI_I2S_GetFlagStatus (SPI2, SPI_I2S_FLAG_RXNE) == RESET)
		{
			retry++;
			
			if(retry>200)
				return 0;
		}		

		/* Return the byte read from the SPI bus */
		
		d8 = SPI_I2S_ReceiveData (SPI2);
		
		if( i == 0)
		{
			d32 = (u32)d8 << 8;
		}
		else
		{
			d32 |= (u32)d8;
		}

		
	}
	
	return 1;
	//return SPI_I2S_ReceiveData (SPI2);
#else
	GPIO_WriteBit( GPIOC, GPIO_Pin_10, 0);

	for(i=0; i< 255; i++)
	{}
	
    	for (i = 31; i >= 0; i--) 
	{
		GPIO_WriteBit( GPIOC, GPIO_Pin_10, 0);

		/* Wait to receive a byte */
		while (SPI_I2S_GetFlagStatus (SPI2, SPI_I2S_FLAG_RXNE) == RESET)
		{
			retry++;
		
			if(retry>200)
				return 0;
		}
		
		d8 <<= 1;
		d8 |= SPI_I2S_ReceiveData (SPI2);
		GPIO_WriteBit( GPIOC, GPIO_Pin_10, 1);

		for(i=0; i< 255; i++)
		{}
    	}
#endif

}




void MX_SPI1_Init(void)
{

	  /* USER CODE BEGIN SPI1_Init 0 */

	  /* USER CODE END SPI1_Init 0 */

	  /* USER CODE BEGIN SPI1_Init 1 */

	  /* USER CODE END SPI1_Init 1 */
	  /* SPI1 parameter configuration*/
	  hspi1.Instance = SPI1;
	  hspi1.Init.Mode = SPI_MODE_MASTER;
	  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	  hspi1.Init.NSS = SPI_NSS_SOFT;
	  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	  hspi1.Init.CRCPolynomial = 10;
	  
	  if (HAL_SPI_Init(&hspi1) != HAL_OK)
	  {
	    	;//Error_Handler();
	  }
}


 HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi)
{
	  /* Check the SPI handle allocation */
	  if(hspi == NULL)
	  {
	    return HAL_ERROR;
	  }

	  /* Check the parameters */
	 // assert_param(IS_SPI_ALL_INSTANCE(hspi->Instance));
	  assert_param(IS_SPI_MODE(hspi->Init.Mode));
	  assert_param(IS_SPI_DIRECTION(hspi->Init.Direction));
	  assert_param(IS_SPI_DATASIZE(hspi->Init.DataSize));
	  assert_param(IS_SPI_CPOL(hspi->Init.CLKPolarity));
	  assert_param(IS_SPI_CPHA(hspi->Init.CLKPhase));
	  assert_param(IS_SPI_NSS(hspi->Init.NSS));
	  assert_param(IS_SPI_BAUDRATE_PRESCALER(hspi->Init.BaudRatePrescaler));
	  assert_param(IS_SPI_FIRST_BIT(hspi->Init.FirstBit));

#if (USE_SPI_CRC != 0U)
	  assert_param(IS_SPI_CRC_CALCULATION(hspi->Init.CRCCalculation));
	  if(hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE)
	  {
	    assert_param(IS_SPI_CRC_POLYNOMIAL(hspi->Init.CRCPolynomial));
	  }
#else
	  hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
#endif /* USE_SPI_CRC */

	  if(hspi->State == HAL_SPI_STATE_RESET)
	  {
	    /* Init the low level hardware : GPIO, CLOCK, NVIC... */
	    HAL_SPI_MspInit(hspi);
	  }
  
	  hspi->State = HAL_SPI_STATE_BUSY;

	  /* Disble the selected SPI peripheral */
	  __HAL_SPI_DISABLE(hspi);

	  /*----------------------- SPIx CR1 & CR2 Configuration ---------------------*/
	  /* Configure : SPI Mode, Communication Mode, Data size, Clock polarity and phase, NSS management,
	  Communication speed, First bit and CRC calculation state */
	  WRITE_REG(hspi->Instance->CR1, (hspi->Init.Mode | hspi->Init.Direction | hspi->Init.DataSize |
	                                  hspi->Init.CLKPolarity | hspi->Init.CLKPhase | (hspi->Init.NSS & SPI_CR1_SSM) |
	                                  hspi->Init.BaudRatePrescaler | hspi->Init.FirstBit  | hspi->Init.CRCCalculation) );

	  /* Configure : NSS management */
	  WRITE_REG(hspi->Instance->CR2, (((hspi->Init.NSS >> 16U) & SPI_CR2_SSOE) | hspi->Init.TIMode));

	  /*---------------------------- SPIx CRCPOLY Configuration ------------------*/
	  /* Configure : CRC Polynomial */
	  WRITE_REG(hspi->Instance->CRCPR, hspi->Init.CRCPolynomial);

#if defined(SPI_I2SCFGR_I2SMOD)
	  /* Activate the SPI mode (Make sure that I2SMOD bit in I2SCFGR register is reset) */
	  CLEAR_BIT(hspi->Instance->I2SCFGR, SPI_I2SCFGR_I2SMOD);
#endif /* SPI_I2SCFGR_I2SMOD */

#if (USE_SPI_CRC != 0U)
#if defined (STM32F101xE) || defined (STM32F103xE)
	  /* Check RevisionID value for identifying if Device is Rev Z (0x0001) in order to enable workaround for
	     CRC errors wrongly detected */
	  /* Pb is that ES_STM32F10xxCDE also identify an issue in Debug registers access while not in Debug mode.
	     Revision ID information is only available in Debug mode, so Workaround could not be implemented
	     to distinguish Rev Z devices (issue present) from more recent version (issue fixed).
	     So, in case of Revison Z F101 or F103 devices, below variable should be assigned to 1 */
	  uCRCErrorWorkaroundCheck = 0U;
#else
	  uCRCErrorWorkaroundCheck = 0U;
#endif /* STM32F101xE || STM32F103xE */
#endif /* USE_SPI_CRC */

	  hspi->ErrorCode = HAL_SPI_ERROR_NONE;
	  hspi->State = HAL_SPI_STATE_READY;
	  
	  return HAL_OK;
}



void HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef_ *GPIO_Init)
{
	  uint32_t position;
	  uint32_t ioposition = 0x00U;
	  uint32_t iocurrent = 0x00U;
	  uint32_t temp = 0x00U;
	  uint32_t config = 0x00U;
	  __IO uint32_t *configregister; /* Store the address of CRL or CRH register based on pin number */
	  uint32_t registeroffset = 0U; /* offset used during computation of CNF and MODE bits placement inside CRL or CRH register */

	  /* Check the parameters */
	  //assert_param(IS_GPIO_ALL_INSTANCE(GPIOx));
	  assert_param(IS_GPIO_PIN(GPIO_Init->Pin));
	  assert_param(IS_GPIO_MODE(GPIO_Init->Mode));

	  /* Configure the port pins */
	  for (position = 0U; position < GPIO_NUMBER; position++)
	  {
	    /* Get the IO position */
	    ioposition = (0x01U << position);

	    /* Get the current IO position */
	    iocurrent = (uint32_t)(GPIO_Init->Pin) & ioposition;

	    if (iocurrent == ioposition)
	    {
	      /* Check the Alternate function parameters */
	      assert_param(IS_GPIO_AF_INSTANCE(GPIOx));

	      /* Based on the required mode, filling config variable with MODEy[1:0] and CNFy[3:2] corresponding bits */
	      switch (GPIO_Init->Mode)
	      {
	        /* If we are configuring the pin in OUTPUT push-pull mode */
	        case GPIO_MODE_OUTPUT_PP:
	          /* Check the GPIO speed parameter */
	          assert_param(IS_GPIO_SPEED(GPIO_Init->Speed));
	          config = GPIO_Init->Speed + GPIO_CR_CNF_GP_OUTPUT_PP;
	          break;

	        /* If we are configuring the pin in OUTPUT open-drain mode */
	        case GPIO_MODE_OUTPUT_OD:
	          /* Check the GPIO speed parameter */
	          assert_param(IS_GPIO_SPEED(GPIO_Init->Speed));
	          config = GPIO_Init->Speed + GPIO_CR_CNF_GP_OUTPUT_OD;
	          break;

	        /* If we are configuring the pin in ALTERNATE FUNCTION push-pull mode */
	        case GPIO_MODE_AF_PP:
	          /* Check the GPIO speed parameter */
	          assert_param(IS_GPIO_SPEED(GPIO_Init->Speed));
	          config = GPIO_Init->Speed + GPIO_CR_CNF_AF_OUTPUT_PP;
	          break;

	        /* If we are configuring the pin in ALTERNATE FUNCTION open-drain mode */
	        case GPIO_MODE_AF_OD:
	          /* Check the GPIO speed parameter */
	          assert_param(IS_GPIO_SPEED(GPIO_Init->Speed));
	          config = GPIO_Init->Speed + GPIO_CR_CNF_AF_OUTPUT_OD;
	          break;

	        /* If we are configuring the pin in INPUT (also applicable to EVENT and IT mode) */
	        case GPIO_MODE_INPUT:
	        case GPIO_MODE_IT_RISING:
	        case GPIO_MODE_IT_FALLING:
	        case GPIO_MODE_IT_RISING_FALLING:
	        case GPIO_MODE_EVT_RISING:
	        case GPIO_MODE_EVT_FALLING:
	        case GPIO_MODE_EVT_RISING_FALLING:
	          /* Check the GPIO pull parameter */
	          assert_param(IS_GPIO_PULL(GPIO_Init->Pull));
	          if (GPIO_Init->Pull == GPIO_NOPULL)
	          {
	            config = GPIO_CR_MODE_INPUT + GPIO_CR_CNF_INPUT_FLOATING;
	          }
	          else if (GPIO_Init->Pull == GPIO_PULLUP)
	          {
	            config = GPIO_CR_MODE_INPUT + GPIO_CR_CNF_INPUT_PU_PD;

	            /* Set the corresponding ODR bit */
	            GPIOx->BSRR = ioposition;
	          }
	          else /* GPIO_PULLDOWN */
	          {
	            config = GPIO_CR_MODE_INPUT + GPIO_CR_CNF_INPUT_PU_PD;

	            /* Reset the corresponding ODR bit */
	            GPIOx->BRR = ioposition;
	          }
	          break;

	        /* If we are configuring the pin in INPUT analog mode */
	        case GPIO_MODE_ANALOG:
	          config = GPIO_CR_MODE_INPUT + GPIO_CR_CNF_ANALOG;
	          break;

	        /* Parameters are checked with assert_param */
	        default:
	          break;
	      }

	      /* Check if the current bit belongs to first half or last half of the pin count number
	       in order to address CRH or CRL register*/
	      configregister = (iocurrent < GPIO_PIN_8) ? &GPIOx->CRL     : &GPIOx->CRH;
	      registeroffset = (iocurrent < GPIO_PIN_8) ? (position << 2U) : ((position - 8U) << 2U);

	      /* Apply the new configuration of the pin to the register */
	      MODIFY_REG((*configregister), ((GPIO_CRL_MODE0 | GPIO_CRL_CNF0) << registeroffset), (config << registeroffset));

	      /*--------------------- EXTI Mode Configuration ------------------------*/
	      /* Configure the External Interrupt or event for the current IO */
	      if ((GPIO_Init->Mode & EXTI_MODE) == EXTI_MODE)
	      {
	        /* Enable AFIO Clock */
	        __HAL_RCC_AFIO_CLK_ENABLE();
	        temp = AFIO->EXTICR[position >> 2U];
	        CLEAR_BIT(temp, (0x0FU) << (4U * (position & 0x03U)));
	        SET_BIT(temp, (GPIO_GET_INDEX(GPIOx)) << (4U * (position & 0x03U)));
	        AFIO->EXTICR[position >> 2U] = temp;


	        /* Configure the interrupt mask */
	        if ((GPIO_Init->Mode & GPIO_MODE_IT) == GPIO_MODE_IT)
	        {
	          SET_BIT(EXTI->IMR, iocurrent);
	        }
	        else
	        {
	          CLEAR_BIT(EXTI->IMR, iocurrent);
	        }

	        /* Configure the event mask */
	        if ((GPIO_Init->Mode & GPIO_MODE_EVT) == GPIO_MODE_EVT)
	        {
	          SET_BIT(EXTI->EMR, iocurrent);
	        }
	        else
	        {
	          CLEAR_BIT(EXTI->EMR, iocurrent);
	        }

	        /* Enable or disable the rising trigger */
	        if ((GPIO_Init->Mode & RISING_EDGE) == RISING_EDGE)
	        {
	          SET_BIT(EXTI->RTSR, iocurrent);
	        }
	        else
	        {
	          CLEAR_BIT(EXTI->RTSR, iocurrent);
	        }

	        /* Enable or disable the falling trigger */
	        if ((GPIO_Init->Mode & FALLING_EDGE) == FALLING_EDGE)
	        {
	          SET_BIT(EXTI->FTSR, iocurrent);
	        }
	        else
	        {
	          CLEAR_BIT(EXTI->FTSR, iocurrent);
	        }
      	}
    }
  }
}





/**
* @brief SPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
	  GPIO_InitTypeDef_ GPIO_InitStruct = {0};
	  GPIO_InitTypeDef GPIO_InitStructure;
	  if(hspi->Instance==SPI1)
	  {
	  /* USER CODE BEGIN SPI1_MspInit 0 */

	  /* USER CODE END SPI1_MspInit 0 */
	    /* Peripheral clock enable */
	    __HAL_RCC_SPI1_CLK_ENABLE();
	  
	    __HAL_RCC_GPIOA_CLK_ENABLE();
	    /**SPI1 GPIO Configuration    
	    PA5     ------> SPI1_SCK
	    PA6     ------> SPI1_MISO
	    PA7     ------> SPI1_MOSI 
	    */
	    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	    GPIO_InitStruct.Pin = GPIO_PIN_6;
	    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /* USER CODE BEGIN SPI1_MspInit 1 */
	  GPIO_InitStructure.GPIO_Pin 		=  GPIO_Pin_10; 			
	  GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_PP;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);
	  /* USER CODE END SPI1_MspInit 1 */
  }

}

