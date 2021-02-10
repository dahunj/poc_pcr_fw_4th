/*
 * drv_i2c.c
 *
 *  Created on: 2019. 9. 3.
 *      Author: jk.choi
 */
#include "drv_i2c.h"




#define ADDR_STATUS_REG	0x04

#if 0
#define ADDR_HOT_JUNCTION_TEMP_REG	0x00
#define ADDR_JUNCTION_TEMP_DELTA_REG	0x01
#define ADDR_COLD_JUNCTION_TEMP_REG	0x02
#define ADDR_RAW_DATA_ADC_REG		0x03
#define ADDR_STATUS_REG				0x04
#define ADDR_SENSOR_CONFIG_REG		0x05
#define ADDR_DEVICE_CONFIG_REG		0x06
#define ADDR_ALERT_1_CONFIG_REG		0x08
#define ADDR_ALERT_2_CONFIG_REG		0x09
#define ADDR_ALERT_3_CONFIG_REG		0x0A
#define ADDR_ALERT_4_CONFIG_REG		0x0B
#define ADDR_ALERT_1_HYS_REG			0x0C
#define ADDR_ALERT_2_HYS_REG			0x0D
#define ADDR_ALERT_3_HYS_REG			0x0E
#define ADDR_ALERT_4_HYS_REG			0x0F
#define ADDR_ALERT_1_LIMIT_REG			0x10
#define ADDR_ALERT_2_LIMIT_REG			0x11
#define ADDR_ALERT_3_LIMIT_REG			0x12
#define ADDR_ALERT_4_LIMIT_REG			0x13
#define ADDR_DEV_ID_VER_ID_REG			0x20
#endif




void I2C1_init(void)
{

	I2C_InitTypeDef  I2C_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	uint16_t	interrupt_mask = 0;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
       	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* Configure I2C_EE pins: SCL and SDA */
	GPIO_InitStructure.GPIO_Pin 		=  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_OD;//GPIO_Mode_AF_PP; //;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* I2C configuration */
	I2C_InitStructure.I2C_Mode 		= I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle 	= I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 	= 0x38;
	I2C_InitStructure.I2C_Ack 			= I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed 	= 60000;//100000; 60000

#if 0
	/* Enable the USART2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel 		= I2C1_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd 		= ENABLE;
	NVIC_Init(&NVIC_InitStructure);	


	interrupt_mask = I2C_IT_RXNE |I2C_IT_TXE | I2C_IT_STOPF | I2C_IT_ADD10 | I2C_IT_BTF | I2C_IT_ADDR | I2C_IT_SB;
	I2C_ITConfig(I2C1, interrupt_mask ,  ENABLE);
#endif

	/* I2C Peripheral Enable */
	I2C_Cmd(I2C1, ENABLE);
	/* Apply I2C configuration after enabling it */
	I2C_Init(I2C1, &I2C_InitStructure);


	
}




#if 1
void I2C1_EV_IRQHandler(void)
{
	s8 data[1];

	if( I2C_GetITStatus( I2C1, I2C_IT_RXNE )  != RESET)
	{
		data[0] =  I2C_ReceiveData( I2C1) & 0xFF;
		I2C_ClearITPendingBit(I2C1, I2C_IT_RXNE);	

	}
}
#endif
#if 0
uint8_t drv_i2c1_ReadByte(unsigned char address) // 8bit
{
	short msb=0;
	short lsb=0;
	uint32_t  cnt = 0;

	//I2C_Cmd(I2C1, DISABLE);
	//I2C_Cmd(I2C1, ENABLE);

  /* Disable the I2C1 peripheral  */
  I2C_Cmd(I2C1, DISABLE);

  /* Reset all I2C2 registers */
  i2c_delay(500);
  I2C_Cmd(I2C1, ENABLE);

  
	I2C_AcknowledgeConfig(I2C1,ENABLE);
	I2C_GenerateSTART(I2C1,ENABLE);

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
	{
		cnt++;
	}
	
	I2C_Send7bitAddress(I2C1, TARGET_MODULE_ADDRESS, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		cnt++;
	}

	I2C_SendData(I2C1,address); 	/* Register Address */
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		cnt++;
	}

	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))		
	{
		cnt++;
	}

	I2C_Send7bitAddress(I2C1, TARGET_MODULE_ADDRESS, I2C_Direction_Receiver);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
		cnt++;
	}

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
	{
		cnt++;
	}
	lsb = I2C_ReceiveData(I2C1);

	I2C_GenerateSTOP(I2C1,ENABLE);
	I2C_AcknowledgeConfig(I2C1,DISABLE);

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
	{
		cnt++;
	}
	I2C_ReceiveData(I2C1);

	return lsb;
}


//short bmp280ReadShort(unsigned char address)
short drv_i2c1_ReadShort(unsigned char address) // 16bit
{
	short msb=0;
	short lsb=0;

	I2C_AcknowledgeConfig(I2C1,ENABLE);
	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	
	I2C_Send7bitAddress(I2C1, TARGET_MODULE_ADDRESS, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2C1,address); 	/* Register Address */
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C1, TARGET_MODULE_ADDRESS, I2C_Direction_Receiver);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	lsb = I2C_ReceiveData(I2C1);

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	msb = I2C_ReceiveData(I2C1);

	I2C_GenerateSTOP(I2C1,ENABLE);
	I2C_AcknowledgeConfig(I2C1,DISABLE);

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	I2C_ReceiveData(I2C1);

	return (msb << 8) | lsb;
}

//unsigned long bmp280ReadLong(unsigned char address)
unsigned long 	drv_i2c1_ReadLong(unsigned char address) //32bit
{
	unsigned long result=0;

	unsigned long msb=0;
	unsigned long lsb=0;
	unsigned long xsb=0;

	I2C_AcknowledgeConfig(I2C1,ENABLE);
	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C1, TARGET_MODULE_ADDRESS, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2C1,address);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C1, TARGET_MODULE_ADDRESS, I2C_Direction_Receiver);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	msb = I2C_ReceiveData(I2C1);

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	lsb = I2C_ReceiveData(I2C1);

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	xsb = I2C_ReceiveData(I2C1);

	I2C_GenerateSTOP(I2C1,ENABLE);
	I2C_AcknowledgeConfig(I2C1,DISABLE);
  
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	I2C_ReceiveData(I2C1);

	result = (msb << 16) | (lsb << 8) | xsb;

	return (result >> 4);
}

//void bmp280WriteByte(unsigned char address, unsigned char data)
void	drv_i2c1_WriteByte(unsigned char address, unsigned char data)
{
	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C1, TARGET_MODULE_ADDRESS, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2C1,address);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_SendData(I2C1,data);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTOP(I2C1,ENABLE);

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}
#endif

void i2c_test(void)
{

	uint8_t read_u8 = 0;

	//read_u8 = drv_i2c1_ReadByte(ADDR_STATUS_REG);
	//read_u8 = drv_i2c1_ReadByte(0x00);
	//read_u8 = CODEC_ReadRegister(ADDR_STATUS_REG);
	//read_temp();
}



void i2c_delay(uint16_t  cnt)
{
	uint16_t i = 0;
	
	for( i = 0 ; i < cnt; i++)
	{
		;
	}
}


#if 0
void I2C_Config(void)
{
  I2C_InitTypeDef I2C_InitStructure;

  /* I2C1 configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = 0x33;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = 100000;
  I2C_Init(I2C1, &I2C_InitStructure);
}
#endif




uint8_t  i2c_start(void)
{
	uint16_t cnt = 0;
	uint8_t   ret = 1;
	uint16_t cnt2 = 0;

	/* While the bus is busy */
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) /* Add 1107 */
	{
		cnt2++;
		if(cnt2 >= 500)
		{
			ret = 0;
			break;
		}
	} /* UnBlock 1106 */

  
	I2C_GenerateSTART(I2C1, ENABLE);
	
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
	{
		cnt++;
		if(cnt >= 500)
		{
			ret = 0;
			break;
		}
	}
	
	return ret;
}

uint8_t i2c_write(uint8_t w_data, uint8_t addr_flag, uint8_t direction)
{
	uint16_t cnt = 0;
	uint8_t   ret = 1;

	if( addr_flag == FALSE ) /* DATA */
	{
		I2C_SendData(I2C1,w_data); 	
		
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
			cnt++;
			if(cnt >= 500)
			{
				ret = 0;
				break;
			}
		}
	}
	else   /* ADDRESS */
	{
		if( direction == I2C_Direction_Receiver) /* READ : Stretch Enable */
		{
			I2C_StretchClockCmd( I2C1, ENABLE);
			// I2C_Send7bitAddress(I2C1, TARGET_MODULE_ADDRESS, direction);
			I2C_Send7bitAddress(I2C1, w_data, direction);
			
			while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
			{
				cnt++;
				if(cnt >= 500)
				{
					ret = 0;
					break;
				}
			}
		}
		else
		{
			I2C_StretchClockCmd( I2C1, DISABLE);
			// I2C_Send7bitAddress(I2C1, TARGET_MODULE_ADDRESS, direction);
			I2C_Send7bitAddress(I2C1, w_data, direction);
			
			while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
			{
				cnt++;
				if(cnt >= 500)
				{
					ret = 0;
					break;
				}
			}	
		}
	}
	
	return ret;

}


void i2c_stop(void)
{
	I2C_GenerateSTOP(I2C1, ENABLE);
}

uint8_t i2c_read(uint8_t ack_nack)
{
	uint16_t cnt = 0;
	uint8_t   ret = 1;

	I2C_AcknowledgeConfig(I2C1, ack_nack);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
	{
		cnt++;
		if(cnt >= 500)
		{
			ret = 0;
			break;
		}
	}

	//I2C_AcknowledgeConfig(I2C1, ack_nack);
	ret = I2C_ReceiveData(I2C1);
	//I2C_AcknowledgeConfig(I2C1, ack_nack);
	
	return ret;
}



#if 0

/* I2C2 for chamber temperature measure */
/* Add 1029 */
void I2C2_init(void)
{

	I2C_InitTypeDef  I2C_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	uint16_t	interrupt_mask = 0;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
      	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);


	/* Configure I2C_EE pins: SCL and SDA */
	GPIO_InitStructure.GPIO_Pin 		=  GPIO_Pin_10 | GPIO_Pin_11;  /* PB10: SDA,     PB11: SCL */
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* I2C configuration */
	I2C_InitStructure.I2C_Mode 		= I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle 	= I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 	= 0x39;
	I2C_InitStructure.I2C_Ack 			= I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed 	= 60000;	//100000


	/* I2C Peripheral Enable */
	I2C_Cmd(I2C2, ENABLE);
	/* Apply I2C configuration after enabling it */
	I2C_Init(I2C2, &I2C_InitStructure);
	
}


uint8_t  i2c2_start(void)
{
	uint16_t cnt = 0;
	uint8_t   ret = 1;
	uint16_t cnt2 = 0;


  	/* While the bus is busy */

#if 1	
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY)) /* Add 1107 */
	{	
		cnt2++;
		if(cnt2 >= 65535)
		{
			ret = 0;
			return ret;
		}
	} /* UnBlock 1106 */
#else
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));/* Add 1107 */
#endif

	I2C_GenerateSTART(I2C2, ENABLE);
	
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT))
	{
		cnt++;
		if(cnt >= 500)
		{
			ret = 0;
			break;
		}
	}
	
	return ret;
}

uint8_t i2c2_write(uint8_t w_data, uint8_t addr_flag, uint8_t direction)
{
	uint16_t cnt = 0;
	uint8_t   ret = 1;

	if( addr_flag == FALSE ) /* DATA */
	{
		I2C_SendData(I2C2,w_data); 	
		
		while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
			cnt++;
			if(cnt >= 65535)
			{
				ret = 0;
				break;
			}
		}
		cnt = 0; /* Add 1107 */
	}
	else   /* ADDRESS */
	{
		if( direction == I2C_Direction_Receiver) /* READ : Stretch Enable */
		{
			I2C_StretchClockCmd( I2C2, ENABLE);
			I2C_Send7bitAddress(I2C2, TARGET_MODULE_ADDRESS, direction);
			
			while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
			{
				cnt++;
				if(cnt >= 65535)
				{
					ret = 0;
					break;
				}
			}
			cnt = 0; /* Add 1107 */
		}
		else
		{
			I2C_StretchClockCmd( I2C2, DISABLE);
			I2C_Send7bitAddress(I2C2, TARGET_MODULE_ADDRESS, direction);
			
			while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
			{
				cnt++;
				if(cnt >= 65535)
				{
					ret = 0;
					break;
				}
			}
			cnt = 0; /* Add 1107 */
		}
	}
	
	return ret;

}


void i2c2_stop(void)
{
	I2C_GenerateSTOP(I2C2, ENABLE);
}

uint8_t i2c2_read(uint8_t ack_nack)
{
	uint16_t cnt = 0;
	uint8_t   ret = 1;

	I2C_AcknowledgeConfig(I2C2, ack_nack);
	while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED))
	{
		cnt++;
		if(cnt >= 65535)
		{
			ret = 0;
			break;
		}
	}
	cnt = 0; /* Add 1107 */
	ret = I2C_ReceiveData(I2C2);
	
	return ret;
}

#endif

void I2C2_init(void)
{
	// dummy
}
