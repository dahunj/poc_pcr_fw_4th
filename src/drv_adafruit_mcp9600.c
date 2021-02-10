/*
 * drv_adafruit_mcp9600.c
 *
 *  Created on: 2019. 9. 6.
 *      Author: HJBAE
 */
#include "drv_adafruit_mcp9600.h"




float drv_mcp9600_read_temp(uint8_t addr_ptr)
{

	uint8_t UpperByte = 0;
	uint8_t LowerByte = 0;
	uint8_t ret_val = 0;
	float temperature = 0;
	float tmp1 = 0, tmp2 = 0;

	if( !((addr_ptr >= ADDR_HOT_JUNCTION_TEMP_REG) && (addr_ptr <= ADDR_COLD_JUNCTION_TEMP_REG) ) )
	{
		return 0;
	 }

	 
	ret_val = i2c_start();
	ret_val = i2c_write( TARGET_MODULE_ADDRESS, TRUE, I2C_Direction_Transmitter );
	ret_val = i2c_write( addr_ptr ,FALSE, 0);
	i2c_stop();
	  
	ret_val = i2c_start();
	ret_val = i2c_write( TARGET_MODULE_ADDRESS, TRUE, I2C_Direction_Receiver);
	UpperByte = i2c_read(ACK);
	i2c_stop();
	LowerByte = i2c_read(NACK);

	if((UpperByte & 0x80) == 0x80)
	{
		tmp1 = (float)(UpperByte * 16);
		tmp2 = (float)LowerByte /(float)16;
		temperature = ( tmp1+ tmp2) - (float)4096;
	}
	else
	{
		tmp1 = (float)(UpperByte * 16);
		tmp2 = (float)LowerByte /(float)16;
		temperature = ( tmp1+ tmp2);
	}

	return temperature;
}


uint8_t drv_mcp9600_set_config(uint8_t addr_ptr, uint8_t w_data)
{

	uint8_t r_data = 0;
	uint8_t ret_val = 0;

	if( !((addr_ptr >= ADDR_STATUS_REG) && (addr_ptr <= ADDR_DEVICE_CONFIG_REG) ) )
	{
		return 0;
	 }
	  
	ret_val = i2c_start();
	ret_val = i2c_write( TARGET_MODULE_ADDRESS, TRUE, I2C_Direction_Transmitter );
	ret_val = i2c_write( addr_ptr ,FALSE, 0);
	ret_val = i2c_write( w_data ,FALSE, 0);
	i2c_stop();
	  
	ret_val = i2c_start();
	ret_val = i2c_write( TARGET_MODULE_ADDRESS, TRUE, I2C_Direction_Receiver);
	
	i2c_stop();
	r_data = i2c_read(NACK);
	
	if( w_data == r_data)
	{
		ret_val = 1;
	}
	
	return ret_val;
}


uint8_t drv_mcp9600_read_multiple(uint8_t addr_ptr, uint8_t r_cnt, uint8_t* p_rbuf)
{


	uint8_t r_data = 0;
	uint8_t ret_val = 0;
	uint8_t i = 0;
	uint8_t tmp = 0;
	
	if( !((addr_ptr >= ADDR_HOT_JUNCTION_TEMP_REG) && (addr_ptr  <= ADDR_DEV_ID_VER_ID_REG) ) )
	{
		return 0;
	 }
	 
	ret_val = i2c_start();
	ret_val = i2c_write( TARGET_MODULE_ADDRESS, TRUE, I2C_Direction_Transmitter );
	ret_val = i2c_write( addr_ptr, FALSE, 0);
	i2c_stop();
	  
	ret_val = i2c_start();
	ret_val = i2c_write( TARGET_MODULE_ADDRESS, TRUE, I2C_Direction_Receiver);
	for( i = 0; i < (r_cnt -1); i++)
	{
		tmp = i2c_read(ACK);
		//*(p_rbuf + i) = i2c_read(ACK);
		*(p_rbuf + i) = tmp;
		
	}
	i2c_stop();
	//*(p_rbuf + i)  = i2c_read(NACK);
	tmp  = i2c_read(NACK);
	*(p_rbuf + i)  = tmp;
	
	ret_val = 1;
	
	return ret_val;
}



uint8_t drv_mcp9600_set_alert(uint8_t addr_ptr, uint16_t w_data)
{
	uint8_t w8u_data = 0;
	uint8_t ret_val = 0;
	uint8_t i = 0;
	uint8_t UpperByte = 0;
	uint8_t LowerByte = 0;
	uint16_t tmp = 0;
	
	ret_val = i2c_start();
	ret_val = i2c_write( TARGET_MODULE_ADDRESS, TRUE, I2C_Direction_Transmitter );
	ret_val = i2c_write( addr_ptr, FALSE, 0);

	if( (addr_ptr >= ADDR_ALERT_1_LIMIT_REG ) && ( addr_ptr <= ADDR_ALERT_4_LIMIT_REG ) )
	{
		w8u_data = (uint8_t)(w_data >> 8);
		ret_val = i2c_write( w8u_data ,FALSE, 0);
		
		w8u_data = (uint8_t)(w_data & 0xFF);
		ret_val = i2c_write( w8u_data ,FALSE, 0);
	}
	else
	{
		w8u_data = (uint8_t)(w_data & 0xFF);
		ret_val = i2c_write( w8u_data ,FALSE, 0);
	}
	
	i2c_stop();

	ret_val = i2c_start();
	ret_val = i2c_write( TARGET_MODULE_ADDRESS, TRUE, I2C_Direction_Receiver);
	
	if( (addr_ptr >= ADDR_ALERT_1_LIMIT_REG ) && ( addr_ptr <= ADDR_ALERT_4_LIMIT_REG ) )
	{
		UpperByte = i2c_read(ACK);
		i2c_stop();
		LowerByte = i2c_read(NACK);
	}
	else
	{
		i2c_stop();
		LowerByte = i2c_read(NACK);
	}

	if( (addr_ptr >= ADDR_ALERT_1_LIMIT_REG ) && ( addr_ptr <= ADDR_ALERT_4_LIMIT_REG ) )
	{
		tmp = ((uint16_t)UpperByte << 8) | (uint16_t)LowerByte;
		if(w_data == tmp)
		{
			ret_val = 1;
		}
		else
		{
			ret_val = 0;
		}
	}
	else
	{
		if(w_data == LowerByte)
		{
			ret_val = 1;
		}
		else
		{
			ret_val = 0;
		}
	}

	return ret_val;
}

void drv_mcp9600_config(void)
{
	uint8_t result = 0;
	
	result = drv_mcp9600_set_config( ADDR_SENSOR_CONFIG_REG,  0x20);	// 0x22
	result = drv_mcp9600_set_config( ADDR_DEVICE_CONFIG_REG,  0xC8);		// 0xA8

	result = drv_mcp9600_set_alert( ADDR_ALERT_1_CONFIG_REG, 0x8C);
	result = drv_mcp9600_set_alert( ADDR_ALERT_2_CONFIG_REG, 0x8C);
	result = drv_mcp9600_set_alert( ADDR_ALERT_3_CONFIG_REG, 0x8C);
	result = drv_mcp9600_set_alert( ADDR_ALERT_4_CONFIG_REG, 0x8C);
	result = drv_mcp9600_set_alert( ADDR_ALERT_1_HYS_REG, 0x02);
	result = drv_mcp9600_set_alert( ADDR_ALERT_2_HYS_REG, 0x02);
	result = drv_mcp9600_set_alert( ADDR_ALERT_3_HYS_REG, 0x02);
	result = drv_mcp9600_set_alert( ADDR_ALERT_4_HYS_REG, 0x02);
	result = drv_mcp9600_set_alert( ADDR_ALERT_1_LIMIT_REG, 0x0780);
	result = drv_mcp9600_set_alert( ADDR_ALERT_2_LIMIT_REG, 0x05A0);
	result = drv_mcp9600_set_alert( ADDR_ALERT_3_LIMIT_REG, 0x03C0);
	result = drv_mcp9600_set_alert( ADDR_ALERT_4_LIMIT_REG, 0x0320);

	uint8_t bf[2];
	drv_mcp9600_read_multiple(ADDR_DEV_ID_VER_ID_REG, 2, bf);
	printf("MCP9600 Module 1: 0x%x 0x%x\n", bf[0], bf[1]);

	// float a = drv_mcp9600_read_temp(ADDR_COLD_JUNCTION_TEMP_REG);
	// printf("[Cold] module 1 %d.%s\n", (int)a, sub_three(a));
}




void adafruit_mcp9600_test(void)
{
	float temp_flt =0;
	uint8_t buf[50] = {0};
	uint8_t result = 0;
	static uint16_t cnt = 0;
	
	if(cnt == 0)
	{
		result = drv_mcp9600_set_config( ADDR_SENSOR_CONFIG_REG,  0x00);
		result = drv_mcp9600_set_config( ADDR_DEVICE_CONFIG_REG,  0xA8);
		
		result = drv_mcp9600_set_alert( ADDR_ALERT_1_CONFIG_REG, 0x8C);
		result = drv_mcp9600_set_alert( ADDR_ALERT_2_CONFIG_REG, 0x8C);
		result = drv_mcp9600_set_alert( ADDR_ALERT_3_CONFIG_REG, 0x8C);
		result = drv_mcp9600_set_alert( ADDR_ALERT_4_CONFIG_REG, 0x8C);
		result = drv_mcp9600_set_alert( ADDR_ALERT_1_HYS_REG, 0x02);
		result = drv_mcp9600_set_alert( ADDR_ALERT_2_HYS_REG, 0x02);
		result = drv_mcp9600_set_alert( ADDR_ALERT_3_HYS_REG, 0x02);
		result = drv_mcp9600_set_alert( ADDR_ALERT_4_HYS_REG, 0x02);
		result = drv_mcp9600_set_alert( ADDR_ALERT_1_LIMIT_REG, 0x0780);
		result = drv_mcp9600_set_alert( ADDR_ALERT_2_LIMIT_REG, 0x05A0);
		result = drv_mcp9600_set_alert( ADDR_ALERT_3_LIMIT_REG, 0x03C0);
		result = drv_mcp9600_set_alert( ADDR_ALERT_4_LIMIT_REG, 0x0320);
	}
	
	cnt++;
	
	drv_mcp9600_read_multiple(ADDR_STATUS_REG, 1, buf);

	if( (buf[0] & UPDATE_MASK) == UPDATE_MASK)
	{

		temp_flt = drv_mcp9600_read_temp( ADDR_HOT_JUNCTION_TEMP_REG );
		temp_flt = drv_mcp9600_read_temp( ADDR_JUNCTION_TEMP_DELTA_REG );
		temp_flt = drv_mcp9600_read_temp( ADDR_COLD_JUNCTION_TEMP_REG );
		
		result = drv_mcp9600_set_config( ADDR_STATUS_REG,  0x40);
	}
	
	
	drv_mcp9600_read_multiple(ADDR_DEVICE_CONFIG_REG, 1, buf);
	drv_mcp9600_read_multiple(ADDR_DEV_ID_VER_ID_REG, 2, buf);
	drv_mcp9600_read_multiple(ADDR_ALERT_1_CONFIG_REG, 1, buf);
	drv_mcp9600_read_multiple(ADDR_ALERT_2_CONFIG_REG, 1, buf);
	drv_mcp9600_read_multiple(ADDR_ALERT_3_CONFIG_REG, 1, buf);
	drv_mcp9600_read_multiple(ADDR_ALERT_4_CONFIG_REG, 1, buf);
	
	drv_mcp9600_read_multiple(ADDR_ALERT_1_HYS_REG, 1, buf);
	drv_mcp9600_read_multiple(ADDR_ALERT_2_HYS_REG, 1, buf);
	drv_mcp9600_read_multiple(ADDR_ALERT_3_HYS_REG, 1, buf);
	drv_mcp9600_read_multiple(ADDR_ALERT_4_HYS_REG, 1, buf);	
	
	drv_mcp9600_read_multiple(ADDR_ALERT_1_LIMIT_REG, 2, buf);
	drv_mcp9600_read_multiple(ADDR_ALERT_2_LIMIT_REG, 2, buf);
	drv_mcp9600_read_multiple(ADDR_ALERT_3_LIMIT_REG, 2, buf);
	drv_mcp9600_read_multiple(ADDR_ALERT_4_LIMIT_REG, 2, buf);	

}


//---------------------------------------------------------------------------------


/* I2C2 for chamber temperature measure */
/* Add 1029 */

float drv_mcp9600_2_read_temp(uint8_t addr_ptr)
{

	uint8_t UpperByte = 0;
	uint8_t LowerByte = 0;
	//uint8_t ret_val = 0;
	uint8_t ret_val = 1; /* 1106 */
	float temperature = 0;
	float tmp1 = 0, tmp2 = 0;
	uint8_t cnt = 0;

	if( !((addr_ptr >= ADDR_HOT_JUNCTION_TEMP_REG) && (addr_ptr <= ADDR_COLD_JUNCTION_TEMP_REG) ) )
	{
		return 0;
	}

	ret_val = i2c_start();
	if( ret_val == 0)
	{
		while( ret_val == 0)
		{
			ret_val = i2c_start();
			cnt++;
			if( cnt >= 3)
			{
				/* Disable the I2C2 peripheral  */
				// I2C_Cmd(I2C2, DISABLE);

				/* Reset all I2C2 registers */
				i2c_delay(500);
				// I2C_Cmd(I2C2, ENABLE);
				return 0;
			}
		}
		cnt = 0;
	}
	
	ret_val = i2c_write( TARGET_MODULE2_ADDRESS, TRUE, I2C_Direction_Transmitter );
	ret_val = i2c_write( addr_ptr ,FALSE, 0);
	i2c_stop();
	
	if( ret_val == 0) /* Error : 1106 */
	{
		return 0;
	}  


	ret_val = i2c_start();
	if( ret_val == 0)
	{
		while( ret_val == 0)
		{
			ret_val = i2c_start();
			cnt++;
			if( cnt >= 3)
			{
				/* Disable the I2C2 peripheral  */
				// I2C_Cmd(I2C2, DISABLE);

				/* Reset all I2C2 registers */
				i2c_delay(500);
				// I2C_Cmd(I2C2, ENABLE);
				return 0;
			}
		}
		cnt = 0;
	}
	
	ret_val        = i2c_write( TARGET_MODULE2_ADDRESS, TRUE, I2C_Direction_Receiver);
	UpperByte = i2c_read(ACK);
	i2c_stop();
	LowerByte = i2c_read(NACK);
	//i2c_stop();
	
	if( ret_val == 0) /* Error : 1106 */
	{
		return 0;
	}

	if((UpperByte & 0x80) == 0x80)
	{
		tmp1 = (float)(UpperByte * 16);
		tmp2 = (float)LowerByte /(float)16;
		temperature = ( tmp1+ tmp2) - (float)4096;
	}
	else
	{
		tmp1 = (float)(UpperByte * 16);
		tmp2 = (float)LowerByte /(float)16;
		temperature = ( tmp1+ tmp2);
	}

	return temperature;
}


uint8_t drv_mcp9600_2_set_config(uint8_t addr_ptr, uint8_t w_data)
{

	uint8_t r_data = 0;
	uint8_t ret_val = 0;
	uint8_t cnt = 0;

	if( !((addr_ptr >= ADDR_STATUS_REG) && (addr_ptr <= ADDR_DEVICE_CONFIG_REG) ) )
	{
		return 0;
	 }
	  
	ret_val = i2c_start();
	if( ret_val == 0)
	{
		while( ret_val == 0)
		{
			ret_val = i2c_start();
			cnt++;
			if( cnt >= 3)
			{
				/* Disable the I2C2 peripheral  */
				// I2C_Cmd(I2C2, DISABLE);

				/* Reset all I2C2 registers */
				i2c_delay(500);
				// I2C_Cmd(I2C2, ENABLE);
				return 0;
			}
		}
		cnt = 0;
	}	
	ret_val = i2c_write( TARGET_MODULE2_ADDRESS, TRUE, I2C_Direction_Transmitter );
	ret_val = i2c_write( addr_ptr ,FALSE, 0);
	ret_val = i2c_write( w_data ,FALSE, 0);
	i2c_stop();
	  
	ret_val = i2c_start();
	if( ret_val == 0)
	{
		while( ret_val == 0)
		{
			ret_val = i2c_start();
			cnt++;
			if( cnt >= 3)
			{
				//i2c_stop();
				/* Disable the I2C2 peripheral  */
				// I2C_Cmd(I2C2, DISABLE);

				/* Reset all I2C2 registers */
				i2c_delay(500);
				// I2C_Cmd(I2C2, ENABLE);
				return 0;
			}
		}
		cnt = 0;
	}	
	
	ret_val = i2c_write( TARGET_MODULE2_ADDRESS, TRUE, I2C_Direction_Receiver);
	i2c_stop();
	r_data = i2c_read(NACK);
	//i2c_stop();

	if( w_data == r_data)
	{
		ret_val = 1;
	}
	
	return ret_val;
}


uint8_t drv_mcp9600_2_read_multiple(uint8_t addr_ptr, uint8_t r_cnt, uint8_t* p_rbuf)
{
	uint8_t r_data = 0;
	uint8_t ret_val = 0;
	uint8_t i = 0;
	uint8_t tmp = 0;
	uint8_t cnt = 0;
	
	if( !((addr_ptr >= ADDR_HOT_JUNCTION_TEMP_REG) && (addr_ptr  <= ADDR_DEV_ID_VER_ID_REG) ) )
	{
		return 0;
	 }
	 
	ret_val = i2c_start();
	if( ret_val == 0)
	{
		while( ret_val == 0)
		{
			ret_val = i2c_start();
			cnt++;
			if( cnt >= 3)
			{
				/* Disable the I2C2 peripheral  */
				// I2C_Cmd(I2C2, DISABLE);

				/* Reset all I2C2 registers */
				i2c_delay(500);
				// I2C_Cmd(I2C2, ENABLE);
				return 0;
			}
		}
		cnt = 0;
	}
	ret_val = i2c_write( TARGET_MODULE2_ADDRESS, TRUE, I2C_Direction_Transmitter );
	ret_val = i2c_write( addr_ptr, FALSE, 0);
	i2c_stop();
	  
	ret_val = i2c_start();
	if( ret_val == 0)
	{
		while( ret_val == 0)
		{
			ret_val = i2c_start();
			cnt++;
			if( cnt >= 3)
			{
				/* Disable the I2C2 peripheral  */
				// I2C_Cmd(I2C2, DISABLE);

				/* Reset all I2C2 registers */
				i2c_delay(500);
				// I2C_Cmd(I2C2, ENABLE);
				return 0;
			}
		}
		cnt = 0;
	}
	
	ret_val = i2c_write( TARGET_MODULE2_ADDRESS, TRUE, I2C_Direction_Receiver);
	for( i = 0; i < (r_cnt -1); i++)
	{
		tmp = i2c_read(ACK);
		*(p_rbuf + i) = tmp;
		
	}

	i2c_stop();
	tmp  = i2c_read(NACK);
	*(p_rbuf + i)  = tmp;	
	//i2c_stop();
	
	ret_val = 1;
	
	return ret_val;
}



uint8_t drv_mcp9600_2_set_alert(uint8_t addr_ptr, uint16_t w_data)
{
	uint8_t w8u_data = 0;
	uint8_t ret_val = 0;
	uint8_t i = 0;
	uint8_t UpperByte = 0;
	uint8_t LowerByte = 0;
	uint16_t tmp = 0;
	uint8_t cnt = 0;

	
	ret_val = i2c_start();
	if( ret_val == 0)
	{
		while( ret_val == 0)
		{
			ret_val = i2c_start();
			cnt++;
			if( cnt >= 3)
			{
				/* Disable the I2C2 peripheral  */
				// I2C_Cmd(I2C2, DISABLE);

				/* Reset all I2C2 registers */
				i2c_delay(500);
				// I2C_Cmd(I2C2, ENABLE);
				return 0;
			}
		}
		cnt = 0;
	}
	ret_val = i2c_write( TARGET_MODULE2_ADDRESS, TRUE, I2C_Direction_Transmitter );
	ret_val = i2c_write( addr_ptr, FALSE, 0);

	if( (addr_ptr >= ADDR_ALERT_1_LIMIT_REG ) && ( addr_ptr <= ADDR_ALERT_4_LIMIT_REG ) )
	{
		w8u_data = (uint8_t)(w_data >> 8);
		ret_val = i2c_write( w8u_data ,FALSE, 0);
		
		w8u_data = (uint8_t)(w_data & 0xFF);
		ret_val = i2c_write( w8u_data ,FALSE, 0);
	}
	else
	{
		w8u_data = (uint8_t)(w_data & 0xFF);
		ret_val = i2c_write( w8u_data ,FALSE, 0);
	}
	
	i2c_stop();

	ret_val = i2c_start();
	if( ret_val == 0)
	{
		while( ret_val == 0)
		{
			ret_val = i2c_start();
			cnt++;
			if( cnt >= 3)
			{
				/* Disable the I2C2 peripheral  */
				// I2C_Cmd(I2C2, DISABLE);

				/* Reset all I2C2 registers */
				i2c_delay(500);
				// I2C_Cmd(I2C2, ENABLE);
				return 0;
			}
		}
		cnt = 0;
	}
	
	ret_val = i2c_write( TARGET_MODULE2_ADDRESS, TRUE, I2C_Direction_Receiver);
	
	if( (addr_ptr >= ADDR_ALERT_1_LIMIT_REG ) && ( addr_ptr <= ADDR_ALERT_4_LIMIT_REG ) )
	{
		UpperByte = i2c_read(ACK);
		i2c_stop();
		LowerByte = i2c_read(NACK);
		//i2c_stop();
	}
	else
	{
		i2c_stop();
		LowerByte = i2c_read(NACK);
		//i2c_stop();
	}


	if( (addr_ptr >= ADDR_ALERT_1_LIMIT_REG ) && ( addr_ptr <= ADDR_ALERT_4_LIMIT_REG ) )
	{
		tmp = ((uint16_t)UpperByte << 8) | (uint16_t)LowerByte;
		if(w_data == tmp)
		{
			ret_val = 1;
		}
		else
		{
			ret_val = 0;
		}
	}
	else
	{
		if(w_data == LowerByte)
		{
			ret_val = 1;
		}
		else
		{
			ret_val = 0;
		}
	}

	return ret_val;
}

void drv_mcp9600_2_config(void)
{
	uint8_t result = 0;
	
	result = drv_mcp9600_2_set_config( ADDR_SENSOR_CONFIG_REG,  0x20); // 0x22
	result = drv_mcp9600_2_set_config( ADDR_DEVICE_CONFIG_REG,  0xC4);	// 0xA4 0xC4
	
	result = drv_mcp9600_2_set_alert( ADDR_ALERT_1_CONFIG_REG, 0x8C);
	result = drv_mcp9600_2_set_alert( ADDR_ALERT_2_CONFIG_REG, 0x8C);
	result = drv_mcp9600_2_set_alert( ADDR_ALERT_3_CONFIG_REG, 0x8C);
	result = drv_mcp9600_2_set_alert( ADDR_ALERT_4_CONFIG_REG, 0x8C);
	result = drv_mcp9600_2_set_alert( ADDR_ALERT_1_HYS_REG, 0x02);
	result = drv_mcp9600_2_set_alert( ADDR_ALERT_2_HYS_REG, 0x02);
	result = drv_mcp9600_2_set_alert( ADDR_ALERT_3_HYS_REG, 0x02);
	result = drv_mcp9600_2_set_alert( ADDR_ALERT_4_HYS_REG, 0x02);
	result = drv_mcp9600_2_set_alert( ADDR_ALERT_1_LIMIT_REG, 0x0780);
	result = drv_mcp9600_2_set_alert( ADDR_ALERT_2_LIMIT_REG, 0x05A0);
	result = drv_mcp9600_2_set_alert( ADDR_ALERT_3_LIMIT_REG, 0x03C0);
	result = drv_mcp9600_2_set_alert( ADDR_ALERT_4_LIMIT_REG, 0x0320);

	uint8_t bf[2];
	drv_mcp9600_read_multiple(ADDR_DEV_ID_VER_ID_REG, 2, bf);
	printf("MCP9600 Module 2: 0x%x 0x%x\n", bf[0], bf[1]);

	// float a = drv_mcp9600_2_read_temp(ADDR_COLD_JUNCTION_TEMP_REG);
	// printf("[Cold] module 2 %d.%s\n", (int)a, sub_three(a));
}




void adafruit_mcp9600_2_test(void)
{
	float temp_flt =0;
	uint8_t buf[50] = {0};
	uint8_t result = 0;
	static uint16_t cnt = 0;
	
	if(cnt == 0)
	{
		result = drv_mcp9600_2_set_config( ADDR_SENSOR_CONFIG_REG,  0x00);
		result = drv_mcp9600_2_set_config( ADDR_DEVICE_CONFIG_REG,  0xA8);
		
		result = drv_mcp9600_2_set_alert( ADDR_ALERT_1_CONFIG_REG, 0x8C);
		result = drv_mcp9600_2_set_alert( ADDR_ALERT_2_CONFIG_REG, 0x8C);
		result = drv_mcp9600_2_set_alert( ADDR_ALERT_3_CONFIG_REG, 0x8C);
		result = drv_mcp9600_2_set_alert( ADDR_ALERT_4_CONFIG_REG, 0x8C);
		result = drv_mcp9600_2_set_alert( ADDR_ALERT_1_HYS_REG, 0x02);
		result = drv_mcp9600_2_set_alert( ADDR_ALERT_2_HYS_REG, 0x02);
		result = drv_mcp9600_2_set_alert( ADDR_ALERT_3_HYS_REG, 0x02);
		result = drv_mcp9600_2_set_alert( ADDR_ALERT_4_HYS_REG, 0x02);
		result = drv_mcp9600_2_set_alert( ADDR_ALERT_1_LIMIT_REG, 0x0780);
		result = drv_mcp9600_2_set_alert( ADDR_ALERT_2_LIMIT_REG, 0x05A0);
		result = drv_mcp9600_2_set_alert( ADDR_ALERT_3_LIMIT_REG, 0x03C0);
		result = drv_mcp9600_2_set_alert( ADDR_ALERT_4_LIMIT_REG, 0x0320);
	}
	
	cnt++;
	
	drv_mcp9600_2_read_multiple(ADDR_STATUS_REG, 1, buf);

	if( (buf[0] & UPDATE_MASK) == UPDATE_MASK)
	{

		temp_flt = drv_mcp9600_2_read_temp( ADDR_HOT_JUNCTION_TEMP_REG );
		temp_flt = drv_mcp9600_2_read_temp( ADDR_JUNCTION_TEMP_DELTA_REG );
		temp_flt = drv_mcp9600_2_read_temp( ADDR_COLD_JUNCTION_TEMP_REG );
		
		result = drv_mcp9600_2_set_config( ADDR_STATUS_REG,  0x40);
	}
	
	
	drv_mcp9600_2_read_multiple(ADDR_DEVICE_CONFIG_REG, 1, buf);
	drv_mcp9600_2_read_multiple(ADDR_DEV_ID_VER_ID_REG, 2, buf);
	drv_mcp9600_2_read_multiple(ADDR_ALERT_1_CONFIG_REG, 1, buf);
	drv_mcp9600_2_read_multiple(ADDR_ALERT_2_CONFIG_REG, 1, buf);
	drv_mcp9600_2_read_multiple(ADDR_ALERT_3_CONFIG_REG, 1, buf);
	drv_mcp9600_2_read_multiple(ADDR_ALERT_4_CONFIG_REG, 1, buf);
	
	drv_mcp9600_2_read_multiple(ADDR_ALERT_1_HYS_REG, 1, buf);
	drv_mcp9600_2_read_multiple(ADDR_ALERT_2_HYS_REG, 1, buf);
	drv_mcp9600_2_read_multiple(ADDR_ALERT_3_HYS_REG, 1, buf);
	drv_mcp9600_2_read_multiple(ADDR_ALERT_4_HYS_REG, 1, buf);	
	
	drv_mcp9600_2_read_multiple(ADDR_ALERT_1_LIMIT_REG, 2, buf);
	drv_mcp9600_2_read_multiple(ADDR_ALERT_2_LIMIT_REG, 2, buf);
	drv_mcp9600_2_read_multiple(ADDR_ALERT_3_LIMIT_REG, 2, buf);
	drv_mcp9600_2_read_multiple(ADDR_ALERT_4_LIMIT_REG, 2, buf);	

}

