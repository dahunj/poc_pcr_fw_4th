
#ifndef DRV_I2C_H_
#define DRV_I2C_H_

#include "stm32f10x_i2c.h"

#define READ  			1
#define WRITE 		0

#define ACK			1
#define NACK			0

#define TRUE			1
#define FALSE			0

// module 1 : 0xce (pull-up 10k) - chamber
// module 2 : 0xca (pull-up 10k, pull-down 22k) - peltier

#define ADDRESS_ADAFRUIT_MPC9600	0xce //0xC0//
#define TARGET_MODULE_ADDRESS 	ADDRESS_ADAFRUIT_MPC9600
#define TARGET_MODULE2_ADDRESS  0xca

#define BMP280_addr 0xED

#define	BMP280_REG_CONTROL 0xF4
#define	BMP280_REG_CONFIG 0xF5

#define	BMP280_REG_RESULT_PRESSURE 0xF7			// 0xF7(msb) , 0xF8(lsb) , 0xF9(xlsb) : stores the pressure data.
#define BMP280_REG_RESULT_TEMPRERATURE 0xFA		// 0xFA(msb) , 0xFB(lsb) , 0xFC(xlsb) : stores the temperature data.

#define	BMP280_OVERSAMPLING_T1		0x20
#define	BMP280_OVERSAMPLING_T2		0x40
#define	BMP280_OVERSAMPLING_T4		0x60
#define	BMP280_OVERSAMPLING_T8		0x80
#define	BMP280_OVERSAMPLING_T16		0xA0

#define	BMP280_OVERSAMPLING_P1		0x04
#define	BMP280_OVERSAMPLING_P2		0x08
#define	BMP280_OVERSAMPLING_P4		0x0C
#define	BMP280_OVERSAMPLING_P8		0x10
#define	BMP280_OVERSAMPLING_P16		0x14

#define	BMP280_MODE_SLEEP			0x00
#define	BMP280_MODE_FORCED			0x01
#define	BMP280_MODE_NORMAL			0x03

#define	BMP280_TSB_0_5				0x00
#define	BMP280_TSB_62_5				0x20
#define	BMP280_TSB_125				0x40
#define	BMP280_TSB_250				0x60
#define	BMP280_TSB_500				0x80
#define	BMP280_TSB_1000				0xA0
#define	BMP280_TSB_2000				0xC0
#define	BMP280_TSB_4000				0xE0

#define	BMP280_FILTER_OFF			0x00
#define	BMP280_FILTER_COEFFICIENT2	0x04
#define	BMP280_FILTER_COEFFICIENT4	0x08
#define	BMP280_FILTER_COEFFICIENT8	0x0C
#define	BMP280_FILTER_COEFFICIENT16	0x10

#define	BMP280_SPI_OFF	0x00
#define	BMP280_SPI_ON	0x01

#define	BMP280_MEAS			(BMP280_OVERSAMPLING_T16 | BMP280_OVERSAMPLING_P16 | BMP280_MODE_NORMAL)
#define	BMP280_CONFIG		(BMP280_TSB_0_5 | BMP280_FILTER_COEFFICIENT16 | BMP280_SPI_OFF)


uint8_t drv_i2c1_ReadByte(unsigned char address);
short drv_i2c1_ReadShort(unsigned char address);
unsigned long 	drv_i2c1_ReadLong(unsigned char address);
void	drv_i2c1_WriteByte(unsigned char address, unsigned char data);

void I2C1_init(void);

void i2c_test(void);
void i2c_delay(uint16_t cnt);
 void I2C_Config(void);

uint8_t  i2c_start(void);
uint8_t i2c_write(uint8_t w_data, uint8_t addr_flag, uint8_t direction);
void i2c_stop(void);
uint8_t i2c_read(uint8_t ack_nack);

/* I2C2 for chamber temperature measure */
/* Add 1029 */
void I2C2_init(void);
uint8_t  i2c2_start(void);
uint8_t i2c2_write(uint8_t w_data, uint8_t addr_flag, uint8_t direction);
void i2c2_stop(void);
uint8_t i2c2_read(uint8_t ack_nack);
 
#endif /* DRV_I2C_H_ */
