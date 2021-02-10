/*
 * drv_adafruit_mcp9600.h
 *
 *  Created on: 2019. 9. 6.
 *      Author: jk.choi
 */

#ifndef DRV_ADAFRUIT_MCP9600_H_
#define DRV_ADAFRUIT_MCP9600_H_

#include "drv_i2c.h"

#define UPDATE_MASK  0x40

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



float drv_mcp9600_read_temp(uint8_t addr_ptr);
uint8_t drv_mcp9600_set_config(uint8_t addr_ptr, uint8_t w_data);
uint8_t drv_mcp9600_read_multiple(uint8_t addr_ptr, uint8_t r_cnt, uint8_t* p_rbuf);
uint8_t drv_mcp9600_set_alert(uint8_t addr_ptr, uint16_t w_data);
void adafruit_mcp9600_test(void);
void drv_mcp9600_config(void);

/* I2C2 for chamber temperature measure */
/* Add 1029 */
 float drv_mcp9600_2_read_temp(uint8_t addr_ptr);
uint8_t drv_mcp9600_2_set_config(uint8_t addr_ptr, uint8_t w_data);
uint8_t drv_mcp9600_2_read_multiple(uint8_t addr_ptr, uint8_t r_cnt, uint8_t* p_rbuf);
// uint8_t drv_mcp9600_2_set_alert(uint8_t addr_ptr, uint16_t w_data);
// void adafruit_mcp9600_2_test(void);
void drv_mcp9600_2_config(void);

#endif /* DRV_ADAFRUIT_MCP9600_H_ */
