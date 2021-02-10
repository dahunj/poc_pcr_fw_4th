/*
 * drv_usart.h
 *
 *  Created on: 2019. 7. 25.
 *      Author: jk.choi
 */

#ifndef DRV_USART_H_
#define DRV_USART_H_


#ifndef  USART_PRESENT
#define  USART_PRESENT

#include <stm32f10x_usart.h>

#define BAUDRATE_485 9600

/* ------------------------------------------------------------------------------------------------- */
/* BSP USART */
/* ------------------------------------------------------------------------------------------------- */
typedef enum
{
	usartmodeDMA,
	usartmodeIRQ,
	usartmodeMAX
} usartmode_type;

typedef enum
{
	usart1ServiceFunction,
	usart2ServiceFunction,
	usart3ServiceFunction,
	usartServiceFunctionMAX
} usart_register_function_type;

typedef            void     (*usart_register_function)(char data);

typedef struct _usart_service_function_type
{
	usart_register_function_type service_type;
	usart_register_function run;
} usart_service_function_type;

/* ------------------------------------------------------------------------------------------------- */
/* function USART */
/* ------------------------------------------------------------------------------------------------- */

void USART1_IRQHandler(void);
void USART2_IRQHandler(void);


/* ------------------------------------------------------------------------------------------------- */
/* extern USART */
/* ------------------------------------------------------------------------------------------------- */

extern void init_usart1_buffer(void);				    
extern void usart_transmit_byte( USART_TypeDef* port, u16 chr);
extern void usart1_transmit_byte( u16 chr);
extern void usart1_transmit_data(uint8_t* p_buf, uint16_t len); /* Add 2019.10.08 */
extern void drv_usart1_init(void);
extern void drv_usart2_init(void);
extern void usart2_transmit_byte(u16 chr);
extern void usart2_transmit_string(char* data);
extern void register_usart_function(usart_register_function_type usart_fn_type, usart_register_function fn);
extern void drv_usart3_init(void);
extern void bsp_init_irq_usart3(void/*isr_function usart3_isr*/);
extern void usart3_transmit_byte(u16 chr);
extern void usart3_transmit_string(char* data);
extern void serial_comm_get_data(uint8_t recv_data);
extern  void serial_get_msg(void);
extern void serial_clear_recv_bytes(void);

#endif                                                          

/* End of module include.                               */


#endif /* DRV_USART_H_ */
