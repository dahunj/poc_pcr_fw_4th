#include  <stdio.h>
#include  <string.h>
#include  <ctype.h>
#include  <stdlib.h>
#include  <stdarg.h>

#include "stm32f10x.h"
#include "hw_config.h"
#include "queue.h"
#include "drv_usart.h"
#include "modbus_hal.h"

#define TRUE 1
#define FALSE 0

// setting USART
#define USART_TX_BUFF_CNT	64
#define USART_RX_BUFF_CNT	10

#define USART_TX_DMA_SIZ          64
#define USART_RX_DMA_SIZ          64

// Add 2019.10.01
stTransiveData recv1_bytes;
stTransiveData recv1_msg;
stTransiveData send1_msg;
uint8_t recv1_msg_flag;

#if 0
q_list_type gbl_qlist_usart1_tx;
q_list_type gbl_qlist_usart1_tx_free;

q_node_type gbl_qdata_usart1_tx[USART_TX_BUFF_CNT];
q_node_type gbl_qdata_usart1_tx_free[USART_TX_BUFF_CNT];

char gbl_usart1_tx_buff[USART_TX_BUFF_CNT][USART_TX_DMA_SIZ];
char gbl_usart1_tx_buff_dma[USART_TX_DMA_SIZ];

q_list_type gbl_qlist_usart1_rx;
q_list_type gbl_qlist_usart1_rx_free;

q_node_type gbl_qdata_usart1_rx[USART_RX_BUFF_CNT];
q_node_type gbl_qdata_usart1_rx_free[USART_RX_BUFF_CNT];

char gbl_usart1_rx_buff[USART_RX_BUFF_CNT][USART_RX_DMA_SIZ];
char gbl_usart1_rx_buff_dma[USART_RX_DMA_SIZ];
#endif
static u16 s_usart1_tx_send_cnt = 0;
static u16 s_usart1_tx_q_free_cnt = 0;
static u16 s_usart1_tx_q_send_cnt = 0;

static s8 usart_format_buff[128];

#if 1
typedef struct
{
	u16 wr_idx;
	u16 rd_idx;
	char buffer[USART_RX_DMA_SIZ];
}usart_rx_type;

usart_rx_type gbl_usart1_rx_buff_proc;
#endif

uint16_t usart1_dma_transfering = FALSE;
//

static usartmode_type s_usart1_mode = usartmodeIRQ;
static usartmode_type s_usart2_mode = usartmodeIRQ;
static usartmode_type s_usart3_mode = usartmodeIRQ;

usart_service_function_type gbl_ar_usart_service[usartServiceFunctionMAX] = 
{
	{usart1ServiceFunction, NULL},
	{usart2ServiceFunction, NULL},
	{usart3ServiceFunction, NULL}
};

/* ------------------------------------------------------------------------------------------------- */
/* BSP USART */
/* ------------------------------------------------------------------------------------------------- */


void usart_transmit_byte( USART_TypeDef* port, u16 chr)
{
	USART_SendData(port, chr);
	while (USART_GetFlagStatus(port, USART_FLAG_TXE) == RESET)
		;		
}

void usart1_transmit_byte(u16 chr)
{
	usart_transmit_byte(USART1, chr);
}

void usart1_transmit_data(uint8_t* p_buf, uint16_t len) /* Add 2019.10.08 */
{
	uint16_t i;
	//uint16_t tmp_chr;
//	uint8_t tmp_chr;
	uint8_t sbuf[100] = {0};
	

	memcpy( sbuf,  p_buf, len);
	
	for(i = 0; i < len; i++)
	{
		//tmp_chr = (uint16_t)(*(p_buf + i));
		//tmp_chr = *(p_buf + i);
		//usart_transmit_byte(USART1, tmp_chr);
		usart_transmit_byte(USART1, sbuf[i]);
	}
}


//void bsp_init_irq_usart1(void/*isr_function usart1_isr*/)
void drv_usart1_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // CJK0820

//	init_usart1_buffer();

	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);		

	/* 
	   USART1_REMAP USART1 remapping
	   This bit is set and cleared by software. It controls the mapping of USART1 TX and RX alternate
	   functions on the GPIO ports.

	   0: No remap (TX/PA9, RX/PA10)
	   1: Remap (TX/PB6, RX/PB7)		
	*/
	// GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);	


	/* Configure the GPIO ports( USART1 Transmit and Receive Lines) */
	/* Configure the USART1_Tx as Alternate function Push-Pull */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	  
	/* Configure the USART1_Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure the USART1 */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	/* Enable the USART1 */
	USART_Cmd(USART1, ENABLE);	

	s_usart1_mode = usartmodeIRQ;

}



/* ------------------------------------------------------------------------------------------------- */
/* PC <-> Board Serial Communication																 */
/* ------------------------------------------------------------------------------------------------- */
void USART1_IRQHandler(void)
{
	u16 data_len;
//	q_node_type* q_usart_pkt_ptr;
	s8 data[1];

	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		data_len = 1;
		data[0] = USART_ReceiveData(USART1) & 0xFF;
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);	

		serial_comm_get_data(data[0] ); 
	}
}



void serial_comm_get_data(uint8_t recv_data)
{
	if(recv1_bytes.buf_use_flag == TRUE)
	{
		recv1_bytes.buf[recv1_bytes.buf_idx] = recv_data;
		

		if( (recv_data == '\n') && (recv1_bytes.buf[recv1_bytes.buf_idx -2] == '\r') )   	
		{
			serial_get_msg();
			serial_clear_recv_bytes();
		}
		else
		{
			recv1_bytes.buf_idx++;
		}
	}
	else
	{
		if(recv_data == 0x3A) 	/* : = 0x3A  = start */
		{
			recv1_bytes.buf_use_flag = TRUE;
			recv1_bytes.buf[recv1_bytes.buf_idx] = recv_data;
			recv1_bytes.buf_idx++;		
		}
	}

}


void serial_get_msg(void)
{
	uint16_t i;
	
	recv1_msg.buf_idx = 0;

	for(i = 0 ; i <= recv1_bytes.buf_idx; i++)
	{
		if( ( i % 2) == 0)
		{
			recv1_msg.buf[recv1_msg.buf_idx] = recv1_bytes.buf[i];
			recv1_msg.buf_idx++;
		}
	}

	recv1_msg.buf_use_flag = TRUE;
}


void serial_clear_recv_bytes(void)
{
	uint16_t i;
	
	recv1_bytes.buf_use_flag = FALSE;
	recv1_bytes.buf_idx = 0;
	
	for(i = 0; i < TRANSIVE_BUF_MAX; i++)
	{
		recv1_bytes.buf[i]  = 0;
	}
}


/*  communication to Optic Sensor */
//void bsp_init_irq_usart2(void/*isr_function usart1_isr*/)
void drv_usart2_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); //0828
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // CJK0805


	/* Enable the USART2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel 		= USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd 		= ENABLE;
	NVIC_Init(&NVIC_InitStructure);		

	/* 
	   USART1_REMAP USART1 remapping
	   This bit is set and cleared by software. It controls the mapping of USART1 TX and RX alternate
	   functions on the GPIO ports.

	   0: No remap (TX/PA9, RX/PA10)
	   1: Remap (TX/PB6, RX/PB7)		
	*/
	// GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);	


	/* Configure the GPIO ports( USART1 Transmit and Receive Lines) */
	/* Configure the USART2_Tx as Alternate function Push-Pull */
	GPIO_InitStructure.GPIO_Pin 		=  GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	  
	/* Configure the USART2_Rx as input floating */
	GPIO_InitStructure.GPIO_Pin 		= GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IN_FLOATING;   
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure the USART2 */
	USART_InitStructure.USART_BaudRate 	= 57600; 		
	USART_InitStructure.USART_WordLength 	= USART_WordLength_8b;
	USART_InitStructure.USART_StopBits 	= USART_StopBits_1;
	USART_InitStructure.USART_Parity 		= USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode 		= USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	USART_ITConfig(USART2, /*USART_IT_TXE |*/ USART_IT_RXNE, ENABLE);

	/* Enable the USART2 */
	USART_Cmd(USART2, ENABLE);	

	s_usart2_mode = usartmodeIRQ;

}



void USART2_IRQHandler(void)
{
	u16 data_len;
	s8 data[1];	

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		data_len = 1;
		data[0] = USART_ReceiveData(USART2) & 0xFF;
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);	

		// modbus_get_data(data[0] ); 
		mod_recevie_msg(0, data[0]);
	}
}

void usart2_transmit_byte(u16 chr)
{
	usart_transmit_byte(USART2, chr);
}

void usart2_transmit_string(char* data)
{
    while(*data != '\0')
	{
        usart2_transmit_byte(*(unsigned char *)data);
        data++;
    }
}


/*  communication to Optic Sensor */
void drv_usart3_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	

	/* Enable the USART2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);		

	/* 
	   USART1_REMAP USART1 remapping
	   This bit is set and cleared by software. It controls the mapping of USART1 TX and RX alternate
	   functions on the GPIO ports.

	   0: No remap (TX/PA9, RX/PA10)
	   1: Remap (TX/PB6, RX/PB7)		
	*/
	// GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);	

	/* Enable the TIM1 Pins Software Remapping */

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3,ENABLE);  

	/* Configure the GPIO ports( USART3 Transmit and Receive Lines) */
	/* Configure the USART3_Tx as Alternate function Push-Pull */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	  
	/* Configure the USART3_Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure the USART3 */
	USART_InitStructure.USART_BaudRate = 57600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  // USART_StopBits_1
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	/* Enable the USART3 */
	USART_Cmd(USART3, ENABLE);		

	s_usart3_mode = usartmodeIRQ;
}

void bsp_init_irq_usart3(void/*isr_function usart1_isr*/)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	

	/* Enable the USART2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);		

	/* 
	   USART1_REMAP USART1 remapping
	   This bit is set and cleared by software. It controls the mapping of USART1 TX and RX alternate
	   functions on the GPIO ports.

	   0: No remap (TX/PA9, RX/PA10)
	   1: Remap (TX/PB6, RX/PB7)		
	*/
	// GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);	


	/* Configure the GPIO ports( USART3 Transmit and Receive Lines) */
	/* Configure the USART3_Tx as Alternate function Push-Pull */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	  
	/* Configure the USART3_Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure the USART3 */
	USART_InitStructure.USART_BaudRate = BAUDRATE_485;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  // USART_StopBits_1
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	/* Enable the USART3 */
	USART_Cmd(USART3, ENABLE);		

	s_usart3_mode = usartmodeIRQ;

}



 

void register_usart_function(usart_register_function_type usart_fn_type, usart_register_function fn)
{
	gbl_ar_usart_service[usart_fn_type].run = fn;
}

void USART3_IRQHandler(void)
{
	u16 data_len;
	s8 data[1];	

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{

		data_len = 1;
		data[0] = USART_ReceiveData(USART3) & 0xFF;

		// uart3 echo test
		// usart1_transmit_byte(data[0]);

		if( gbl_ar_usart_service[usart3ServiceFunction].run != NULL )
		{
			gbl_ar_usart_service[usart3ServiceFunction].run(data[0]);
		}

		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		mod_recevie_msg(1, data[0]);
	}
}

void usart3_transmit_byte(u16 chr)
{
	usart_transmit_byte(USART3, chr);
}

void usart3_transmit_string(char* data)
{
    while(*data != '\0')
	{
        usart3_transmit_byte(*(unsigned char *)data);
        data++;
    }
}


