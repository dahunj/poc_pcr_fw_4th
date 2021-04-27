

#ifndef DRV_RS485_H_
#define DRV_RS485_H_


#define RS485_EN_PORT  	GPIOC    
#define RS485_EN_PIN  	GPIO_Pin_14

void bsp_rs485_gpio_init(void);
void rs485_set_rx(void);
void rs485_set_tx(void);
void bsp_rs485_init(void);
void rs485_puts(char *data);

void USART3_IRQHandler(void);

#endif /* DRV_RS485_H_ */
