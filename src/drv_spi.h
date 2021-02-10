/*
 * drv_spi.h
 *
 *  Created on: 2019. 7. 29.
 *      Author: jk.choi
 */

#ifndef DRV_SPI_H_
#define DRV_SPI_H_


#include "stm32f1xx_hal_spi.h"

/* ------------------------------------------------------------------------------------------------- */
/* BSP SPI */
/* ------------------------------------------------------------------------------------------------- */

#define SPI_SPEED_2   0
#define SPI_SPEED_4   1
#define SPI_SPEED_8   2
#define SPI_SPEED_16  3
#define SPI_SPEED_256 4

#define SPI_SPEED_LOW   0
#define SPI_SPEED_HIGH  1

                          
#define GPIO_CRL_MODE0_Pos                   (0U)                              
#define GPIO_CRL_MODE0_Msk                   (0x3U << GPIO_CRL_MODE0_Pos)      /*!< 0x00000003 */
#define GPIO_CRL_MODE0                       GPIO_CRL_MODE0_Msk                /*!< MODE0[1:0] bits (Port x mode bits, pin 0) */
#define  GPIO_SPEED_FREQ_HIGH             (GPIO_CRL_MODE0)   /*!< High speed */

#define  GPIO_MODE_INPUT                        0x00000000U   /*!< Input Floating Mode                   */
#define  GPIO_MODE_AF_PP                        0x00000002U   /*!< Alternate Function Push Pull Mode     */
#define  GPIO_NOPULL        0x00000000U   /*!< No Pull-up or Pull-down activation  */

#define GPIO_NUMBER           16U
#define IS_GPIO_ALL_INSTANCE(INSTANCE) (((INSTANCE) == GPIOA) || \
                                        ((INSTANCE) == GPIOB) || \
                                        ((INSTANCE) == GPIOC) || \
                                        ((INSTANCE) == GPIOD) || \
                                        ((INSTANCE) == GPIOE))

/**************************** GPIO Alternate Function Instances ***************/
#define IS_GPIO_AF_INSTANCE(INSTANCE) IS_GPIO_ALL_INSTANCE(INSTANCE)

#define  GPIO_MODE_INPUT                        0x00000000U   /*!< Input Floating Mode                   */
#define  GPIO_MODE_OUTPUT_PP                    0x00000001U   /*!< Output Push Pull Mode                 */
#define  GPIO_MODE_OUTPUT_OD                    0x00000011U   /*!< Output Open Drain Mode                */
#define  GPIO_MODE_AF_PP                        0x00000002U   /*!< Alternate Function Push Pull Mode     */
#define  GPIO_MODE_AF_OD                        0x00000012U   /*!< Alternate Function Open Drain Mode    */
#define  GPIO_MODE_AF_INPUT                     GPIO_MODE_INPUT          /*!< Alternate Function Input Mode         */

/* Definitions for bit manipulation of CRL and CRH register */
#define  GPIO_CR_MODE_INPUT         0x00000000U /*!< 00: Input mode (reset state)  */
#define  GPIO_CR_CNF_ANALOG         0x00000000U /*!< 00: Analog mode  */
#define  GPIO_CR_CNF_INPUT_FLOATING 0x00000004U /*!< 01: Floating input (reset state)  */
#define  GPIO_CR_CNF_INPUT_PU_PD    0x00000008U /*!< 10: Input with pull-up / pull-down  */
#define  GPIO_CR_CNF_GP_OUTPUT_PP   0x00000000U /*!< 00: General purpose output push-pull  */
#define  GPIO_CR_CNF_GP_OUTPUT_OD   0x00000004U /*!< 01: General purpose output Open-drain  */
#define  GPIO_CR_CNF_AF_OUTPUT_PP   0x00000008U /*!< 10: Alternate function output Push-pull  */
#define  GPIO_CR_CNF_AF_OUTPUT_OD   0x0000000CU /*!< 11: Alternate function output Open-drain  */

#define  GPIO_MODE_IT_RISING                    0x10110000U   /*!< External Interrupt Mode with Rising edge trigger detection          */
#define  GPIO_MODE_IT_FALLING                   0x10210000U   /*!< External Interrupt Mode with Falling edge trigger detection         */
#define  GPIO_MODE_IT_RISING_FALLING            0x10310000U   /*!< External Interrupt Mode with Rising/Falling edge trigger detection  */

#define  GPIO_MODE_EVT_RISING                   0x10120000U   /*!< External Event Mode with Rising edge trigger detection               */
#define  GPIO_MODE_EVT_FALLING                  0x10220000U   /*!< External Event Mode with Falling edge trigger detection              */
#define  GPIO_MODE_EVT_RISING_FALLING           0x10320000U   /*!< External Event Mode with Rising/Falling edge trigger detection       */
#define  GPIO_NOPULL        0x00000000U   /*!< No Pull-up or Pull-down activation  */
#define  GPIO_PULLUP        0x00000001U   /*!< Pull-up activation                  */
#define  GPIO_PULLDOWN      0x00000002U   /*!< Pull-down activation                */
#define  GPIO_MODE_ANALOG                       0x00000003U   /*!< Analog Mode  */

#define IS_GPIO_PULL(PULL) (((PULL) == GPIO_NOPULL) || ((PULL) == GPIO_PULLUP) || \
                            ((PULL) == GPIO_PULLDOWN))

#define GPIO_PIN_8                 ((uint16_t)0x0100)  /* Pin 8 selected    */
#define EXTI_MODE             0x10000000U

#define __HAL_RCC_AFIO_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define GPIO_GET_INDEX(__GPIOx__) (((__GPIOx__) == (GPIOA))? 0U :\
                                   ((__GPIOx__) == (GPIOB))? 1U :\
                                   ((__GPIOx__) == (GPIOC))? 2U :\
                                   ((__GPIOx__) == (GPIOD))? 3U :4U)
#define GPIO_MODE_IT          0x00010000U
#define GPIO_MODE_EVT         0x00020000U
#define RISING_EDGE           0x00100000U
#define FALLING_EDGE          0x00200000U
#define GPIO_OUTPUT_TYPE      0x00000010U


#define GPIO_PIN_0                 ((uint16_t)0x0001)  /* Pin 0 selected    */
#define GPIO_PIN_1                 ((uint16_t)0x0002)  /* Pin 1 selected    */
#define GPIO_PIN_2                 ((uint16_t)0x0004)  /* Pin 2 selected    */
#define GPIO_PIN_3                 ((uint16_t)0x0008)  /* Pin 3 selected    */
#define GPIO_PIN_4                 ((uint16_t)0x0010)  /* Pin 4 selected    */
#define GPIO_PIN_5                 ((uint16_t)0x0020)  /* Pin 5 selected    */
#define GPIO_PIN_6                 ((uint16_t)0x0040)  /* Pin 6 selected    */
#define GPIO_PIN_7                 ((uint16_t)0x0080)  /* Pin 7 selected    */
#define GPIO_PIN_8                 ((uint16_t)0x0100)  /* Pin 8 selected    */
#define GPIO_PIN_9                 ((uint16_t)0x0200)  /* Pin 9 selected    */
#define GPIO_PIN_10                ((uint16_t)0x0400)  /* Pin 10 selected   */
#define GPIO_PIN_11                ((uint16_t)0x0800)  /* Pin 11 selected   */
#define GPIO_PIN_12                ((uint16_t)0x1000)  /* Pin 12 selected   */
#define GPIO_PIN_13                ((uint16_t)0x2000)  /* Pin 13 selected   */
#define GPIO_PIN_14                ((uint16_t)0x4000)  /* Pin 14 selected   */
#define GPIO_PIN_15                ((uint16_t)0x8000)  /* Pin 15 selected   */
#define GPIO_PIN_All               ((uint16_t)0xFFFF)  /* All pins selected */
uint32_t  spi_temp_u32;


#define UNUSED(X) (void)X      /* To avoid gcc/g++ warnings */

#define __HAL_RCC_SPI1_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)

#define __HAL_RCC_GPIOA_CLK_ENABLE()   do { \
                                        __IO uint32_t tmpreg; \
                                        SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);\
                                        /* Delay after an RCC peripheral clock enabling */\
                                        tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);\
                                        UNUSED(tmpreg); \
                                      } while(0U)
                                      
typedef enum
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;


typedef struct 
{
	uint32_t spi_temp_u32;
	uint8_t   spi_temp_flag;
	
}st_SPIdata;


st_SPIdata spi_data;


typedef struct
{
  uint32_t Pin;       /*!< Specifies the GPIO pins to be configured.
                           This parameter can be any value of @ref GPIO_pins_define */

  uint32_t Mode;      /*!< Specifies the operating mode for the selected pins.
                           This parameter can be a value of @ref GPIO_mode_define */

  uint32_t Pull;      /*!< Specifies the Pull-up or Pull-Down activation for the selected pins.
                           This parameter can be a value of @ref GPIO_pull_define */

  uint32_t Speed;     /*!< Specifies the speed for the selected pins.
                           This parameter can be a value of @ref GPIO_speed_define */
} GPIO_InitTypeDef_;

void drv_spi_init(void);
extern void drv_set_spi1_speed (u8 speed);
extern u8 drv_readwritebyte_spi1 (u8 tx_data);
extern void drv_set_spi2_speed (u8 speed);
extern void drv_set_spi2_speed_mp3(u8 SpeedSet);
extern u8 drv_readwritebyte_spi2 (u8 tx_data);
u8 bsp_readwritebyte_spi2 (u8 tx_data);
void bsp_touch_spi_init(void);
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi);
void MX_SPI1_Init(void);
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi);

#endif /* DRV_SPI_H_ */
