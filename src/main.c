/*
******************************************************************************
File:     main.c
Info:     Generated by Atollic TrueSTUDIO(R) 9.3.0   2019-07-01

The MIT License (MIT)
Copyright (c) 2019 STMicroelectronics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

******************************************************************************
*/

/* Includes */
#include <stddef.h>
#include "stm32f10x.h"

#include "task.h"
#include "hw_config.h"

extern  stTaskType	task_desc[12];

#ifdef USE_STM3210B_EVAL
 #include "stm3210b_eval.h"
 #include "stm3210b_eval_lcd.h"
 #define USE_BOARD
 #define USE_LED
#elif defined USE_STM3210E_EVAL
 #include "stm3210e_eval.h"
 #include "stm3210e_eval_lcd.h"
 #define USE_BOARD
 #define USE_LED
#elif defined USE_STM3210C_EVAL
 #include "stm3210c_eval.h"
 #include "stm3210c_eval_lcd.h"
 #include "stm3210c_eval_i2c_ee.h"
 #define USE_BOARD
 #define USE_LED
 #define USE_SEE
#elif defined USE_STM32100B_EVAL
 #include "stm32100b_eval.h"
 #include "stm32100b_eval_lcd.h"
 #define USE_BOARD
 #define USE_LED
#elif defined USE_STM32100E_EVAL
 #include "stm32100e_eval.h"
 #include "stm32100e_eval_lcd.h"
 #include "stm32100e_eval_i2c_ee.h"
 #define USE_BOARD
 #define USE_LED
 #define USE_SEE
#elif defined USE_STM32_DISCOVERY
 #include "STM32vldiscovery.h"
#elif defined USE_IAR_STM32F103ZE
 #include "board.h"
 #define USE_LED
#elif defined USE_KEIL_MCBSTM32
 #include "board.h"
 #define USE_LED
#endif


/* Private typedef */
/* Private define  */
#ifdef USE_STM3210B_EVAL
  #define MESSAGE1   "STM32 Medium Density"
  #define MESSAGE2   " Device running on  "
  #define MESSAGE3   "   STM3210B-EVAL    "
  #define MESSAGE4   "                    "
#elif defined USE_STM3210E_EVAL
  #define MESSAGE1   " STM32 High Density "
  #define MESSAGE2   " Device running on  "
  #define MESSAGE3   "   STM3210E-EVAL    "
  #define MESSAGE4   "                    "
#elif defined USE_STM3210C_EVAL
  #define MESSAGE1   " STM32 Connectivity "
  #define MESSAGE2   " Line Device running"
  #define MESSAGE3   " on STM3210C-EVAL   "
  #define MESSAGE4   "                    "
#elif defined USE_STM32100B_EVAL
  #define MESSAGE1   "STM32 Medium Density"
  #define MESSAGE2   " Value Line Device  "
  #define MESSAGE3   "    running on      "
  #define MESSAGE4   "   STM32100B-EVAL   "
#elif defined USE_STM32100E_EVAL
  #define MESSAGE1   " STM32 High Density "
  #define MESSAGE2   " Value Line Device  "
  #define MESSAGE3   "    running on      "
  #define MESSAGE4   "   STM32100E-EVAL   "
#endif
  #define MESSAGE5   " program built with "
  #define MESSAGE6   " Atollic TrueSTUDIO "


/* Private macro */
/* Private variables */
 USART_InitTypeDef USART_InitStructure;

/* Private function prototypes */
/* Private functions */

/**
**===========================================================================
**
**  Program: POC_PCR_HD		Ver1.0
**
**===========================================================================
*/
int main(void)
{
  /**
  *  IMPORTANT NOTE!
  *  The symbol VECT_TAB_SRAM needs to be defined when building the project
  *  if code has been located to RAM and interrupts are used. 
  *  Otherwise the interrupt table located in flash will be used.
  *  See also the <system_*.c> file and how the SystemInit() function updates 
  *  SCB->VTOR register.  
  *  E.g.  SCB->VTOR = 0x20000000;  
  */

  	hw_init();
	queue_init();
	printf("task init..\n");
	task_init();
	task_run();

}





#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/*
 * Minimal __assert_func used by the assert() macro
 * */
void __assert_func(const char *file, int line, const char *func, const char *failedexpr)
{
  while(1)
  {}
}

/*
 * Minimal __assert() uses __assert__func()
 * */
void __assert(const char *file, int line, const char *failedexpr)
{
   __assert_func (file, line, NULL, failedexpr);
}

#ifdef USE_SEE
#ifndef USE_DEFAULT_TIMEOUT_CALLBACK
/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval sEE_FAIL.
  */
uint32_t sEE_TIMEOUT_UserCallback(void)
{
  /* Return with error code */
  return sEE_FAIL;
}
#endif
#endif /* USE_SEE */

