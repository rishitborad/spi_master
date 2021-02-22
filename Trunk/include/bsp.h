/******************************************************************************
* File name: bsp.h
*
* File description:
*	       Board Support package, configure the Cortex M3
*
* $Rev: 15722 $
* $Date: 2011-01-21 01:24:06 -0800 (Fri, 21 Jan 2011) $
* $Author: whpeng $
 ******************************************************************************/

#ifndef CONF_GPIO_H
#define  CONF_GPIO_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes  */
#include "stm32f2xx.h"
#include "data_type.h"
#include "timer.h"
#include "iar_stm32f207zg_sk.h"

/* Set the number of ticks in one second    */
#define OS_TICKS_PER_SEC        (1000)

#define SERVOMID		(75)

/* declare  Functions */ 
void led_on(Led_TypeDef led);
void led_off(Led_TypeDef led);
void led_toggle(Led_TypeDef led);

void BSP_init(void);
void RCC_config(void);
void GPIO_config(void);
void NVIC_config(void);
void DelayMs(uint32_t dly_ticks);

#ifdef __cplusplus
}
#endif
  
#endif /* CONF_GPIO_H */

