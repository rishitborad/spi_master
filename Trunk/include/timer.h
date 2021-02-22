/******************************************************************************
* File name:	
*	timer.h
*
* File description:
*	timer head file
*
* $Revision: 15722 $
*
* $Date: 2011-01-21 01:24:06 -0800 (Fri, 21 Jan 2011) $
*
* $Author: whpeng $
 ******************************************************************************/

#ifndef __TIMER_H
#define __TIMER_H

#include "stdint.h"
#include "stm32f2xx.h"

typedef struct
{
  uint32_t seconds;
  uint32_t milliseconds;
} sys_time_t;

typedef uint32_t tTime;
#define TIME_MAX UINT32_MAX

void DelayMs( tTime delay);
tTime TimeNow();
tTime TimePassed(tTime since);
void InitSystemTimer();
uint32_t GetSysTime();

#endif
