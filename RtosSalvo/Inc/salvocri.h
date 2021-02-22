/************************************************************
The contents of this file are subject to the Pumpkin Salvo
License (the "License").  You may not use this file except
in compliance with the License. You may obtain a copy of
the License at http://www.pumpkininc.com, or from
warranty@pumpkininc.com.

Software distributed under the License is distributed on an
"AS IS" basis, WITHOUT WARRANTY OF ANY KIND, either express
or implied. See the License for specific language governing
the warranty and the rights and limitations under the
License.

The Original Code is Salvo - The RTOS that runs in tiny
places(TM). Copyright (C) 1995-2008 Pumpkin, Inc. and its
Licensor(s). All Rights Reserved.

$Source: C:\\RCS\\D\\Pumpkin\\Salvo\\inc\\salvocri.h,v $
$Author: aek $
$Revision: 3.7 $
$Date: 2008-04-27 14:45:24-07 $

Salvo critical-section macros.

************************************************************/
#ifndef __SALVOCRI_H
#define __SALVOCRI_H

/************************************************************
****                                                     ****
**                                                         **
OSEnter|LeaveCritical():

Macros for entering and exiting critical regions of code.

These macros are normally mapped to user hooks.

**                                                         **
****                                                     ****
************************************************************/


/* 
 * All Salvo services with critical sections begin the 
 * critical section with OSEnterCritical().
 */ 
#if !defined(OSEnterCritical)
#define OSEnterCritical()          		do { OSDisableHook(); \
                                     	  } while (0);
#endif                                     	


/* 
 * All Salvo services with critical sections end the 
 * critical section with OSLeaveCritical().
 */ 
#if !defined(OSLeaveCritical)
#define OSLeaveCritical()          		do { OSEnableHook(); \
                                        } while (0);
#endif


#endif /* __SALVOCRI_H */
