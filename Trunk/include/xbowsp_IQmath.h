/***************************************************************************** 
* File name:  xbowsp_IQmath.h
*
* File description: 
*   -header file of xbowsp_IQmath.c
*
* $Rev: 17465 $
* $Date: 2011-02-09 20:20:16 -0800 (Wed, 09 Feb 2011) $
* $Author: by-denglish $
*
***********************************************************************************/
#if ! defined IQMATH_H
#define IQMATH_H


typedef long IQ27;

#define IQ27_TO_DOUBLE  7.450580596923828e-009   /*32/(2^32)   */

extern double IQ27toF (IQ27 tmp); 
 



#endif