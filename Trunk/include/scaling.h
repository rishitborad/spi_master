/*********************************************************************************
* File name:  scaling.h
*
* File description: 
*   - scaling constants
*               
* $Rev: 16236 $
* $Date: 2010-08-30 15:04:41 -0700 (Mon, 30 Aug 2010) $
* $Author: by-tdelong $
*
*********************************************************************************/

#ifndef SCALING_H
#define SCALING_H 

// math constants
#define TWOPI 6.28318530718
#define PI  3.1415926535898
#define GRAVITY    9.80665 
 
/* scaling conversions */ 
#define SCALE_BY_8(value)					((value) * 8)
#define SCALE_BY_2POW16_OVER_2PI(value)		((value) * (65536 / (2.0 * PI)))
#define SCALE_BY_2POW16_OVER_7PI(value)     ((value) * (65536 / (7.0 * PI))) 
#define SCALE_BY_2POW16_OVER_2(value)       ((value) * (65536 / 2))
#define SCALE_BY_2POW16_OVER_20(value)      ((value) * (65536 / 20))  
#define SCALE_BY_2POW16_OVER_64(value)      ((value) * (65536 / 64))
#define SCALE_BY_2POW16_OVER_128(value)     ((value) * (65536 / 128))
#define SCALE_BY_2POW16_OVER_200(value)     ((value) * (65536 / 200))
#define SCALE_BY_2POW16_OVER_512(value)     ((value) * (65536 / 512)) 
 
#define MAXUINT16_OVER_2PI ( MAXUINT16 / TWOPI)	   /* 10430.3783505 */
#define MAXINT16_OVER_2PI  ( MAXINT16 / TWOPI)
#define MAXUINT16_OVER_512 ( MAXUINT16 / 512.0)	   /* 128.0 */   
#define MAXUINT16_OVER_2   ( MAXUINT16 / 2.0)	   /* 32768.0 */ 
#define DEGREES_TO_RADS    ( TWOPI / 360.0)

#define MAXINT32_20BIT_OVER131072M  8  			   /* 2^20/(2^17) */
 
#endif 
 
