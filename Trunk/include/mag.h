/**********************************************************************************
* File name:  mag.h
*
* File description: 
*   -Header file for remote magnetometer interfacing
*               
* $Rev: 16002 $
* $Date: 2010-08-09 16:15:19 -0700 (Mon, 09 Aug 2010) $
* $Author: by-dan $
*
***********************************************************************************/
#ifndef MAG_H
#define MAG_H
         
extern BOOL resetMagFilter;
extern BOOL gRemoteMagDataRdy ;
extern remoteMagStruct gRemoteMagData;

extern void get_remotemagdata(sensorStruct *sensor); 

extern void remotemag_init(sensorStruct *sensor);   

#endif /* MAG_H */
