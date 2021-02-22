/*****************************************************************************
* File name:  xbowsp_init.h
*
* File description: 
*   -header file of xbowsp_init.c
*
* $Rev: 17479 $
* $Date: 2011-02-09 22:02:39 -0800 (Wed, 09 Feb 2011) $
* $Author: by-dan $
*
***********************************************************************************/

#ifndef XBOWSP_INIT_H
#define XBOWSP_INIT_H

#include "xbowsp_generaldrivers.h"
#include "xbowsp_algorithm.h"
extern void initConfigureUnit(void);   
extern BOOL ValidPortConfiguration (ConfigurationStruct *proposedConfiguration);

extern AlgorithmStruct gAlgorithm;
extern softwareVersionStruct bootFMversion; 
extern ConfigurationStruct gConfiguration;
extern CalibrationStruct gCalibration;
extern softwareVersionStruct dupFMversion;
extern softwareVersionStruct ioupFMversion;   
                                  

extern void initAlgStruct(void);


#endif
