/**********************************************************************************
* File name:  s_eeprom.h
*
* File description: 
*   -Header file for s_eeprom.c, simulated spi eeprom
*               
*************************************/
#ifndef S_EEPROM_H
#define S_EEPROM_H

extern void readEEPROMWords(uint16_t addr, uint16_t num, void *destination) ;
extern BOOL writeEEPROMWords(uint16_t addr, uint16_t num, void *source) ;
extern void readEEPROMByte(uint16_t addr, uint16_t num, void *destination) ;
extern BOOL writeEEPROMByte(uint16_t addr, uint16_t num, void *source) ;
extern BOOL write8BitsBytes2seprom(unsigned int addr, unsigned char *source, int num);

#endif /* S_EEPROM_H */ 


