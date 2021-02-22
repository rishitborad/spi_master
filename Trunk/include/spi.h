/** ****************************************************************************
 * @file spi.h
 * @author jsun
 * @date: 2011-02-10 11:42:26 -0800 (Thu, 10 Feb 2011) $
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 * @brief description: This SPI driver sets up a DMA interrupt to transfer data
 * back and forth
 *
 * $Revision: 15866 $
 ******************************************************************************/
#ifndef __SPI_H
#define __SPI_H

#include "stdint.h"
#include "stm32f2xx.h"
#include "stm32f2xx_spi.h"

#define SPI_CPOL_AND_CPHA_HIGH  1
#define SPI_CPOL_AND_CPHA_LOW   0

#define  HOLD_CS    0
#define  TOGGLE_CS  1

uint8_t spi_configure(SPI_TypeDef* SPIx, uint8_t cpolAndCphaHigh, void (*callback)(void));

void spi_go_really_slow(SPI_TypeDef* SPIx);
void spi_go_slow(SPI_TypeDef* SPIx);
void spi_go_fast(SPI_TypeDef* SPIx);
void spi_go_faster(SPI_TypeDef* SPIx);
void spi_go_even_faster(SPI_TypeDef* SPIx);
void spi_go_slightly_faster( SPI_TypeDef* SPIx );
void spi_go_super_fast(SPI_TypeDef* SPIx);

void ReadFromRegisters( uint8_t startingAddress,
                        uint8_t   endingAddress,
                        uint16_t  *RxBuffer,
                        uint16_t  delayCounts,
                        uint8_t   toggleOrHoldFlag );
/*
void ReadFromRegistersNoChipSelect( uint8_t startingAddress,
                         uint8_t endingAddress,
                         uint16_t *RxBuffer,
                         uint16_t delayCounts );
*/
void ReadFromRegisters_noChipSelect( uint8_t startingAddress,
                         uint8_t endingAddress,
                         uint16_t *RxBuffer,
                         uint16_t delayCounts );

void BurstReadFromRegisters( uint8_t startingAddress,
                              uint8_t endingAddress,
                              uint16_t *RxBuffer,
                              uint16_t delayCounts );

uint16_t WriteToRegister( uint8_t registerAddress,
                           uint8_t dataByte );

uint16_t WriteBlock(uint16_t regAddress, uint16_t blockNum, 
                       uint8_t *txBuffer, uint16_t length);

uint16_t ReadFromRegister(uint16_t regAddress);
uint16_t WriteWord(uint16_t DataWord, uint8_t toggleCS);
uint16_t SendCommand( uint8_t registerAddress, uint8_t dataByte );


#define SPI_ERROR_GENERIC 1
#define SPI_NO_ERROR 0
#endif
