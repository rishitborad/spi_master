/** ***************************************************************************
 * @file   UserCommunication_SPI.c
 * @Author
 * @date   September, 2013
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * Set up Master SPI3 and user DEBUG USART
 ******************************************************************************/
#include <salvo.h>

#include "boardDefinition.h"
#include "timer.h"

#include "UserCommunication_SPI.h"
#include "spi.h"
#include "debug.h"

#include "stm32f2xx.h"
#include "stm32f2xx_spi.h"

/** ***************************************************************************
 * @name    _UserCommSpiDMADoneCallback()
 * @brief   callback for the spi done isr (set) the CS (A15) line high
 *
 * @param N/A
 * @retval N/A
 ******************************************************************************/
static void _UserCommSpiDMADoneCallback( void )
{
    GPIO_SetBits( SPI3_SLAVE_SELECT_PORT, SPI3_SLAVE_SELECT_PIN );
}

/** ***************************************************************************
 * @name    InitCommunication_UserSPI()
 * @brief   Initialize the user communication SPI line then set the CS high
 *
 * @param N/A
 * @retval 1 success
 ******************************************************************************/
uint8_t InitCommunication_UserSPI( void )
{
    uint8_t          error;
    GPIO_InitTypeDef GPIO_InitStructure;

    // ---- Initialize UARTB RX (pin D2) and SPI3 nSS (pin A15) as input pins,
    //      regardless of how SPI is configured (in 'spi_configure' below),
    //      neither interfere with each other ----
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;     // input/output/alt func/analog
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    // push-pull or open-drain
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL; // Up/Down/NoPull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // low/med/fast/high speed

    // A15 is the nominal SPI3 nSS pin
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init( GPIOA, &GPIO_InitStructure);

    // D2 is the nominal UART5 RX pin
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init( GPIOD, &GPIO_InitStructure);

    // ------ Set up the SPI3 IO pins, protocol, and interrupt ------
    error = spi_configure( kUserCommunicationSPI,
                           SPI_CPOL_AND_CPHA_HIGH,
                           &_UserCommSpiDMADoneCallback ); // local set CS high
    if( error ) {
        return error;
    }

    spi_go_faster( kUserCommunicationSPI ); // 0.5 Mhz start slow

    // Set the SPI nSS pin high and pause briefly
    GPIO_SetBits( SPI3_SLAVE_SELECT_PORT, SPI3_SLAVE_SELECT_PIN );
    DelayMs( 5 );

    return( 1 );
}

