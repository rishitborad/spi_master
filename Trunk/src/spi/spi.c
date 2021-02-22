/** ****************************************************************************
 * @file spi.c
 * @author jsun
 * @date: 2011-02-10 11:42:26 -0800 (Thu, 10 Feb 2011) $
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 * @brief description: This SPI driver sets up an interrupt to transfer data
 * back and forth across the bus
 *
 * $Revision: 15866 $
 ******************************************************************************/
#include <stdint.h>
#include <salvo.h>
#include "dmu.h"
#include "boardDefinition.h"

#include "stm32f2xx_dma.h"
#include "stm32f2xx_spi.h"
#include "UserCommunication_SPI.h" // set up for DMU380
#include "debug.h" // for debug console output

#include "xbowsp_version.h"
#include "spi.h"

// Defined in commands.c
// used in stm32f2xx_it.c
extern volatile uint8_t CmdTransmitted;
extern volatile uint8_t Tx_Idx;
extern volatile uint8_t CmdStatus;

extern uint8_t TxBuffer[];

static uint16_t _crc = 0x1D0F;

void _initCRC(uint16_t val)
{
    _crc = val;
}

/** ***************************************************************************
 * @name _calcCRC()
 * @brief Calculate the 16-bit CRC for the burst register
 *
 * @param [in] buf - input buffer to calculate the CRC
 * @param [in] num - size of the buffer
 * @retval CRC
 ******************************************************************************/
uint16_t _calcCRC( volatile uint8_t *buf,
                   uint16_t         num )
{
    int      i;
    int      j;

    for( i = 0; i < num; i++ ) {
        _crc ^= buf[i] << 8;

        for( j = 0; j < 8; j++ ) {
            if(_crc & 0x8000) {
                _crc = (_crc << 1) ^ 0x1021;
            } else {
                _crc = _crc << 1;
            }
        }
    }
    return _crc;
}




extern UserSpiStruct gUserSpi;

void (*gSPI3CallbackOnDMAComplete)(void) = NULL;

int gSPI3Complete = FALSE;

/** ***************************************************************************
 * @name    _SPI3_Init()
 * @brief The following function sets up the user communication protocol over
 * the MASTER SPI3 interface
 *
 * @param [in] cpolAndCphaHigh -set cpol high or low
 * @retval success = 0
 ******************************************************************************/
uint8_t _SPI3_Init( uint8_t cpolAndCphaHigh )
{
    // Define setup variables used in initialization functions
    SPI_InitTypeDef  SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    SPI_DeInit( SPI3 ); //  Reset SPI parameters to their default reset values
    SPI_Cmd( SPI3, DISABLE ); //  Disable SPI3 during configuration

    // ---------- Peripheral Clock Enable ----------
    // Enable the SPI clock
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI3, ENABLE );

    // Enable GPIO clocks
    RCC_AHB1PeriphClockCmd( SPI3_GPIO_MOSI_CLK, ENABLE );
    RCC_AHB1PeriphClockCmd( SPI3_GPIO_MISO_CLK, ENABLE );
    RCC_AHB1PeriphClockCmd( SPI3_GPIO_SCK_CLK,  ENABLE );
    RCC_AHB1PeriphClockCmd( SPI3_SLAVE_SELECT_CLK, ENABLE );

    // Enable DMA clock
    RCC_AHB1PeriphClockCmd( SPI3_DMA_CLK, ENABLE );

    // ---------- SPI GPIO Configuration ----------
    // Connect pins to AF6 (GPIO alternate function)
    GPIO_PinAFConfig( SPI3_MOSI_PORT, SPI3_MOSI_SOURCE, GPIO_AF_SPI3 );
    GPIO_PinAFConfig( SPI3_MISO_PORT, SPI3_MISO_SOURCE, GPIO_AF_SPI3 );
    GPIO_PinAFConfig( SPI3_SCK_PORT,  SPI3_SCK_SOURCE,  GPIO_AF_SPI3 );

    // Configure GPIO pins
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;     // input/output/alt func/analog
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    // push-pull or open-drain
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;   // Up/Down/NOPULL
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // low/med/fast/high speed

    GPIO_InitStructure.GPIO_Pin = SPI3_MOSI_PIN; // MOSI
    GPIO_Init( SPI3_MOSI_PORT, &GPIO_InitStructure );
    GPIO_InitStructure.GPIO_Pin = SPI3_MISO_PIN; // MISO
    GPIO_Init( SPI3_MISO_PORT, &GPIO_InitStructure );

    // Initialize the clk pin with a pull-up setting
    GPIO_InitStructure.GPIO_Pin  = SPI3_SCK_PIN;  // SCK
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;   // Up/Down/NOPULL
    GPIO_Init( SPI3_SCK_PORT, &GPIO_InitStructure );

    // Initialize the nSS pin as on output pin (not an AF pin)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; // input/OUTPUT/alt func/analog
    GPIO_InitStructure.GPIO_Pin  = SPI3_SLAVE_SELECT_PIN; // nSS
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init( SPI3_SLAVE_SELECT_PORT, &GPIO_InitStructure );

    // ---------- SPI GPIO Configuration ----------
    // Call the init structure function to set the values to default. Change
    // individual options as needed
    SPI_StructInit( &SPI_InitStructure );
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    // Full-duplex communication
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b; // Bits per transfer
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; // Transmit MSB first
    // CPOL/CPHA
    if ( cpolAndCphaHigh ) {
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    } else {
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    }
    // nSS mode
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;

    // fPCLK / baudrate
    // fPCLK is the APB1 clock frequency, which is currenly SystemCoreClock
    // (120MHz).  The MAX21000 can run at 10MHz but SPI mode starts slow and
    //  speeds up. start at ~0.25Mhz
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
	SPI_Init( SPI3, &SPI_InitStructure );

    // Disable the TXE interrupt when the device is a master
    //SPI_I2S_ITConfig( SPI3, SPI_I2S_IT_TXE, DISABLE );

    // ---------- Re-enable the SPI peripheral ----------
    SPI_Cmd( SPI3, ENABLE ); // stm32f2xx_spi.c

    return SPI_NO_ERROR;
}

/** ***************************************************************************
 * @name    spi_configure()
 * @brief Configure the SPI interface via the initialization functions
 * (defined above) in addition to the arguments passed to this function (used to
 * set CPOL/CHPA and callback, which defines the callback function that executes
 * upon DMA complete)
 *
 * @param [in] SPIx - number of times to blink
 * @param [in] cpolAndCphaHigh - speed of the blinks
 * @param [in] isr callback
 * @retval succes fail
 ******************************************************************************/
uint8_t spi_configure(SPI_TypeDef* SPIx,
                      uint8_t     cpolAndCphaHigh,
                      void (*callback)(void))
{
     if( SPIx == SPI3 ) {
         // Initialize user-communication SPI interface
         gSPI3CallbackOnDMAComplete = callback;
         return _SPI3_Init( cpolAndCphaHigh );
     }

    return SPI_ERROR_GENERIC;
}

void SPI3_IRQHandler( void )
{
    OSDisableHook();
    // If the device is a slave device, do this...
    // TXE interrupt for the master goes here (if used)
    OSEnableHook();
}

/** ***************************************************************************
 * @name    _spi_set_baud() LOCAL
 * @brief  read in a 16-bit word over the SPI
 *
 * @param [in] SPIx - SPI bus selected
 * @param [in] baudRate - baudrate clock pre scaler
 * @retval N/A
 ******************************************************************************/
void _spi_set_baud(SPI_TypeDef* SPIx,
                   uint16_t     baudRate)
{
    uint16_t       tmpreg = 0;
    const uint16_t baudRateMask = SPI_BaudRatePrescaler_256;

    __disable_irq();

    if ( SPIx == SPI1 )
        SPI_Cmd( SPI1, DISABLE );
    else if( SPIx == SPI3 )
        SPI_Cmd( SPI3, DISABLE );

    tmpreg    = SPIx->CR1;
    tmpreg   &= ~baudRateMask;
    tmpreg   |= baudRate;
    SPIx->CR1 = tmpreg;

    if ( SPIx == SPI1 )
        SPI_Cmd( SPI1, ENABLE );
    else if( SPIx == SPI3 )
        SPI_Cmd( SPI3, ENABLE );

    __enable_irq();
}

void spi_go_super_fast(SPI_TypeDef* SPIx)
{
    // with clock at 120Hz, this is ~8Mhz
    // baud rate is clock/(2*prescaler)
    _spi_set_baud(SPIx, SPI_BaudRatePrescaler_4);    // 8.0 MHz
}

void spi_go_fast(SPI_TypeDef* SPIx)
{
    // with clock at 120Hz, this is ~8Mhz
    // baud rate is clock/(2*prescaler)
    _spi_set_baud(SPIx, SPI_BaudRatePrescaler_8);    // 4.0 MHz
}

void spi_go_even_faster( SPI_TypeDef* SPIx )
{
    // clock is 120Hz, baud rate is clock/(2*prescaler)
    _spi_set_baud(SPIx, SPI_BaudRatePrescaler_16);    // 2.0 MHz
}

/** ***************************************************************************
 * @name    spi_go_slightly_faster()
 * @brief  set speed to 1Mhz
 * clock is 120Hz, baud rate is clock/(2*prescaler)
 * @param [in] SPIx - SPI bus selected
 * @retval N/A
 ******************************************************************************/
void spi_go_slightly_faster( SPI_TypeDef* SPIx )
{
    _spi_set_baud(SPIx, SPI_BaudRatePrescaler_32);    // 1.0 MHz
}

/** ***************************************************************************
 * @name    spi_go_faster()
 * @brief  set speed to 0.5Mhz
 * InitCommunication_UserSPI() - system starts slow
 * CmdUserSpi_jdInitProcess()
 *
 * @param [in] SPIx - SPI bus selected
 * @retval N/A
 ******************************************************************************/
void  spi_set_speed_div(SPI_TypeDef* SPIx, int div)
{
    switch(div){
    case 128:
        _spi_set_baud(SPIx, SPI_BaudRatePrescaler_128);
        break;
    case 64:
        _spi_set_baud(SPIx, SPI_BaudRatePrescaler_64);
        break;
    case 32:
        _spi_set_baud(SPIx, SPI_BaudRatePrescaler_32);
        break;
    case 16:
        _spi_set_baud(SPIx, SPI_BaudRatePrescaler_16);
        break;
    case 8:
        _spi_set_baud(SPIx, SPI_BaudRatePrescaler_8);
        break;
    }
    _spi_set_baud(SPIx, SPI_BaudRatePrescaler_128);
}

void spi_go_faster(SPI_TypeDef* SPIx)
{
    // with clock at 120Hz, this is ~0.5Mhz
    // baud rate is clock/(2*prescaler)
    _spi_set_baud(SPIx, SPI_BaudRatePrescaler_64);
}

void  spi_go_slow(SPI_TypeDef* SPIx)
{
    // with clock at 120Hz, this is ~0.25Mhz
    // baud rate is clock/(2*prescaler)
    _spi_set_baud(SPIx, SPI_BaudRatePrescaler_128);
}

void  spi_go_really_slow(SPI_TypeDef* SPIx)
{
    // with clock at 120Hz, this is ~0.125Mhz
    // baud rate is clock/(2*prescaler)
    _spi_set_baud( SPIx, SPI_BaudRatePrescaler_256 );
}

/** ***************************************************************************
 * @name ReadFromRegisters()
 * @brief  read in a 16-bit word over the SPI NON-DMA
 *
 * @param [in] startingAddress - SPI register address start
 * @param [in] endingAddress - SPI register end
 * @param [in] RxBuffer - buffer to hold incomming data
 * @param [in] delay counts - for spin delay loop
 * @retval N/A
 ******************************************************************************/
void ReadFromRegisters( uint8_t startingAddress,
                        uint8_t endingAddress,
                        uint16_t *RxBuffer,
                        uint16_t delayCounts,
                        uint8_t toggleOrHoldFlag )
{
    uint16_t DataWord;      // hold the shift-register data (SPI->DR)
    uint8_t  RegAddr   = 0; // Register address
    uint8_t  wordCount = 0;
    int      i;

    // Disable interrupts
    OSDisableHook();

    /** Send out the read command for the selected data registers - starting
         addresses must be even and two bytes are clocked out at a time (MSB and
         LSB), which is the reason for incrementing the register address by 0x02
         each time.  The loop continues until the register address is two-bytes
         beyond the ending address.  This is due to the extra two-bytes needed
         to clock out the final register information. */
    for( RegAddr = startingAddress; RegAddr <= ( endingAddress + 0x2 ); RegAddr = RegAddr + 0x2 ) {
        // Set CS low.  If toggling the CS then set the pin high after the read
        //   (below).
        SPI3_SLAVE_SELECT_PORT->BSRRH = SPI3_SLAVE_SELECT_PIN;

        // 2) wait for TXE
        while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_TXE ) == RESET )
        {  /* spin */; }
        // 3) Write to DR (the register address - 16 bits) or 2 zero-bytes to complete the read.
        if( RegAddr == ( endingAddress + 0x2 ) ) {
            kUserCommunicationSPI->DR = endingAddress << 8;
        } else {
            // Shift the address to form the 16-bit word (padded by 8 zeros)
            kUserCommunicationSPI->DR = RegAddr << 8;
        }
        // 4) Wait for RXNE before reading DR
        while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_RXNE ) == RESET )
        {  /* spin */; }
        DataWord = kUserCommunicationSPI->DR; // Read the DR
        // 6) Wait for the transfer to finish
        while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_BSY ) == SET )
        {  /* spin */; }
        if( toggleOrHoldFlag == TOGGLE_CS ) {
            SPI3_SLAVE_SELECT_PORT->BSRRL = SPI3_SLAVE_SELECT_PIN; // CS high - deselect
        }
        RxBuffer[ wordCount ] = DataWord; // return buffer
        wordCount++;

        for( i = 0; i <= delayCounts; i++ ) { DataWord = 0x0000; /* spin */}
    } // End of register read (for-loop)

    // Set chip-select high (regardless of toggling setting) end re-enable
    //   interrupts
    SPI3_SLAVE_SELECT_PORT->BSRRL = SPI3_SLAVE_SELECT_PIN; // CS high - deselect
    OSEnableHook(); // Enable interrupts
}

/** ***************************************************************************
 * @name ReadFromRegisters()
 * @brief  read in a 16-bit word over the SPI NON-DMA
 *
 * @param [in] startingAddress - SPI register address start
 * @param [in] endingAddress - SPI register end
 * @param [in] RxBuffer - buffer to hold incomming data
 * @param [in] delay counts - for spin delay loop
 * @retval N/A
 ******************************************************************************/
void ReadFromRegisters_noChipSelect( uint8_t startingAddress,
                         uint8_t endingAddress,
                         uint16_t *RxBuffer,
                         uint16_t delayCounts )
{
    uint16_t DataWord;      // hold the shift-register data (SPI->DR)
    uint8_t  RegAddr   = 0; // Register address
    uint8_t  wordCount = 0;
    int      i;

    OSDisableHook(); // Disable interrupts
    /** Send out the read command for the selected data registers - starting
         addresses must be even and two bytes are clocked out at a time (MSB and
         LSB), which is the reason for incrementing the register address by 0x02
         each time.  The loop continues until the register address is two-bytes
         beyond the ending address.  This is due to the extra two-bytes needed
         to clock out the final register information. */
    for( RegAddr = startingAddress; RegAddr <= ( endingAddress + 0x2 ); RegAddr = RegAddr + 0x2 ) {

//        SPI3_SLAVE_SELECT_PORT->BSRRH = SPI3_SLAVE_SELECT_PIN; // CS low
        // 2) wait for TXE
        while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_TXE ) == RESET )
        {  /* spin */; }
        // 3) Write to DR (the register address - 16 bits) or 2 zero-bytes to complete the read.
        if( RegAddr == ( endingAddress + 0x2 ) ) {
            kUserCommunicationSPI->DR = 0x0000;
        } else {
            // Shift the address to form the 16-bit word (padded by 8 zeros)
            kUserCommunicationSPI->DR = RegAddr << 8;
        }
        // 4) Wait for RXNE before reading DR
        while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_RXNE ) == RESET )
        {  /* spin */; }
        DataWord = kUserCommunicationSPI->DR; // Read the DR
        // 6) Wait for the transfer to finish
        while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_BSY ) == SET )
        {  /* spin */; }
        SPI3_SLAVE_SELECT_PORT->BSRRL = SPI3_SLAVE_SELECT_PIN; // CS high - deselect
        RxBuffer[ wordCount ] = DataWord; // return buffer
        wordCount++;

        for( i = 0; i <= delayCounts; i++ ) { DataWord = 0x0000; /* spin */}
    } // End of register read (for-loop)
    OSEnableHook(); // Enable interrupts
}


/** ***************************************************************************
 * @name  BurstReadFromRegisters()
 * @brief  read in a 16-bit word over the SPI NON-DMA ADI style burst
 * with cs set low then left low until the read is complete.
 *
 * @param [in] startingAddress - SPI register address start
 * @param [in] endingAddress - SPI register end
 * @param [in] RxBuffer - buffer to hold incomming data
 * @param [in] delay counts - for spin delay loop
 * @retval N/A
 ******************************************************************************/
void BurstReadFromRegisters( uint8_t startingAddress,
                             uint8_t endingAddress,
                             uint16_t *RxBuffer,
                             uint16_t delayCounts )
{
    uint16_t DataWord;      // hold the shift-register data (SPI->DR)
    uint8_t  RegAddr   = 0; // Register address
    uint8_t  wordCount = 0;
    int      i;

    OSDisableHook(); // Disable interrupts
    /** Send out the read command for the selected data registers - starting
         addresses must be even and two bytes are clocked out at a time (MSB and
         LSB), which is the reason for incrementing the register address by 0x02
         each time.  The loop continues until the register address is two-bytes
         beyond the ending address.  This is due to the extra two-bytes needed
         to clock out the final register information. */
    // clear dr
    DataWord = kUserCommunicationSPI->DR; 
   
    for( RegAddr = startingAddress; RegAddr <= ( endingAddress + 0x2 ); RegAddr = RegAddr + 0x2 ) {
       SPI3_SLAVE_SELECT_PORT->BSRRH = SPI3_SLAVE_SELECT_PIN; // CS low  
      // 2) wait for TXE
        while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_TXE ) == RESET )
        {  /* spin */; }

        // 3) Write to DR (the register address - 16 bits) or 2 zero-bytes to complete the read.
//        if( RegAddr == ( endingAddress + 0x2 ) ) {
//            kUserCommunicationSPI->DR = 0x0000;
//        } else {
//            // Shift the address to form the 16-bit word (padded by 8 zeros)
//            kUserCommunicationSPI->DR = RegAddr << 8;
//        }
        if( RegAddr != startingAddress ) {
            kUserCommunicationSPI->DR = 0x0000;
        } else {
            // Shift the address to form the 16-bit word (padded by 8 zeros)
            kUserCommunicationSPI->DR = RegAddr << 8;
        }

        // 4) Wait for RXNE before reading DR
        while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_RXNE ) == RESET )
        {  /* spin */; }
        DataWord = kUserCommunicationSPI->DR; // Read the DR
        // 6) Wait for the transfer to finish
        while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_BSY ) == SET )
        {  /* spin */; }
        RxBuffer[ wordCount ] = DataWord; // return buffer
        wordCount++;
        for( i = 0; i <= delayCounts; i++ ) { DataWord = 0x0000; /* spin */}
        SPI3_SLAVE_SELECT_PORT->BSRRL = SPI3_SLAVE_SELECT_PIN; // CS high - deselect
    } // End of register read (for-loop)
    
    OSEnableHook(); // Enable interrupts
}

/** ***************************************************************************
 * @name WriteToRegister()
 * @brief  write a 16-bit word over the SPI to a single data register.  Provide
 *         the address (8-bits) and the data word 8-bits). NON-DMA
 *
 * @param [in] registerAddress - SPI register to write to
 * @param [in] dataByte - for spin delay loop
 * @retval N/A
 ******************************************************************************/
uint16_t WriteToRegister( uint8_t registerAddress,
                           uint8_t dataByte )
{
    uint16_t DataWord; // word to hold the outgoing data
    
    OSDisableHook(); // Disable interrupts
    // clear DR
    DataWord = kUserCommunicationSPI->DR; 
    //   DataWord to send to the slave:
    //   MSB: write-bit + address
    //   LSB: data-byte
    DataWord = (( registerAddress | 0x80 ) << 8 )  | dataByte;

    // CS low - enable
    SPI3_SLAVE_SELECT_PORT->BSRRH = SPI3_SLAVE_SELECT_PIN;
    for( uint8_t i = 0; i <= 30; i++ ) {};
    // 2) wait for TXE
    while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_TXE )  == RESET )  {  /* spin */; }
    kUserCommunicationSPI->DR = DataWord; // Write to DR
    // 3) Wait for RXNE before reading DR
    while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_RXNE ) == RESET ) {  /* spin */; }
    // Read the DR
    DataWord = kUserCommunicationSPI->DR;
    // Wait for the transfer to finish
    while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_BSY ) == SET )    {  /* spin */; }
    // Wait before removing CS
    for( uint8_t i = 0; i <= 30; i++ ) { };
    // CS high - disable
    SPI3_SLAVE_SELECT_PORT->BSRRL = SPI3_SLAVE_SELECT_PIN; 
    // Re-Enable interrupts
    OSEnableHook();

    return DataWord; // return the data word (even though it might not be useful)
}

uint16_t SendCommand( uint8_t registerAddress, uint8_t dataByte )
{
    uint16_t DataWord; // word to hold the outgoing data
    uint16_t crc;
    uint8_t  *dataPtr;
    _initCRC(0xffff);
    
    OSDisableHook(); // Disable interrupts
    // clear DR
    DataWord = kUserCommunicationSPI->DR; 
    // DataWord to calculate CRC
    DataWord = (registerAddress | 0x80 ) | (dataByte << 8);
    // calculate CRC here - over DataWord
    dataPtr = (uint8_t *)&DataWord;
    crc     = _calcCRC(dataPtr, 2);
    //   DataWord to send to the slave:
    //   MSB: write-bit + address
    //   LSB: data-byte
    DataWord = (( registerAddress | 0x80 ) << 8 )  | dataByte;

    SPI3_SLAVE_SELECT_PORT->BSRRH = SPI3_SLAVE_SELECT_PIN; // CS low - enable
    for( uint8_t i = 0; i <= 30; i++ ) {};
    // 2) wait for TXE
    while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_TXE ) == RESET )  {  /* spin */; }
    kUserCommunicationSPI->DR = DataWord; // Write to DR
    while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_TXE ) == RESET )  {  /* spin */; }
    // Write CRC 
    kUserCommunicationSPI->DR = crc; 
    // Wait for the transfer to finish
    while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_BSY ) == SET )    {  /* spin */; }
    // 4) Wait for RXNE before reading DR
    while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_RXNE ) == RESET ) {  /* spin */; }
    DataWord = kUserCommunicationSPI->DR;  // Read the DR
    for( uint8_t i = 0; i <= 30; i++ ) {}; // spin
    SPI3_SLAVE_SELECT_PORT->BSRRL = SPI3_SLAVE_SELECT_PIN; // CS high - disable
    OSEnableHook(); // Re-Enable interrupts

    return DataWord; // return the data word (even though it might not be useful)
}


/** ***************************************************************************
 * @name WriteToRegister()
 * @brief  write a 16-bit word over the SPI to a single data register.  Provide
 *         the address (8-bits) and the data word 8-bits). NON-DMA
 *
 * @param [in] registerAddress - SPI register to write to
 * @param [in] dataByte - for spin delay loop
 * @retval N/A
 ******************************************************************************/
uint16_t WriteWord(uint16_t DataWord, uint8_t toggleCS)
{
    OSDisableHook(); // Disable interrupts
    // clear DR
    DataWord = kUserCommunicationSPI->DR; 
    // CS low - enable
    SPI3_SLAVE_SELECT_PORT->BSRRH = SPI3_SLAVE_SELECT_PIN;
    for( uint8_t i = 0; i <= 90; i++ ) { ; }
    // 2) wait for TXE
    while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_TXE ) == RESET ) {  /* spin */; }
    // Write to DR - will be sent MSB first
    kUserCommunicationSPI->DR = DataWord; 
    // Wait for RXNE before reading DR
    while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_RXNE ) == RESET ) {/* spin */; }
    // Read the DR
    DataWord = kUserCommunicationSPI->DR; 
    // Wait for the transfer to finish
    while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_BSY ) == SET ) {  /* spin */; }

    for( uint8_t i = 0; i <= 90; i++ ) {};

    SPI3_SLAVE_SELECT_PORT->BSRRL = SPI3_SLAVE_SELECT_PIN; // CS high - disable

    OSEnableHook(); // Re-Enable interrupts

    return DataWord; // return the data word (even though it might not be useful)
}


/** ***************************************************************************
 * @name  WriteWordsToRegister()
 * @brief  write 16-bit words over the SPI NON-DMA ADI style burst
 * with cs set low then left low until the write is complete.
 *
 * @param [in] regAddress   - SPI register address start
 * @param [in] length       - number of bytes to write
 * @param [in] txBuffer     - buffer with data to transmit
 * @param [in] delay counts - for spin delay loop
 * @retval N/A
 ******************************************************************************/
uint16_t SendBlock(uint16_t regAddress, uint16_t blockNum, uint8_t *txBuffer, uint16_t length)
{
    uint16_t Address, Block, StatusWord, DataWord, crc = 0x1122;
    uint8_t *dataPtr; 
    _initCRC(0xffff);
    
    OSDisableHook(); // Disable interrupts
    // If RXNE - read DR
    if( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_RXNE ) == RESET ) {
        // Read the DR
        DataWord = kUserCommunicationSPI->DR;
    }

    Address = (regAddress | 0x80);                 
    Block   = (blockNum << 8) & 0xff00;
    Block  |= (blockNum >> 8) & 0xff;
    // calculate CRC here - over address + blocNum + txBuffer[length]
    dataPtr = (uint8_t *)&Address;
    _calcCRC(dataPtr, 2);
    dataPtr = (uint8_t *)&Block;
    _calcCRC(dataPtr, 2);
    crc = _calcCRC(txBuffer, length); 
    Address = Address  << 8;                                          // big endian
    Block   = blockNum;
    // Remove RXNE condition
    StatusWord = kUserCommunicationSPI->DR;
    // Set CS low
    SPI3_SLAVE_SELECT_PORT->BSRRH = SPI3_SLAVE_SELECT_PIN; // CS low
    // Wait for TXE
    while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_TXE ) == RESET )  {  /* spin */; }
    // Write register address, shifted by 8 and padded by 0
    kUserCommunicationSPI->DR = Address;
    // Wait for RXNE
    while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_RXNE ) == RESET )  {  /* spin */; }
    // Read status
    StatusWord = kUserCommunicationSPI->DR;
    // Write Fragment Number
    kUserCommunicationSPI->DR = Block;
    // Wait for RXNE
    while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_RXNE ) == RESET )  {  /* spin */; }
    // Read status
    StatusWord = kUserCommunicationSPI->DR;
    for( int i = 0; i < length; i+=2) {
        // wait for TXE
        while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_TXE ) == RESET )  {  /* spin */; }
        // Big endian since byte-oriented
        DataWord = txBuffer[i] << 8 | txBuffer[i+1];
        // Write word to DR
        kUserCommunicationSPI->DR = DataWord; 
    } // End of register rwrite (for-loop)
    // Wait for TXE
    while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_TXE ) == RESET )  {  /* spin */; }
    // Write CRC 
    kUserCommunicationSPI->DR = crc; 
    // Wait for the transfer to finish
    while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_BSY ) == SET )
    // Wwait before removing CS
    for( uint8_t i = 0; i <= 30; i++ ) {};
    // CS high - deselect
    SPI3_SLAVE_SELECT_PORT->BSRRL = SPI3_SLAVE_SELECT_PIN; 
    // Enable interrupts
    OSEnableHook(); 
    
    return StatusWord;
}


/** ***************************************************************************
 * @name ReadFromRegister()
 * @brief  read in a 16-bit word over the SPI NON-DMA
 *
 * @param [in] startingAddress - SPI register address start
 * @param [in] endingAddress - SPI register end
 * @param [in] RxBuffer - buffer to hold incomming data
 * @param [in] delay counts - for spin delay loop
 * @retval N/A
 ******************************************************************************/
uint16_t ReadFromRegister(uint16_t regAddress)
{
    uint16_t DataWord = 0;      // hold the shift-register data (SPI->DR)
    int      i;

    // Disable interrupts
    OSDisableHook();
    // clear DR
    DataWord = kUserCommunicationSPI->DR; 
    // Set CS low
    SPI3_SLAVE_SELECT_PORT->BSRRH = SPI3_SLAVE_SELECT_PIN;
    // Wait for TXE
    while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_TXE ) == RESET ) {  /* spin */; }
    // Shift the address to form the 16-bit word (padded by 8 zeros)
    kUserCommunicationSPI->DR = regAddress << 8;
    // Wait for RXNE before reading DR
    while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_RXNE ) == RESET ){  /* spin */; }
    // Read the DR
    DataWord = kUserCommunicationSPI->DR; 
    // delay between readings
    for( i = 0; i <= 30; i++ ){};
    // Wait for TXE
    while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_TXE ) == RESET ) {  /* spin */; }
    // Send dummy data
    kUserCommunicationSPI->DR = 0;
    // Wait for RXNE before reading DR
    while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_RXNE ) == RESET ){  /* spin */; }
    // Read the DR
    DataWord = kUserCommunicationSPI->DR; 
    // Wait for the transfer to finish
    while( SPI_I2S_GetFlagStatus( kUserCommunicationSPI, SPI_I2S_FLAG_BSY ) == SET ) {  /* spin */; }
    // delay before removing CS
    for( i = 0; i <= 30; i++ ){};
    // CS high - deselect
    SPI3_SLAVE_SELECT_PORT->BSRRL = SPI3_SLAVE_SELECT_PIN; 

    OSEnableHook(); // Enable interrupts
    
    return DataWord;
}
