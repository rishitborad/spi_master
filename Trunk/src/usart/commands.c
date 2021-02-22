/** ***************************************************************************
 * @file   commands.c
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * Commands available to the commandLine.c shell input from the debug serial
 * console interface
 ******************************************************************************/
#include <stdlib.h>
#include <math.h> // floating point sqrt

#include "commandline.h"
#include "commands.h"
#include "salvodefs.h"

#define LOGGING_LEVEL LEVEL_INFO
#include "debug.h"
#include "dmu.h"
#include "timer.h"
#include "xbowsp_version.h"

#include "bsp.h" // LED4

#include "spi.h" // for spi3
#include "boardDefinition.h"
#include "UserCommunication_SPI.h"


int32_t nabs( int32_t signedInteger );

// Timer control and informational variables (used with sync output)
extern uint8_t ToggleSyncPin;
extern volatile uint8_t pwmEnabled;
extern uint8_t pwmDivider;
extern uint16_t freqRatio;
extern uint32_t ElapsedTime_sec, ElapsedTime_usec;

//volatile uint8_t Tx_Idx; // stm32f2xx_it.c
//volatile uint8_t CmdStatus; // stm32f2xx_it.c

//uint8_t TxBuffer[20]; // stm32f2xx_it.c

// Set default values for ...
static uint32_t TotalTests = 10;
static uint32_t DelayTime  = 100; //500;
static uint32_t ODR_Hz     = 2;

static uint8_t startingAddress = 0x3E;
static uint8_t numberOfBytes   = 16;
static uint8_t endingAddress;
static uint8_t TotalWords;

static uint8_t WaitForDRDY = 0;

static uint8_t displayHex = 0;

static uint16_t RxWord[ 128 ];    ///< input sensor data buffer

static char msgType[8];

static uint8_t DebugFlag = 1; // display the console data


#define ONE_MHZ 2
#define TWO_MHZ 3

void _SetSPIClockSpeed( uint8_t clockSpeed );
uint32_t _SetOutputDataRate( uint32_t ODR_Hz );

void _ToggleInputPin( uint8_t pinType, uint16_t numberOfToggles, uint16_t lowPeriod, uint16_t highPeriod );
void _SendJDConfigCmds( void );
void _SendIndraConfigCmds( void );
uint8_t _WhoAmI( uint8_t debugFlag );

void _PrintHeader( uint8_t TotalWords, uint8_t startingAddress, uint32_t TotalTests, uint32_t ODR_Hz );


void _BurstRead(void);

void _DisplayMesage_Stnd( uint16_t *RxWord );
void _DisplayMesage_Stnd2( uint16_t *RxWord );
void _DisplayMesage_S0( uint16_t *RxWord );
void _DisplayMesage_S1( uint16_t *RxWord );
void _DisplayMesage_A1( uint16_t *RxWord );
void _DisplayMesage_A2( uint16_t *RxWord );
void _DisplayMesage_N1( uint16_t *RxWord );


#define NUM_CYCLES_IN_MS 9000
#define NRST 0
#define NSS  1

// Packet ending-address computation: dec2hex( hex2dec( startAddr ) + NumOfBytes - 1 )
// F1 (54 bytes = 27 words: reg 0x3F --> 0x74)
// F2 (66 bytes = 33 words: reg 0x40 --> 0x81)
// S0 (30 bytes = 15 words: reg 0x41 --> 0x5E)
// S1 (24 bytes = 12 words: reg 0x42 --> 0x59)
// A1 (32 bytes = 16 words: reg 0x43 --> 0x62)


// Display firmware version
void CmdVersion(uint32_t data)
{
    DEBUG_INT(" ", VERSION_MAJOR);
    DEBUG_INT(".", VERSION_MINOR);
    DEBUG_INT(".", VERSION_PATCH);
    DEBUG_INT(".", VERSION_STAGE);
    DEBUG_INT(".", VERSION_BUILD);
    DEBUG_ENDLINE();
} // End of display firmware version

// local functions
void _InitDataReady( FunctionalState NewState );

#include "port_def.h"
#include "comm_buffers.h"
#include "uart.h"


// Initialize 
volatile static uint8_t DataReadyContinueFlag = 0; // DR ISR sets this flag
volatile static uint8_t ToogleCSFlag          = HOLD_CS; // DR ISR sets this flag

static volatile int ddd = 0;

static void waitMs()
{
    for(int i = 0; i < NUM_CYCLES_IN_MS; i++) ddd++;
}

static void ppsOn()
{
    ONE_PPS_PORT->BSRRH = ONE_PPS_PIN;
}

static void ppsOff()
{
     ONE_PPS_PORT->BSRRL = ONE_PPS_PIN;
}



/** ***************************************************************************
 * @name    _InitDataReady() LOCAL set up the DR B3 pin
 * @brief  set up the GPIO pins used by BurstRead
 * CmdUserSpi_ReadRegisterHex() - "readHex"
 * CmdUserSpi_ReadRegister() - "rHex"
 * CmdUserSpi_BurstTest01() - "burstTest01"
 * CmdUserSpi_BurstReadHex() - "burstHex"
 * CmdUserSpi_BurstRead() - "burst"
 *
 * @param [in] NewState - ENABLE
 * @retval N/A
 ******************************************************************************/
void _InitDataReady( FunctionalState NewState )
{
    /// Define the initialization structures
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /// Enable the peripheral clocks
    RCC_AHB1PeriphClockCmd( DATA_READY_CLK, ENABLE );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_SYSCFG, ENABLE ); // for interrupts

    /// Configure the data-ready pin (C13) as input
    GPIO_StructInit( &GPIO_InitStructure );
    GPIO_InitStructure.GPIO_Pin  = DATA_READY_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init( DATA_READY_PORT, &GPIO_InitStructure );

    /// Configure EXTI line
    EXTI_StructInit( &EXTI_InitStructure );
    EXTI_InitStructure.EXTI_Line    = DATA_READY_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;   //EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = NewState;
    EXTI_Init( &EXTI_InitStructure );

    /// Connect EXTI Line to GPIO Pin
    SYSCFG_EXTILineConfig( DATA_READY_EXTI_PORT_SOURCE, DATA_READY_EXTI_PIN_SOURCE );

    /// Enable and set EXTI Interrupt to the lowest priority
    NVIC_InitStructure.NVIC_IRQChannel                   = DATA_READY_EXTI_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = NewState;
    NVIC_Init( &NVIC_InitStructure );
    DataReadyContinueFlag = 0;
}

/** ***************************************************************************
 * @name EXTI3_IRQHandler() DATA_READY_EXTI_IRQHandler
 * @brief  ISR for data ready startup_stm32f2xx.s
 *
 * @param [in] NewState - ENABLE
 * @retval N/A
 ******************************************************************************/
void EXTI3_IRQHandler( void )
{
    OSDisableHook();

    /// SPI DR data ready
    /// Set the flag that tells the software to continue
    DataReadyContinueFlag = 1;

    // clear the interrupt bit
    EXTI_ClearITPendingBit( DATA_READY_EXTI_LINE );

    OSEnableHook();
}

// For use when data-ready pin is C13 (IAR board)
void EXTI15_10_IRQHandler( void )
{
    OSDisableHook();

    // Set the flag that tells the software to continue
    DataReadyContinueFlag = 1;

    // clear the interrupt bit
    EXTI_ClearITPendingBit( DATA_READY_EXTI_LINE );

    OSEnableHook();
}

/** ***************************************************************************
 * @name    CmdUserUsart() send serial message out selected bus
 * @brief test a usart
 * ">>output which string"
 *
 * @param [in] uart - which uart?
 * @param [in] buffer - pointer to string to send out
 * @retval N/A
 ******************************************************************************/
void CmdUserUsart(uint32_t data)
{
    uint32_t uart = 0;
    unsigned char *buffer;
    int ok = 0;

    CmdLineGetArgUInt(&uart);
    CmdLineGetArgString(&buffer);

    DEBUG_STRING("Wrote (");
    DEBUG_STRING((const char*)buffer);
    DEBUG_INT(") to ", uart);
    DEBUG_INT(", ok= ", ok);
    DEBUG_ENDLINE();
}


/** ***************************************************************************
 * @name    CmdGpioPin() set gpio pins
 * @brief  "pin" command
 *
 * @param [in] data - N/A
 * @param [in] port - GPIO port
 * @param [in] pin - GPIO pin
 * @param [in] state - 0 or 1
 * @retval N/A
 ******************************************************************************/
void CmdGpioPin(uint32_t data)
{
    uint32_t pin   = 1;
    uint32_t state = 1;
    uint8_t  *port;
    GPIO_TypeDef* GPIO;
    GPIO_InitTypeDef GPIO_InitStructure;

    CmdLineGetArgString(&port);
    CmdLineGetArgUInt(&pin);
    CmdLineGetArgUInt(&state);

    switch(port[0]) {
    case 'A':
      GPIO = GPIOA;
      break;
    case 'B':
      GPIO = GPIOB;
      break;
    case 'C':
      GPIO = GPIOC;
      break;
    case 'D':
      GPIO = GPIOD;
      break;
    case 'F':
      GPIO = GPIOF;
      break;
    default: ERROR_STRING("Unknown port\r\n"); return;
    }

    pin = 1 << pin;

    GPIO_InitStructure.GPIO_Pin   = pin;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIO, &GPIO_InitStructure);

    if (state) {
        GPIO_SetBits(GPIO, pin);   // 1
    } else {
        GPIO_ResetBits(GPIO, pin); // 0
    }
}


/** ***************************************************************************
 * @name CmdUserSpi_ATP_Process()  A series of commands that automate the ATP
 *       check-out procedure
 * @brief "atp" command
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void CmdUserSpi_ATP_Process( uint32_t data )
{
    uint16_t RxBuffer[ 65 ];
    uint8_t  error = 0;

    DEBUG_STRING( "Beginning ATP process...." );
    DEBUG_ENDLINE();
    DEBUG_ENDLINE();

    DEBUG_STRING( "   Product identification:" );
    DEBUG_ENDLINE();

    ReadFromRegisters( 0x52, 0x58, RxBuffer, 500, ToogleCSFlag ); // 25 usec

    DEBUG_HEX( "      Product ID: 0x", RxBuffer[3] );
    if( RxBuffer[3] != 0x3810 ) {
        error = error + 1;
        DEBUG_STRING( "      --> Invalid product ID <---" );
    }
    DEBUG_ENDLINE();    DEBUG_ENDLINE();

    /// Lot ID (1&2) and serial number
    DEBUG_HEX( "      Lot ID #1: 0x", RxBuffer[1] );
    DEBUG_ENDLINE();
    DEBUG_HEX( "      Lot ID #2: 0x", RxBuffer[2] );
    DEBUG_ENDLINE();    DEBUG_ENDLINE();
    DEBUG_HEX( "      Serial Number: 0x", RxBuffer[4] );
    DEBUG_ENDLINE();    DEBUG_ENDLINE();

    /// Restore factory calibration - has no effect as the calibration cannot be
    ///   modified by the user
    DEBUG_STRING( "   Restoring factory calibration" );
    DEBUG_STRING( "      (command has no effect)" );
    DEBUG_ENDLINE();    DEBUG_ENDLINE();
    WriteToRegister( 0x3E, 0x02 );

    /// Configure GPIO #1 as an output and the others as an input
    WriteToRegister( 0x32, 0x01 );
    ReadFromRegisters( 0x32, 0x32, RxBuffer, 500, ToogleCSFlag ); /// 25usec

    DEBUG_HEX( "   Configuring GPIO_CTRL: 0x", RxBuffer[1] );
    if( RxBuffer[1] == 0x0001 ) {
        DEBUG_STRING( "    (Test passed: successful write)" );
    } else {
        DEBUG_STRING( "     --> Test failed: unsuccessful write <---" );
        error++;
    }
    DEBUG_ENDLINE();
    DEBUG_ENDLINE();

    /// Set SMPL_PRD register
    WriteToRegister( 0x36, 0x00 );   /// Set external sampling clock
    WriteToRegister( 0x37, 0x01 );   /// Set decimation
    ReadFromRegisters( 0x36, 0x36, RxBuffer, 500, ToogleCSFlag ); /// 25usec

    DEBUG_HEX( "   Configuring SMPL_PRD: 0x", RxBuffer[1] );
    if( RxBuffer[1] == 0x0100 ) {
        DEBUG_STRING( "    (Test passed: successful write)" );
    } else {
        DEBUG_STRING( "     --> Test failed: unsuccessful write <---" );
        error++;
    }
    DEBUG_ENDLINE();    DEBUG_ENDLINE();

    /// Set the SENS_AVG register
    WriteToRegister( 0x38, 0x05 );   ///< Set external sampling clock
    WriteToRegister( 0x39, 0x02 );   ///< Set decimation
    ReadFromRegisters( 0x38, 0x38, RxBuffer, 500, ToogleCSFlag ); ///< 25usec

    DEBUG_HEX( "   Configuring SENS_AVG: 0x", RxBuffer[1] );
    if( RxBuffer[1] == 0x0205 ) {
        DEBUG_STRING( "    (Test passed: successful write)" );
    } else {
        DEBUG_STRING( "     --> Test failed: unsuccessful write <---" );
        error++;
    }
    DEBUG_ENDLINE();    DEBUG_ENDLINE();

    /// Set the MSC_CTRL register
    WriteToRegister( 0x34, 0x04 );   ///< Set decimation
    ReadFromRegisters( 0x34, 0x34, RxBuffer, 500, ToogleCSFlag );

    DEBUG_HEX( "   Configuring MSC_CTRL (data ready): 0x", RxBuffer[1] );
    if( RxBuffer[1] == 0x0004 ) {
        DEBUG_STRING( "    (Test passed: successful write)" );
    } else {
        DEBUG_STRING( "     --> Test failed: unsuccessful write <---" );
        error++;
    }
    DEBUG_ENDLINE();
    DEBUG_ENDLINE();

    /// Start the self-test
    WriteToRegister( 0x35, 0x04 );   ///< Set decimation
    DelayMs( 1000 );
    ReadFromRegisters( 0x3C, 0x3C, RxBuffer, 500, ToogleCSFlag ); ///< 25usec

    DEBUG_HEX( "   Beginning self test (data ready): 0x", RxBuffer[1] );
    if( ( RxBuffer[1] & 0020 ) == 0x0000 ) {
        DEBUG_STRING( "    (Test passed: successful self-test)" );
    } else {
        DEBUG_STRING( "     --> Self-test failed <---" );
        error++;
    }
    DEBUG_ENDLINE();
    DEBUG_ENDLINE();

    if( error == 0 ) {
        DEBUG_STRING( "ATP passed without error" );
    } else {
        DEBUG_STRING( "ATP failed (check above output for details)" );
    }
    DEBUG_ENDLINE();
    DEBUG_ENDLINE();
}


/** ***************************************************************************
 * @name CmdUserSpi_jdInitProcess() A series of commands that test the John
 *       Deere initialize
 * @brief "j" command
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void CmdUserSpi_jdInitProcess( uint32_t data )
{
    float    tempVal;       // convert from uint to float after the read
    uint8_t  signVal;

    //uint16_t RxWord[ 128 ] = { 0 };

    // Specify times in milliseconds
    uint16_t pinLowTime[2]  = {  500, 500 };
    uint16_t pinHighTime[2] = { 2000, 500 };

    /// (based on burst-read requirements)
    startingAddress = 0x3E; // burst mode
    numberOfBytes   = 16;
    endingAddress   = startingAddress + ( numberOfBytes - 1 );
    TotalWords = ( ( endingAddress + 1 ) - startingAddress )/2;

    //uint16_t testNumber;
    uint8_t toggleFlag = 1;

    uint32_t numOfTimeouts = 0;

    // Specify the default number of toggles of the nRst and nSS lines (can be
    //   overwritten via input arguments)
    uint32_t numberOfToggles[2] = { 5, 5 };

    // Specify the default number of tests (consisting of alternating nRst and
    //   nSS toggles).  Can be overwritten via input arguments.
    uint32_t numberOfTests = 3;

    // get the variables from the console arguments
    CmdLineGetArgUInt( &numberOfTests );
    CmdLineGetArgUInt( &numberOfToggles[0] );
    CmdLineGetArgUInt( &numberOfToggles[1] );

    // Limit the number of tests and toggles
    if( numberOfTests > 65500 ) {
        numberOfTests = 65500;
    }

    if( numberOfToggles[0] > 65500 ) {
        numberOfToggles[0] = 65500;
    }

    if( numberOfToggles[1] > 65500 ) {
        numberOfToggles[1] = 65500;
    }

    // Flag used to wait in place for the data-ready line before reading sensor
    //  data
    DataReadyContinueFlag = 0;

    // Display flag -- display every tenth reading
    uint8_t TrialCounter = 10;

    // Set SPI clock speed
    _SetSPIClockSpeed( ONE_MHZ );

    // Get the product ID
    _WhoAmI( 1 );
    DelayMs( 200 );

    // whoami
    uint16_t RxBuffer[ 65 ] = { 0 };
    ReadFromRegisters_noChipSelect( 0x56, 0x56, RxBuffer, 150 ); /// product ID (0x3810) 150 ~10usec
    DelayMs( 500 );

    // Set up the external data-ready (DR) interrupt.  'DataReadyContinueFlag'
    //   toggles when new data is available from the 380.
    _InitDataReady( ENABLE );
    DelayMs( 200 );

    // ----- Reset the power to the 380 and drop the reset line -----
    // turn off power to the board, set nSS high, set nRst low
    DEBUG_STRING( "Remove power from the 380 and set nRst low" );
    DEBUG_ENDLINE();
    GPIOE->BSRRH = GPIO_Pin_0;  // Set E0 Low (remove power)
    SPI3_SLAVE_SELECT_PORT->BSRRL = SPI3_SLAVE_SELECT_PIN;
    GPIOE->BSRRH = GPIO_Pin_2;

    // Wait ...
    DelayMs( 2000 );

    // Power on board
    DEBUG_STRING( "Restore power to the 380 and release nRst 1 second after power applied" );
    DEBUG_ENDLINE();
    GPIOE->BSRRL = GPIO_Pin_0;  // Set E0 high

    // Set reset (E2) high after 100 msec
    DelayMs( 1000 );
    GPIOE->BSRRL = GPIO_Pin_2;

    // Wait 100 msec before talking to unit
    DEBUG_ENDLINE();
    DEBUG_STRING( "Begin configuration of 380..." );
    DEBUG_ENDLINE();
    DelayMs( 500 );

    // Send the JD configuration commands
    _SendJDConfigCmds();

    uint32_t loopCntr = 0;
    uint32_t loopCntrLimit = 10000;

    // **************** READ DATA ****************
    DEBUG_STRING( "Configuration complete." );
    DEBUG_ENDLINE();
    DelayMs( 500 );

    DEBUG_ENDLINE();
    DEBUG_STRING( "Begin post-configuration data read:" );
    DEBUG_ENDLINE();
    DelayMs( 500 );

    DataReadyContinueFlag = 0;
    /// Burst-read: perform the read a specified number of times
    for( int TrialNumber = 0; TrialNumber < 100; TrialNumber++ )
    {
        while( !DataReadyContinueFlag ) {
            /* Wait here until the data-ready line is set by the 380 */
        }
        DataReadyContinueFlag = 0;

        ReadFromRegisters( startingAddress, endingAddress, RxWord, 105, ToogleCSFlag );

        TrialCounter = TrialCounter + 1;

        if( TrialCounter >= 10 ) {
            TrialCounter = 0;
            /// Display the UUT data from the SPI slave
            for( int LoopCounter = 1; LoopCounter < TotalWords+1; LoopCounter++ ) {
                if( LoopCounter == 1 ) {
                    /// Status word
                    DEBUG_HEX( "", RxWord[ LoopCounter ] );
                } else if( LoopCounter <= 4 ) {
                    /// RATE sensor conversion
                    tempVal = 0.005 * (float)( (int16_t)RxWord[ LoopCounter ] );
                    if( tempVal >= 0 ) {
                        DEBUG_STRING( " " ); // ' ' instead of '-' for positive data
                    }

                    /// Spaces to adjust the columns based on value (ones/tens/hundreds)
                    signVal = abs( (int8_t)tempVal );
                    if( signVal < 10 ) {
                        DEBUG_STRING( "  " );
                    } else if( signVal < 100 ) {
                        DEBUG_STRING( " " );
                    }

                    /// Write the RATE data to the console
                    DEBUG_FLOAT( "  ", tempVal, 4 );
                } else if ( LoopCounter <= 7 ) {
                    /// ACCELeration conversion
                    tempVal = 0.00025 * (float)( (int16_t)RxWord[ LoopCounter ] );
                    if( tempVal >= 0 ) {
                        DEBUG_STRING( " " );
                    }
                    DEBUG_FLOAT( "   ", tempVal, 4 );
                } else {
                    /// board TEMPERATURE - conversion
                    DEBUG_FLOAT( "   ", 0.073111172849435 * (float)( (int16_t)RxWord[ LoopCounter ] ) + 31.0, 4 );
                }
            }
            DEBUG_ENDLINE(); // Print a return character
        }
        DelayMs( 1 );
    }

    // Perform the nSS/nRst toggle tests
    for( uint16_t testNumber = 0; testNumber < 2*numberOfTests; testNumber++ ) {
        toggleFlag = toggleFlag + 1;
        if( toggleFlag > 1 ) {
            toggleFlag = 0;
        }

        DEBUG_ENDLINE();
        if( toggleFlag == 0 ) {
            DEBUG_INT( "Toggling nRst ", numberOfToggles[0] );
        } else {
            DEBUG_INT( "Toggling nSS ", numberOfToggles[1] );
        }
        DEBUG_INT( " times followed by data read (test ", testNumber+1 );
        DEBUG_INT( "/", 2*numberOfTests );
        DEBUG_STRING( "):  " );
        DelayMs( 500 );

        // Toggle the nSS or nRst pin based on input arguments
        _ToggleInputPin( toggleFlag,
                         numberOfToggles[toggleFlag],
                         pinLowTime[toggleFlag],
                         pinHighTime[toggleFlag] );

        DataReadyContinueFlag = 0;
        /// Burst-read: perform the read a specified number of times
        for( int TrialNumber = 0; TrialNumber < 100; TrialNumber++ )
        {
            while( !DataReadyContinueFlag ) {
                /* Wait here until the data-ready line is set by the 380.  If
                   the master waits for longer than 10000 cycles then display
                   a message and syst */
                loopCntr++;
                DelayMs(1);
                if( loopCntr > loopCntrLimit ) {
                    if( toggleFlag == 0 ) {
                        DEBUG_STRING( "   Time-out during nSS toggle ..." );
                        DEBUG_ENDLINE();
                        DEBUG_STRING( "   system being reset (toggling nRst)" );
DelayMs( 50 );
_WhoAmI( 1 );
DelayMs( 50 );

                        //
                        _ToggleInputPin( NRST, 1, pinLowTime[NRST], pinHighTime[NRST] );
                        DelayMs( 2000 );
                    } else {
                        DEBUG_STRING( "   Time-out during nRst toggle ..." );
DelayMs( 50 );
_WhoAmI( 1 );
DelayMs( 50 );
                    }
                    DEBUG_ENDLINE();
                    DEBUG_ENDLINE();

                    numOfTimeouts++;

                    // Exit from the loop and continue with the next test
                    break;
                }
            }
            DataReadyContinueFlag = 0;
            loopCntr = 0;

            ReadFromRegisters( startingAddress, endingAddress, RxWord, 105, ToogleCSFlag );

            TrialCounter = TrialCounter + 1;

            if( TrialCounter >= 10 ) {
                TrialCounter = 0;
                /// Display the UUT data from the SPI slave
                for( int LoopCounter = 1; LoopCounter < TotalWords+1; LoopCounter++ ) {
                    if( LoopCounter == 1 ) {
                        /// Status word
                        DEBUG_HEX( "", RxWord[ LoopCounter ] );
                    } else if( LoopCounter <= 4 ) {
                        /// RATE sensor conversion
                        tempVal = 0.005 * (float)( (int16_t)RxWord[ LoopCounter ] );
                        if( tempVal >= 0 ) {
                            DEBUG_STRING( " " ); // ' ' instead of '-' for positive data
                        }

                        /// Spaces to adjust the columns based on value (ones/tens/hundreds)
                        signVal = abs( (int8_t)tempVal );
                        if( signVal < 10 ) {
                            DEBUG_STRING( "  " );
                        } else if( signVal < 100 ) {
                            DEBUG_STRING( " " );
                        }

                        /// Write the RATE data to the console
                        DEBUG_FLOAT( "  ", tempVal, 4 );
                    } else if ( LoopCounter <= 7 ) {
                        /// ACCELeration conversion
                        tempVal = 0.00025 * (float)( (int16_t)RxWord[ LoopCounter ] );
                        if( tempVal >= 0 ) {
                            DEBUG_STRING( " " );
                        }
                        DEBUG_FLOAT( "   ", tempVal, 4 );
                    } else {
                        /// board TEMPERATURE - conversion
                        DEBUG_FLOAT( "   ", 0.073111172849435 * (float)( (int16_t)RxWord[ LoopCounter ] ) + 31.0, 4 );
                    }
                }
                DEBUG_ENDLINE(); // Print a return character
            }
            DelayMs( 1 );
        }
    }

    DEBUG_ENDLINE();
    DEBUG_STRING( "Initialization test complete" );
    DEBUG_INT( " (number of timeouts: ", numOfTimeouts );
    DEBUG_STRING( ")" );
    DEBUG_ENDLINE();
    DEBUG_ENDLINE();
    DelayMs( 500 );

    // Reset the SPI clock to 1 MHz and disable the data-ready interrupt
    _SetSPIClockSpeed( ONE_MHZ );
    _InitDataReady( DISABLE );
}


/** ***************************************************************************
 * @name CmdUserSpi_SelfTest()
 * @brief Initiate self test and tabulate the results
 * "selftest #" command
 * Performs sanity checks: unit connected? Z down near 1g and X rate near zero
 * commands the self test
 *
 * @param [in] numberOfRuns - number of times to repeat the self test
 * @param [in] readDelay - variable delay between checks for self-test completion (in msec)
 * @retval console:
 *
 * Example Usage: type in the console:
 * @code
 *    >> selftest 3 10 // run the self test 3 times; wait ten millisecond between check for
                       //   test completion
 * @endcode
 ******************************************************************************/
void CmdUserSpi_SelfTest( uint32_t data )
{
    uint16_t RxBuffer[ 128 ] = {0};
    uint8_t  error        = 0;
    uint16_t reg34        = 0x0000;
    uint16_t reg3C        = 0x0000;
    uint16_t reg52        = 0x0000; // return 0x1310
    uint32_t escapeCntr   = 0;
    uint32_t runNumber    = 0;

    uint32_t numberOfRuns = 10;    // number of self tests to execute back-to-back
    uint32_t readDelay    = 5;     // msec
    CmdLineGetArgUInt( &numberOfRuns );
    CmdLineGetArgUInt( &readDelay );

    DEBUG_STRING( "Beginning DMU380 self-test...." );
    DEBUG_ENDLINE();    DEBUG_ENDLINE();

    // Set the SPI clock speed
    _SetSPIClockSpeed( ONE_MHZ );

    // Set ODR of the 380 to 200 Hz
    ODR_Hz = _SetOutputDataRate( 200 );

    /// Begin the self-test iterations
    for( runNumber = 0; runNumber < numberOfRuns; runNumber++ )
    {
        /// read the product ID (like whoami) to verify that the unit is conected
        ReadFromRegisters( 0x56, 0x56, RxBuffer, 150, ToogleCSFlag ); /// product ID (0x3810) 150 ~10usec
 		reg52 = RxBuffer[1];

        // Proceed with self-test if the product ID is correct
		if ( reg52 == 0x3810 ) {
            /// Begin the self-test by writing 0x04 to register 0x35
            WriteToRegister( 0x35, 0x04 );

            // Perform the self-test up to 1500 times before escaping and declaring the self-test
            //   failed.
            escapeCntr = 0;
            do {
                // Wait 1 millisecond before checking for completion (readDelay can also be set to
                //   another value through the command line)
                DelayMs(readDelay);

                // Read the self-test status from register 0x34 (bit 10 is set high when self-test
                //   is executing).
                ReadFromRegisters( 0x34, 0x34, RxBuffer, 425, ToogleCSFlag ); // 425 ~ 27usec
                reg34 = RxBuffer[1];
                escapeCntr++;
            } while( ( reg34 & 0x0400 ) && ( escapeCntr < 1500 ) );

            // After test completes, check the Self-test result register for test results
            ReadFromRegisters( 0x3C, 0x3C, RxBuffer, 425, ToogleCSFlag ); // 425 ~ 27usec
            reg3C = RxBuffer[1];

            // If more than 1500 milliseconds elapsed after the start of the self-test or the
            //   the result stored in register indicates a failed self-test then report a test
            //   failure.
            if( ( escapeCntr >= 1500 ) || ( reg3C & 0x0020 ) ) {
                error++;
                DEBUG_INT( "Run number ", runNumber+1 );
                DEBUG_HEX( ": Register 0x", 0x3C );
                DEBUG_HEX( " =  0x", reg3C );
                DEBUG_INT( " (escape cntr = ", escapeCntr );
                DEBUG_STRING( ")" );
                DEBUG_ENDLINE();
            } else { /// caution this will hit if UUT is not connected
                DEBUG_INT( "Run number ", runNumber+1 );
                DEBUG_INT( ": self-test completed in ", escapeCntr );
                DEBUG_HEX( " cycles (diagnostic register at test completion, 0x", reg3C );
                DEBUG_STRING( ")" );
                DEBUG_ENDLINE();
            }

            // Delay 1/2 second before beginning the next self-test
            DelayMs( 500 );
        } else {
                error++;
                DEBUG_INT( "Run number: ", runNumber );
                DEBUG_STRING( " No Unit connected or connected unit faulty." );
                DEBUG_ENDLINE();
        }
    }

    // Report the results of the self-test
    DEBUG_ENDLINE();
    DEBUG_INT( "Self-test attempted ", numberOfRuns );
    DEBUG_INT( " tests with ", error );
    DEBUG_STRING( " errors." );
    DEBUG_ENDLINE();

    // Restore the default ODR of the 380
    ODR_Hz = _SetOutputDataRate( 2 );
}

/** ***************************************************************************
 * @name    CmdUserSpi_BurstRead()
 * @brief   Initiate a series of reads using the SPI
 *
 * This function reads data from the DMU380 over the SPI and displays the data
 *   on the console. Shows the following: status word, rates, accelerations,
 *   and temp. * commands a burst of data through the SPI bus
 *
 * @param [in] TotalTests - number of times to repeat the burst read
 * @param [in] ODR - output data rate of the 380 in Hz
 * @retval console
 *
 * Example Usage:
 * @code
 *    burst 5 10; // return 5 sets of data at a 10 Hz ODR
 * @endcode
 ******************************************************************************/
uint16_t dataBuffer[100];
void CmdUserSpi_BurstRegs( uint32_t data )
{
    int arg1 = 0, arg2 = 0;
    /// Starting address and number of registers per read based on burst-read requirements
    CmdLineGetArgUInt( &arg1 );
    CmdLineGetArgUInt( &arg2 );

    startingAddress = 0;
    numberOfBytes   = 16;

    if(arg1){
        startingAddress = arg1;
    }

    if(arg2){
        numberOfBytes = arg2;
    }
    
    endingAddress   = startingAddress + ( numberOfBytes - 1 );

    // Each register is 8-bits (1-byte).  A word is two registers (16-bits).
    TotalWords = ( ( endingAddress + 1 ) - startingAddress )/2;

    memset(dataBuffer, 0xaa, sizeof(dataBuffer));
    //
    strcpy(msgType, "stnd");
    BurstReadFromRegisters( startingAddress, endingAddress, dataBuffer, 100); // 25 usec

    /// Display the UUT data from the SPI slave
    for( int LoopCounter = 0; LoopCounter < TotalWords+1; LoopCounter++ )
    {
        DEBUG_HEX( " 0x", dataBuffer[LoopCounter ]);
    }
    DEBUG_ENDLINE();
}


/** ***************************************************************************
 * @name    CmdUserSpi_BurstRead()
 * @brief   Initiate a series of reads using the SPI
 *
 * This function reads data from the DMU380 over the SPI and displays the data
 *   on the console. Shows the following: status word, rates, accelerations,
 *   and temp. * commands a burst of data through the SPI bus
 *
 * @param [in] TotalTests - number of times to repeat the burst read
 * @param [in] ODR - output data rate of the 380 in Hz
 * @retval console
 *
 * Example Usage:
 * @code
 *    burst 5 10; // return 5 sets of data at a 10 Hz ODR
 * @endcode
 ******************************************************************************/
void CmdUserSpi_BurstRead( uint32_t data )
{
    /// Starting address and number of registers per read based on burst-read requirements
    startingAddress = 0x3E;
    numberOfBytes   = 16;
    endingAddress   = startingAddress + ( numberOfBytes - 1 );

    // Each register is 8-bits (1-byte).  A word is two registers (16-bits).
    TotalWords = ( ( endingAddress + 1 ) - startingAddress )/2;

    // Extract values from the console arguments (provide default values in case no arguments
    //   are specified)
    TotalTests = 10;
    ODR_Hz     = 2;
    CmdLineGetArgUInt( &TotalTests );
    CmdLineGetArgUInt( &ODR_Hz );

    //
    strcpy(msgType, "stnd");
    _BurstRead();
}

void CmdUserSpi_BurstRead2( uint32_t data )
{
    /// Starting address and number of registers per read based on burst-read requirements
    startingAddress = 0x3F;
    numberOfBytes   = 20;
    endingAddress   = startingAddress + ( numberOfBytes - 1 );

    // Each register is 8-bits (1-byte).  A word is two registers (16-bits).
    TotalWords = ( ( endingAddress + 1 ) - startingAddress )/2;

    // Extract values from the console arguments (provide default values in case no arguments
    //   are specified)
    TotalTests = 10;
    ODR_Hz     = 2;
    CmdLineGetArgUInt( &TotalTests );
    CmdLineGetArgUInt( &ODR_Hz );

    //
    strcpy(msgType, "stnd2");
    _BurstRead();
}


/** ***************************************************************************
 * @name    CmdUserSpi_BurstRead_S0()
 * @brief   Initiate a series of reads using the SPI
 *
 * This function reads data from the DMU380 over the SPI and displays the data
 *   on the console. Shows the following: status word, rates, accelerations,
 *   and temp. * commands a burst of data through the SPI bus
 *
 * @param [in] TotalTests - number of times to repeat the burst read
 * @param [in] DelayTime - number of times to repeat the burst readdelay in ms
 * @retval console
 *
 * Example Usage:
 * @code
 *    burst_s0 3 1; // return 3 sets of data with a 1ms dealy
 * @endcode
 ******************************************************************************/
void CmdUserSpi_BurstRead_S0( uint32_t data )
{
    /// Starting address and number of registers per read based on burst-read requirements
    startingAddress = 0x41;
    numberOfBytes   = 30;
    endingAddress   = startingAddress + ( numberOfBytes - 1 );

    // Each register is 8-bits (1-byte).  A word is two registers (16-bits).
    TotalWords = ( ( endingAddress + 1 ) - startingAddress )/2;

    // Extract values from the console arguments (provide default values in case no arguments
    //   are specified)
    TotalTests = 10;
    ODR_Hz     = 2;
    CmdLineGetArgUInt( &TotalTests );
    CmdLineGetArgUInt( &ODR_Hz );

    //
    strcpy(msgType, "s0");
    _BurstRead();
}


/** ***************************************************************************
 * @name    CmdUserSpi_BurstRead_S1()
 * @brief   Initiate a series of reads using the SPI
 *
 * This function reads data from the DMU380 over the SPI and displays the data
 *   on the console. Shows the following: status word, rates, accelerations,
 *   and temp. * commands a burst of data through the SPI bus
 *
 * @param [in] TotalTests - number of times to repeat the burst read
 * @param [in] DelayTime - number of times to repeat the burst readdelay in ms
 * @retval console
 *
 * Example Usage:
 * @code
 *    burst_s0 3 1; // return 3 sets of data with a 1ms dealy
 * @endcode
 ******************************************************************************/
void CmdUserSpi_BurstRead_S1( uint32_t data )
{
    /// Starting address and number of registers per read based on burst-read requirements
    startingAddress = 0x42;
    numberOfBytes   = 24;
    endingAddress   = startingAddress + ( numberOfBytes - 1 );

    // Each register is 8-bits (1-byte).  A word is two registers (16-bits).
    TotalWords = ( ( endingAddress + 1 ) - startingAddress )/2;

    // Extract values from the console arguments (provide default values in case no arguments
    //   are specified)
    TotalTests = 10;
    ODR_Hz     = 2;
    CmdLineGetArgUInt( &TotalTests );
    CmdLineGetArgUInt( &ODR_Hz );

    //
    strcpy(msgType, "s1");
    _BurstRead();
}




void _BurstRead(void)
{
    uint16_t waitTime = (uint16_t)( ( 1.0 / (float)ODR_Hz) * 1000.0 + 0.5 );

    // Set ODR of the 380
    ODR_Hz = _SetOutputDataRate( ODR_Hz );

    // Set up external Data-Ready (DR) interrupt.  DataReadyContinueFlag toggles when new data is
    //   available from the 380.
    _InitDataReady( ENABLE );

    /// Print a header in the terminal window
    if( DebugFlag ) {
        _PrintHeader( TotalWords, startingAddress, TotalTests, ODR_Hz );
    }

    /// Read from the 380 a specified number of times (10, by default, or value specified in command line)
    for( uint32_t TrialNumber = 0; TrialNumber < TotalTests; TrialNumber++ )
    {
        /// Wait here until the data-ready (DR) line goes low (set in the data-
        ///   ready interrupt handler)before proceeding
        if( WaitForDRDY ) {
            while( !DataReadyContinueFlag )
            {  /* Wait*/   }
        } else {
            DelayMs( waitTime );  // user i pause between readings
        }

        // Reset wait flag and proceed with read from the DMU380
        DataReadyContinueFlag = 0;
        ReadFromRegisters( startingAddress, endingAddress, RxWord, 105, ToogleCSFlag );

        //
        if( DebugFlag )
        {
            if( strcmp( "stnd", msgType ) == 0 ) {
                _DisplayMesage_Stnd(RxWord);
            } else if( strcmp( "stnd2", msgType ) == 0 ) {
                _DisplayMesage_Stnd2(RxWord);
            } else if( strcmp( "s0", msgType ) == 0 ) {
                _DisplayMesage_S0(RxWord);
            } else if( strcmp( "s1", msgType ) == 0 ) {
                _DisplayMesage_S1(RxWord);
            } else if( strcmp( "a1", msgType ) == 0 ) {
                _DisplayMesage_A1(RxWord);
            } else if( strcmp( "a2", msgType ) == 0 ) {
                _DisplayMesage_A2(RxWord);
            } else if( strcmp( "n1", msgType ) == 0 ) {
                _DisplayMesage_N1(RxWord);
            }
        }

        // Pause here for 1 msec before returning to the top of the loop (to prevent reaching the
        //   top before the interrupt is reset -- although this shouldn't occur)
        //DelayMs( 1 );  // user i pause between readings
    }

    // Disable the data-ready
    _InitDataReady( DISABLE );

    // Restore the default number of data collects and ODR of the 380 between collections
    TotalTests = 10;  // number of samples (10)
//    ODR_Hz = _SetOutputDataRate( 2 );  // 100 Hz ODR
}


/** ***************************************************************************
 * @name    CmdUserSpi_GetBoardTemp()
 * @brief   This function reads temperature data from the DMU380 over the SPI
 * and displays the data on the monitor.
* "temp # #" command
 *
 * @param [in] TotalTests - number of lines to write out
 * @param [in] DelayTime - variable delay in ms
 * @retval console:
 *
 * Example Usage: type in the console:
 * @code
 *    >>temp 3 1// send out 3 sample of temperature data
 * @endcode
 ******************************************************************************/
void CmdUserSpi_GetBoardTemp( uint32_t data )
{
  //uint16_t RxWord[ 128 ];       // the read sensor data
  uint8_t  TotalWords      = 0; // how many words are read in from the slave
  // (based on burst-read requirements)
  uint8_t  startingAddress = 0x18;
  uint8_t  endingAddress   = 0x18;

  // Populate the variable from the program arguments
  CmdLineGetArgUInt( &TotalTests );
  CmdLineGetArgUInt( &DelayTime );

    // Set the SPI clock speed
    _SetSPIClockSpeed( ONE_MHZ );

    /// Print a header in the terminal window
    if( DebugFlag ) {
        _PrintHeader( TotalWords, startingAddress, TotalTests, ODR_Hz );
    }

  for( uint32_t TrialNumber = 0; TrialNumber < TotalTests; TrialNumber++ )
  {
      ReadFromRegisters( startingAddress, endingAddress, RxWord, 150, ToogleCSFlag );

      if( DebugFlag )
      {
          if( ( TrialNumber+1 ) < 10 ) {
              DEBUG_STRING( "   " );
          } else if( ( TrialNumber+1 ) < 100 ) {
              DEBUG_STRING( "  " );
          } else if( ( TrialNumber+1 ) < 1000 ) {
              DEBUG_STRING( " " );
          }
          DEBUG_INT( "",  TrialNumber+1 );
          DEBUG_STRING( ":  " );

          // Display the returned slave data
          for( int LoopCounter = 1; LoopCounter < TotalWords+1; LoopCounter++ ) {
              /// board TEMPERATURE
              DEBUG_FLOAT( "  ", 0.073111172849435 * (float)( (int16_t)RxWord[ LoopCounter ] ) + 31.0, 4 );
          }
      }
      DEBUG_ENDLINE();
      DelayMs( DelayTime ); // pause between readings
  }
}


/** ***************************************************************************
 * @name    CmdUserSpi_GetAccels()
 * @brief This function reads data from the DMU380 over the SPI and displays
 * the data on the monitor.
 * "accels # #" command
 *
 * @param [in] TotalTests - number of lines to write out
 * @param [in] DelayTime - variable delay in ms
 * @retval console:
 *
 * Example Usage: type in the console:
 * @code
 *    >>accels 3 1// send out 3 sample of temperature data
 * @endcode
 ******************************************************************************/
void CmdUserSpi_GetAccels( uint32_t data )
{
  //uint16_t RxWord[ 128 ]; // read sensor data
  uint8_t  TotalWords = 0; // how many words read in from the slave
  // (based on burst-read requirements)
  uint8_t  startingAddress = 0x0A;
  uint8_t  endingAddress   = 0x0E;
  float    tempVal;       // convert from uint to float after the read

    TotalWords = ( ( endingAddress + 2 ) - startingAddress )/2;

  // Populate the variable from the program arguments
  CmdLineGetArgUInt( &TotalTests );
  CmdLineGetArgUInt( &DelayTime );

    // Set the SPI clock speed
    _SetSPIClockSpeed( ONE_MHZ );

    /// Print a header in the terminal window
    if( DebugFlag ) {
        _PrintHeader( TotalWords, startingAddress, TotalTests, ODR_Hz );
    }

  // Perform the read the specified number of times
  for( uint32_t TrialNumber = 0; TrialNumber < TotalTests; TrialNumber++ )
  {
      ReadFromRegisters( startingAddress, endingAddress, RxWord, 150, ToogleCSFlag );

      if( DebugFlag )
      {   // Display the reading number
          if( ( TrialNumber+1 ) < 10 ) {
              DEBUG_STRING( "   " );
          } else if( ( TrialNumber+1 ) < 100 ) {
              DEBUG_STRING( "  " );
          } else if( ( TrialNumber+1 ) < 1000 ) {
              DEBUG_STRING( " " );
          }
          DEBUG_INT( "",  TrialNumber+1 );
          DEBUG_STRING( ":  <" );

          // Display the returned slave data
          for( int LoopCounter = 1; LoopCounter < TotalWords+1; LoopCounter++ ) {
              /// Acceleration values
              tempVal = 0.00025 * (float)( (int16_t)RxWord[ LoopCounter ] );

              // extra space for non-negative numbers
              if( tempVal >= 0 ) {
                  DEBUG_STRING( " " );
              }
              DEBUG_FLOAT( "  ", tempVal, 4 );
          }
          DEBUG_STRING( " >" );
      }
      DEBUG_ENDLINE();
      DelayMs( DelayTime );
  }
}

/** ***************************************************************************
 * @name    CmdUserSpi_GetRates()
 * @brief This function reads sensor data from the DMU380 using polled reads over
 * the SPI and displays the data on the monitor.
 * "rates # #" command
 *
 * @param [in] TotalTests - number of lines to write out
 * @param [in] DelayTime - variable delay in ms
 * @retval console:
 *
 * Example Usage: type in the console:
 * @code
 *    >>rates 3 1// send out 3 sample of temperature data
 * @endcode
 ******************************************************************************/
void CmdUserSpi_GetRates( uint32_t data )
{
    //uint16_t RxWord[ 128 ]; // read sensor data

    // (based on burst-read requirements)
    uint8_t  startingAddress = 0x04;
    uint8_t  numberOfBytes   = 6;
    uint8_t  endingAddress   = startingAddress + ( numberOfBytes - 1 ); //0x08;
    uint8_t TotalWords = ( ( endingAddress + 1 ) - startingAddress )/2;

    float    tempVal;       // convert from uint to float after the read
    uint8_t  signVal;

    // Populate the variable from the console arguments
    TotalTests = 10;
    ODR_Hz     = 2;
    CmdLineGetArgUInt( &TotalTests );
    CmdLineGetArgUInt( &ODR_Hz );

    // Set the ODR of the 380
    ODR_Hz = _SetOutputDataRate( ODR_Hz );

    // Set the SPI clock speed
    _SetSPIClockSpeed( TWO_MHZ );

    // Initialize the data-ready interrupt
    _InitDataReady( ENABLE );

    /// Print a header in the terminal window
    if( DebugFlag ) {
        _PrintHeader( TotalWords, startingAddress, TotalTests, ODR_Hz );
    }

    //
    for( uint32_t TrialNumber = 0; TrialNumber < TotalTests; TrialNumber++ )
    {
        // Wait here until the data-ready line goes low (set by the 380).  Data transfer is
        //   performed only when new data is available from the 380.
        while( !DataReadyContinueFlag ) {
            /* Wait for the data-ready flag before proceeding with read
               (set in the data-ready interrupt routine) */
        }

        // Reset wait flag and proceed with the read from the 380
        DataReadyContinueFlag = 0;
        ReadFromRegisters( startingAddress, endingAddress, RxWord, 150, ToogleCSFlag );

        if( DebugFlag )
        {   // Print the trial number at the start of the line
            if( ( TrialNumber+1 ) < 10 ) {
                DEBUG_STRING( "   " );
            } else if( ( TrialNumber+1 ) < 100 ) {
                DEBUG_STRING( "  " );
            } else if( ( TrialNumber+1 ) < 1000 ) {
                DEBUG_STRING( " " );
            }
            DEBUG_INT( "",  TrialNumber+1 );
            DEBUG_STRING( ":  <" );

            // Display the slave data
            for( int LoopCounter = 1; LoopCounter < TotalWords+1; LoopCounter++ ) {
                /// RATE sensor values
                tempVal = 0.005 * (float)( (int16_t)RxWord[ LoopCounter ] );
                if( tempVal >= 0 ) {
                    DEBUG_STRING( " " ); // align columns
                }

                /// adjust the columns based on value (ones/tens/hundreds)
                signVal = abs( (int8_t)tempVal );
                if( signVal < 10 ) {
                    DEBUG_STRING( "  " );
                } else if( signVal < 100 ) {
                    DEBUG_STRING( " " );
                }
                DEBUG_FLOAT( "  ", tempVal, 4 ); // Write the RATE to the console
            }
            DEBUG_STRING( " >" );
            DEBUG_ENDLINE();
        }
        DelayMs( 1 );
    }

    //
    _InitDataReady( DISABLE );
    _SetSPIClockSpeed( ONE_MHZ );

    // Restore the default number of data collects and the ODR of the 380
    TotalTests = 10;
    ODR_Hz = _SetOutputDataRate( 2 );   // 2 Hz ODR
}

/** ***************************************************************************
 * @name CmdUserSpi_GetSensorValues() command the sensors and read the values off
 * the SPI
 * @brief "sensors" command
 *
 * @param [in] TotalTests - run the test this number of times
 * @param [in] DelayTime - delay between the tests
 * @retval N/A
 ******************************************************************************/
void CmdUserSpi_GetSensorValues( uint32_t data )
{
  //uint16_t RxWord[ 128 ]; // sensor data
  uint8_t  TotalWords = 0; // words read in from the slave
  // (based on burst-read requirements)
  uint8_t  startingAddress = 0x04;
  uint8_t  endingAddress   = 0x0E;
  float    tempVal; // convert from uint to float after the read
  uint8_t  signVal;

    TotalWords = ( ( endingAddress + 2 ) - startingAddress )/2;

  // Populate the variable from the console arguments
  CmdLineGetArgUInt( &TotalTests );
  CmdLineGetArgUInt( &DelayTime );

    // Set the SPI clock speed
    _SetSPIClockSpeed( ONE_MHZ );

    /// Print a header in the terminal window
    if( DebugFlag ) {
        _PrintHeader( TotalWords, startingAddress, TotalTests, ODR_Hz );
    }

  for( uint32_t TrialNumber = 0; TrialNumber < TotalTests; TrialNumber++ )
  {
      ReadFromRegisters( startingAddress, endingAddress, RxWord, 150, ToogleCSFlag );

      if( DebugFlag )
      {   // Print the trial number at the start of the line
          if( ( TrialNumber+1 ) < 10 ) {
              DEBUG_STRING( "   " );
          } else if( ( TrialNumber+1 ) < 100 ) {
              DEBUG_STRING( "  " );
          } else if( ( TrialNumber+1 ) < 1000 ) {
              DEBUG_STRING( " " );
          }
          DEBUG_INT( "",  TrialNumber+1 );
          DEBUG_STRING( ":  <" );

          // Display the data from the SLAVE
          for( int LoopCounter = 1; LoopCounter < TotalWords+1; LoopCounter++ ) {
              if( LoopCounter <= 3 ) {
                  /// RATE sensor values
                  tempVal = 0.005 * (float)( (int16_t)RxWord[ LoopCounter ] );
                  if( tempVal >= 0 ) {
                      DEBUG_STRING( " " ); // align columns
                  }
                  // adjust column width(ones/tens/hundreds)
                  signVal = abs( (int8_t)tempVal );
                  if( signVal < 10 ) {
                      DEBUG_STRING( "  " );
                  } else if( signVal < 100 ) {
                      DEBUG_STRING( " " );
                  }


                  DEBUG_FLOAT( "  ", tempVal, 4 ); // Write the RATE
              } else if( LoopCounter <= 6 ) {
                  /// ACCeleration values
                  tempVal = 0.00025 * (float)( (int16_t)RxWord[ LoopCounter ] );
                  if( tempVal >= 0 ) {
                      DEBUG_STRING( " " );
                  }
                  DEBUG_FLOAT( "  ", tempVal, 4 );
              }
          }
          DEBUG_STRING( " >" );
          DEBUG_ENDLINE();
      }
      DelayMs( DelayTime );
  }
}

/** ***************************************************************************
 * @name CmdUserSpi_SpeedTest() set the CS pin low then high to time a clock tick
 * @brief "boardSpeedTest" command
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void CmdUserSpi_SpeedTest( uint32_t data )
{
    GPIO_ResetBits( SPI3_SLAVE_SELECT_PORT, SPI3_SLAVE_SELECT_PIN ); ///< CS low
    asm("nop");
    GPIO_SetBits( SPI3_SLAVE_SELECT_PORT, SPI3_SLAVE_SELECT_PIN );   ///< CS high
}


/** ***************************************************************************
 * @name    CmdUserSpi_ReadRegister()
 * @brief   Read regisgters on the UUT
 * "r" or "read"
 * Send a request to read registers on the slave (UUT) over the SPI bus. Always
 * reads a word.
 *
 * @param [in] regAddr - address to read from
 * @param [in] numOfWords - number of words to retrieve
 * @param [in] numOfIterations - number of times to repeat
 * @retval N/A
 ******************************************************************************/
void CmdUserSpi_ReadRegister( uint32_t data )
{
    // Define the words (16-bit) to hold the data
    uint16_t waitTime = (uint16_t)( ( 1.0 / (float)ODR_Hz) * 1000.0 + 0.5 );
    uint16_t RxBuffer[ 128 ] = {0};
    uint8_t  startingAddress  = 0x00;
    uint8_t  endingAddress    = 0x00;
    // send the burst-read command and clock in 26 bytes
    //   should match the expected string (0102, 1112, 1314, ... -- ignore the CRC for now)
    uint32_t regAddr         = 0;
    uint32_t numOfWords      = 1;
    uint32_t numOfIterations = 1;
    uint8_t  registerAddress = 0x00;
    float    tempVal         = 0.0;
    uint8_t  signVal         = 1;

    // Populate the variables from the comand line arguments
    CmdLineGetArgUInt( &regAddr );
    CmdLineGetArgUInt( &numOfWords );
    CmdLineGetArgUInt( &numOfIterations );

    startingAddress = (uint8_t)regAddr;
    endingAddress   = startingAddress + 2*( numOfWords-1 );

    _InitDataReady( ENABLE ); // init DR

    // Set the SPI clock speed
    _SetSPIClockSpeed( TWO_MHZ );

    if( DebugFlag ) {
        DEBUG_INT( "Reading ", numOfWords );
        if( numOfWords == 1 ) {
            DEBUG_STRING( " byte" );
        } else {
            DEBUG_STRING( " bytes" );
        }
        DEBUG_HEX( " over SPI from register address 0x", startingAddress );
        DEBUG_INT( " (repeated ", numOfIterations );
        if( numOfIterations == 1 ) {
            DEBUG_STRING( " time)" );
        } else {
            DEBUG_STRING( " times)" );
        }
        DEBUG_ENDLINE();
    }

    for( int IterationNumber = 0; IterationNumber < numOfIterations; IterationNumber++ )
    {
        /// Wait here until the data-ready (DR) line goes low (set in the data-
        ///   ready interrupt handler)before proceeding
        if( WaitForDRDY ) {
            while( !DataReadyContinueFlag )
            {  /* Wait*/   }
        } else {
            DelayMs( waitTime );  // user i pause between readings
        }
        DataReadyContinueFlag = 0;
        ReadFromRegisters( startingAddress, endingAddress, RxBuffer, 105, ToogleCSFlag );

        if( DebugFlag ) {
            for( int LoopCounter = 1; LoopCounter < numOfWords+1; LoopCounter++ )
            {
                registerAddress = startingAddress + 2*( LoopCounter-1 );
                if( registerAddress <= 0x02 )
                {
                    DEBUG_HEX( "0x", RxBuffer[ LoopCounter ] );
                    DEBUG_STRING( "    " );
                }
                else if( registerAddress <= 0x08 )
                {   /// RATE-sensor data
                    tempVal = 0.005 * (float)( (int16_t)RxBuffer[ LoopCounter ] );
                    if( tempVal >= 0 ) {
                        DEBUG_STRING( " " );
                    }

                    signVal = abs( (int)tempVal );
                    if( signVal < 10 ) {
                        DEBUG_STRING( "  " );
                    } else if( signVal < 100 ) {
                        DEBUG_STRING( " " );
                    }
                    DEBUG_FLOAT( "  ", tempVal, 4 );
                }
                else if( registerAddress <= 0x0E )
                {   /// ACCelerometer data
                    tempVal = 0.00025 * (float)( (int16_t)RxBuffer[ LoopCounter ] );
                    if( tempVal > 0 ) {
                        DEBUG_STRING( " " );
                    }
                    DEBUG_FLOAT( "  ", tempVal, 4 );
                }
                else
                {
                    DEBUG_HEX( "0x", RxBuffer[ LoopCounter ] );
                    DEBUG_STRING( "    " );
                }
            }
            DEBUG_ENDLINE();
        }
    }

    _SetSPIClockSpeed( ONE_MHZ );
}

/** ***************************************************************************
 * @name    CmdUserSpi_ReadRegisterHex()
 * @brief   Read regisgters on the UUT display returned values in hex format
 * "rHex" or "readHex"
 * Send a request to read registers on the slave (UUT) over the SPI bus
 *
 * @param [in] regAddr - address to read from
 * @param [in] numOfWords - number of words to retrieve
 * @param [in] numOfIterations - number of times to repeat
 * @param [in] ODR - output data rate of the DMU380 in Hz
 * @retval N/A
 *
 * Example Usage:
 * @code
 *    readhex 4 6 50 5  // return 6 contiguous words starting at register 0x04.
 *                      //   Repeat fifty times at a five hertz rate
 * @endcode
 ******************************************************************************/
void CmdUserSpi_ReadRegisterHex( uint32_t data )
{
    // Define the words (16-bit) to hold the data
    uint16_t RxBuffer[ 128 ] = {0};
    uint16_t waitTime = (uint16_t)( ( 1.0 / (float)ODR_Hz) * 1000.0 + 0.5 );

    // Populate the variable from the console arguments
    uint32_t regAddr         = 0x04;
    uint32_t numOfWords      = 1;
    uint32_t TotalTests      = 10;
    ODR_Hz = 100;
    CmdLineGetArgUInt( &regAddr );
    CmdLineGetArgUInt( &numOfWords );
    CmdLineGetArgUInt( &TotalTests );
    CmdLineGetArgUInt( &ODR_Hz );

    // Get the starting and ending register addresses from input arguments
    uint8_t startingAddress = (uint8_t)regAddr;
    uint8_t endingAddress   = startingAddress + ( 2*numOfWords - 1 );

    // Set the ODR of the 380
    ODR_Hz = _SetOutputDataRate( ODR_Hz );

    // Set the SPI clock speed
    _SetSPIClockSpeed( ONE_MHZ );

    // Initialize the external data-ready interrupt
    _InitDataReady( ENABLE ); ///< init DR

    // Reset the data-ready continuation flag
    DataReadyContinueFlag  = 0;

    //
    if( DebugFlag ) {
        DEBUG_INT( "Reading ", numOfWords );
        if( numOfWords == 1 ) {
            DEBUG_STRING( " byte" );
        } else {
            DEBUG_STRING( " bytes" );
        }
        DEBUG_HEX( " over SPI from register address 0x", startingAddress );
        DEBUG_INT( " (repeated ", TotalTests );
        if( TotalTests == 1 ) {
            DEBUG_STRING( " time)" );
        } else {
            DEBUG_STRING( " times)" );
        }

        DEBUG_INT( " at ", ODR_Hz );
        DEBUG_STRING( " Hz" );
        DEBUG_ENDLINE();
    }

    //
    for( uint32_t IterationNumber = 0; IterationNumber < TotalTests; IterationNumber++ )
    {
        if( WaitForDRDY ) {
            while( !DataReadyContinueFlag )
            {  /* Wait*/   }
        } else {
            DelayMs( waitTime );  // user i pause between readings
        }

        // Reset the wait flag and proceed with the read from the DMU380
        DataReadyContinueFlag = 0;
        ReadFromRegisters( startingAddress, endingAddress, RxBuffer, 150, ToogleCSFlag );

        // Echo the results to the terminal
        if( DebugFlag ) {
            for( int LoopCounter = 1; LoopCounter < numOfWords+1; LoopCounter++ )
            {
                DEBUG_HEX( "0x", RxBuffer[ LoopCounter ] );
                DEBUG_STRING( "    " );
            }
            DEBUG_ENDLINE();
        }

        DelayMs( 1 );
    }

    // Disable DR interrupt and return SPI clock to its default value
    _InitDataReady( DISABLE );
    _SetSPIClockSpeed( ONE_MHZ );

    // Restore default values
    TotalTests = 10;
    ODR_Hz = _SetOutputDataRate( 2 );   // 2 Hz ODR
}

/** ***************************************************************************
 * @name    CmdUserSpi_WriteRegister()
 * @brief   Write a to a SPI register on UUT (client)
 *
 * Send a command over the SPI bus
 *
 * @param [in] RegisterAddress - address to write to
 * @param [in] Message - data to write to the register
 * @param [out] debug serial console
 * @retval N/A6
 ******************************************************************************/
void CmdUserSpi_WriteRegister( uint32_t data )
{
    uint32_t RegisterAddress = 0x71; ///< default - reserved
    uint32_t Message         = 0x0F;

    /// get the variables from the console comand line
    CmdLineGetArgUInt( &RegisterAddress );
    CmdLineGetArgUInt( &Message );

    WriteToRegister( (uint8_t)RegisterAddress, (uint8_t)Message );

    /// Print the data to the serial debug console
    if( DebugFlag ) {
        DEBUG_HEX( "Writing  0x", Message );
        DEBUG_HEX( " over SPI to register address 0x", RegisterAddress );
        DEBUG_ENDLINE();
    }
}

/** ***************************************************************************
 * @name CmdUserSpi_WhoAmI()
 * @brief Send request to UUT (SPI Client) to return Product identification code
 *
 * request product ID through the SPI bus
 *
 * @param [in] data
 * @retval N/A
 ******************************************************************************/
void CmdUserSpi_WhoAmI( uint32_t data )
{
    uint16_t RxBuffer[ 128 ]  = {0};
    uint8_t  wordCount        = 0;
    uint8_t  startingAddress  = 0x56;
    uint8_t  endingAddress    = 0x56;

    uint32_t TotalReads = 1; // Only run the test once

    // Set the SPI clock speed
    _SetSPIClockSpeed( ONE_MHZ );

    // Populate the variable from the console arguments
    CmdLineGetArgUInt( &TotalReads );
    if( TotalReads > 250 ) {
        TotalReads = 250;
    }

    // Perform tests - this loop only runs ONCE
    for( uint32_t readNum = 0; readNum < TotalReads; readNum++ )
    {
        wordCount = ( endingAddress - startingAddress )/2 + 1;
        // Don't wait for the Data-Ready (DR) line to go low just read blind
        ReadFromRegisters( startingAddress, endingAddress, RxBuffer, 150, ToogleCSFlag );

        // Print the data to the debug console (do not print the first word as it
        //   is leftover in the SPI data register)
        if( DebugFlag ) {
            if( readNum == 0 ) {
                DEBUG_INT( "Reading ", wordCount );
                if( wordCount == 1 ) {
                    DEBUG_STRING( " byte" );
                } else {
                    DEBUG_STRING( " bytes" );
                }
                DEBUG_HEX( " over SPI (register address: 0x", startingAddress );
                DEBUG_INT( ") ", TotalReads );
                if( TotalReads == 1 ) {
                    DEBUG_STRING( " time" );
                } else {
                    DEBUG_STRING( " times" );
                }
                DEBUG_ENDLINE();
            }

            if( readNum < 9 ) {
                DEBUG_INT( "   ", readNum+1 );
            } else if( readNum < 99 ) {
                DEBUG_INT( "  ", readNum+1 );
            } else {
                DEBUG_INT( " ", readNum+1 );
            }

            DEBUG_HEX( ": Rx = 0x", RxBuffer[ 1 ] );
            DEBUG_ENDLINE();
        }
        DelayMs( DelayTime );
    }
}


/** ***************************************************************************
 * @name    CmdUserSpiTestSequence01()
 * @brief   write regisgters to the UUT display both sent and returned values
 * "spiCmd01"
 * Send a request to write and read back registers on the slave (UUT) over the
 * SPI bus.
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void CmdUserSpiTestSequence01( uint32_t data )
{
    uint16_t index               = 0;
    uint16_t BytesToTransmit     = 10;
    uint8_t TxBuffer[ 10 ]       = { 0x00 };
    uint8_t RxBuffer[ 10 ]       = { 0x00 };
    uint8_t RegisterAddress      = 0;
    uint8_t StartRegisterAddress = 0x55; // + 10 (0xa) = 0x5f

    TxBuffer[0] = 0x0a;
    TxBuffer[1] = 0x09;
    TxBuffer[2] = 0x08;
    TxBuffer[3] = 0x07;
    TxBuffer[4] = 0x06;
    TxBuffer[5] = 0x05;
    TxBuffer[6] = 0x04;
    TxBuffer[7] = 0x03;
    TxBuffer[8] = 0x02;
    TxBuffer[9] = 0x01;

    // SPI
    for(RegisterAddress = StartRegisterAddress;
          RegisterAddress < ( StartRegisterAddress + BytesToTransmit );
          RegisterAddress++ ) {
         index = RegisterAddress - StartRegisterAddress; // zero based
         RxBuffer[index] = WriteToRegister( RegisterAddress, TxBuffer[index] );
    }

    DEBUG_HEX("Transmission over SPI to register address 0x", RegisterAddress );
    DEBUG_ENDLINE();
    for( index = 0; index < BytesToTransmit; index++ )
    {
        DEBUG_HEX("Tx = 0x", TxBuffer[index] );
        DEBUG_HEX( ", Rx = 0x", RxBuffer[index] );
        DEBUG_ENDLINE();
    }
}

/** ***************************************************************************
 * @name    CmdBlinkLED()
 * @brief   command local (spi master) board to Blink LED4
 *
 * @param [in] numBlinks - number of times to blink
 * @param [in] msApart - speed of the blinks
 * @retval N/A
 ******************************************************************************/
void CmdBlinkLED(uint32_t data)
{
    static uint32_t blinks;
    uint32_t        numBlinks = 3;
    uint32_t        msApart = 500;

    CmdLineGetArgUInt( &numBlinks );
    CmdLineGetArgUInt( &msApart );

    blinks = numBlinks;

    INFO_INT( "Blinking LED4 ", blinks );
    INFO_INT( " times, ", msApart );
    INFO_STRING( " ms apart\r\n" );

    while( blinks )
    {
        led_on(LED4);
        DelayMs(msApart/2);

        led_off(LED4);
        DelayMs(msApart/2);

        blinks--;
    }
    DEBUG_ENDLINE();
}

// Should have a command to initialize to initialize the UART port too
/** ***************************************************************************
 * @name    CmdInitSPIPeripheral()
 * @brief   call LOCAL user SPI3 bus initialization routine
 * "initSpi"
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void CmdInitSPIPeripheral( uint32_t data )
{
    InitCommunication_UserSPI();
}


/** ***************************************************************************
 * @name CmdGetFWVersion_UserSpi() Get the version of the firmware from the
 * slave over the SPI communications port
 * @brief   display returned values
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void CmdGetFWVersion_UserSpi( uint32_t data )
{
    uint8_t RegisterAddress  = 0x41;
    uint16_t RxBuffer[ 4 ]    = { 0 };

    /// Read from reg addr 0x41 to get the fw version running on the DMU380
    ReadFromRegisters( RegisterAddress, RegisterAddress, RxBuffer, 150, ToogleCSFlag );

    // Display results on the terminal
    DEBUG_HEX( "Transmission over SPI to register address 0x", RegisterAddress );
    DEBUG_ENDLINE();

    DEBUG_INT( "", 1 );
    DEBUG_HEX( ", Rx = 0x", RxBuffer[0] );
    DEBUG_ENDLINE();
}





/** ***************************************************************************
 * @name    CmdUserSpi_ReadRegisterHex()
 * @brief   Read regisgters on the UUT display returned values in hex format
 * "rHex" or "readHex"
 * Send a request to read registers on the slave (UUT) over the SPI bus
 *
 * @param [in] regAddr - address to read from
 * @param [in] numOfWords - number of words to retrieve
 * @param [in] numOfIterations - number of times to repeat
 * @param [in] ODR - output data rate of the DMU380 in Hz
 * @retval N/A
 *
 * Example Usage:
 * @code
 *    readhex 4 6 50 5  // return 6 contiguous words starting at register 0x04.
 *                      //   Repeat fifty times at a five hertz rate
 * @endcode
 ******************************************************************************/
void CmdUserSpi_ReadRegisterHex2( uint32_t data )
{
    // Define the words (16-bit) to hold the data
    uint16_t RxBuffer[ 128 ] = {0};

    // Populate the variable from the console arguments
    uint32_t regAddr         = 0x04;
    uint32_t numOfWords      = 1;
    uint32_t TotalTests      = 10;
    ODR_Hz = 2;
    CmdLineGetArgUInt( &regAddr );
    CmdLineGetArgUInt( &numOfWords );
    CmdLineGetArgUInt( &TotalTests );
    CmdLineGetArgUInt( &ODR_Hz );
DelayTime = (int)( 1000 / ODR_Hz );

    // Get the starting and ending register addresses from input arguments
    uint8_t startingAddress = (uint8_t)regAddr;
    uint8_t endingAddress   = startingAddress + ( 2*numOfWords - 1 );

    // Set the ODR of the 380
    ODR_Hz = 4;
//    ODR_Hz = _SetOutputDataRate( ODR_Hz );

    // Set the SPI clock speed
    _SetSPIClockSpeed( ONE_MHZ );

    // Initialize the external data-ready interrupt
//    _InitDataReady( ENABLE ); ///< init DR

    // Reset the data-ready continuation flag
//    DataReadyContinueFlag  = 0;

    //
    if( DebugFlag ) {
        DEBUG_INT( "Reading ", numOfWords );
        if( numOfWords == 1 ) {
            DEBUG_STRING( " byte" );
        } else {
            DEBUG_STRING( " bytes" );
        }
        DEBUG_HEX( " over SPI from register address 0x", startingAddress );
        DEBUG_INT( " (repeated ", TotalTests );
        if( TotalTests == 1 ) {
            DEBUG_STRING( " time)" );
        } else {
            DEBUG_STRING( " times)" );
        }

        DEBUG_INT( " at ", ODR_Hz );
        DEBUG_STRING( " Hz" );
        DEBUG_ENDLINE();
    }

    //
    for( uint32_t IterationNumber = 0; IterationNumber < TotalTests; IterationNumber++ )
    {
        //
//        while( !DataReadyContinueFlag ) ///< DR ISR flag
//        {  /* Wait for the DR signal */ ; }

        // Reset the wait flag and proceed with the read from the DMU380
//        DataReadyContinueFlag = 0;
        ReadFromRegisters( startingAddress, endingAddress, RxBuffer, 150, ToogleCSFlag );

        // Echo the results to the terminal
        if( DebugFlag ) {
            for( int LoopCounter = 1; LoopCounter < numOfWords+1; LoopCounter++ )
            {
                DEBUG_HEX( "0x", RxBuffer[ LoopCounter ] );
                DEBUG_STRING( "    " );
            }
            DEBUG_ENDLINE();
        }

        DelayMs( DelayTime );
    }

    // Disable DR interrupt and return SPI clock to its default value
//    _InitDataReady( DISABLE );
    _SetSPIClockSpeed( ONE_MHZ );

    // Restore default values
    TotalTests = 10;
    ODR_Hz = _SetOutputDataRate( 2 );   // 2 Hz ODR
}

/** ***************************************************************************
 * @name    CmdUserSpi_BurstRead_F2()
 * @brief   Initiate a series of reads using the SPI
 *
 * This function reads data from the DMU380 over the SPI and displays the data
 *   on the console. Shows the following: status word, rates, accelerations,
 *   and temp. * commands a burst of data through the SPI bus
 *
 * @param [in] TotalTests - number of times to repeat the burst read
 * @param [in] DelayTime - number of times to repeat the burst readdelay in ms
 * @retval console
 *
 * Example Usage:
 * @code
 *    burst 3 1; // return 3 sets of data with a 1ms dealy
 * @endcode
 ******************************************************************************/
void CmdUserSpi_BurstRead_F2( uint32_t data )
{
//    float    tempVal;            // convert from uint to float after the read
//    uint8_t  signVal;
//    uint32_t TotalTests     = 10;
//    uint32_t DelayTime      = 500;
//    uint32_t TotalTests     = 20; //10;
//    uint32_t DelayTime      = 100; //500;
    //uint16_t RxWord[ 128 ];      ///< input sensor data buffer

    /// (based on burst-read requirements)
    uint8_t startingAddress = 0x40; // burst mode
    uint8_t endingAddress   = 0x81;
    uint8_t TotalWords = ( ( endingAddress + 1 ) - startingAddress )/2;

uint32_t outputWord;
int16_t outputVal;

uint8_t count;


    _InitDataReady( ENABLE ); ///< set up external interrupt (Data-Ready DR)

    // get the variables from the console arguments
    CmdLineGetArgUInt( &TotalTests );
    CmdLineGetArgUInt( &DelayTime );

    // Set the SPI clock speed
    _SetSPIClockSpeed( TWO_MHZ );

    /// Print a header in the terminal window
    if( DebugFlag ) {
        _PrintHeader( TotalWords, startingAddress, TotalTests, ODR_Hz );
    }

    /// Perform the read the specified number of times 10 or command line input
    for( uint32_t TrialNumber = 0; TrialNumber < TotalTests; TrialNumber++ )
    {
        /// Wait here until the data-ready (DR) line goes low
        while( !DataReadyContinueFlag )
        {  /* Spin */ ; }
        DataReadyContinueFlag = 0;
        ReadFromRegisters( startingAddress, endingAddress, RxWord, 105, ToogleCSFlag );

        if( DebugFlag )
        {
            /// Display the UUT data from the SPI slave
            for( int LoopCounter = 0; LoopCounter < TotalWords+1; LoopCounter++ ) {
                if( LoopCounter == 0 ) {
                    // data left over Tx buffer
                    //DEBUG_HEX( "  ", RxWord[ LoopCounter ] );

                    DEBUG_STRING( "Reading #" );
                    DEBUG_INT( " ", TrialNumber );

                    count = 0;
                } else if( LoopCounter < TotalWords ) {
                    if( LoopCounter == 1 ) {
                        DEBUG_ENDLINE();
                        DEBUG_STRING( "Accel: " );
                    }
                    if( LoopCounter == 7 ) {
                        DEBUG_ENDLINE();
                        DEBUG_STRING( "Rate: " );
                    }
                    if( LoopCounter == 13 ) {
                        DEBUG_ENDLINE();
                        DEBUG_STRING( "Mag: " );
                    }
                    if( LoopCounter == 19 ) {
                        DEBUG_ENDLINE();
                        DEBUG_STRING( "Accel Temp: " );
                    }
                    if( LoopCounter == 25 ) {
                        DEBUG_ENDLINE();
                        DEBUG_STRING( "Rate Temp: " );
                    }
                    if( LoopCounter == 31 ) {
                        DEBUG_ENDLINE();
                        DEBUG_STRING( "Board Temp: " );
                    }

                    if( count == 0 ) {
                        outputWord = RxWord[ LoopCounter ] << 16;
                        count = 1;

//                        DEBUG_HEX( "  ", RxWord[ LoopCounter ] );
                    } else if( count == 1 ) {
                        outputWord = outputWord | RxWord[ LoopCounter ];
                        count = 0;

                        if( LoopCounter < 19 ) {
                            // reading =
                            outputVal = (int16_t)( (int32_t)( outputWord >> 7 ) - 32768 );
                        } else if( LoopCounter < 31 ) {
                            // reading =
                            outputVal = (int16_t)( (int32_t)( outputWord >> 14 ) - 32768 );
                        } else {
                            // reading  = output - 2^15
                            outputVal = (int16_t)( (int32_t)( outputWord ) - 32768 );
                        }

//                        DEBUG_HEX( "", RxWord[ LoopCounter ] );
                        DEBUG_HEX( "   ", outputWord );
                        DEBUG_INT( " = ", outputVal );
                        DEBUG_STRING( " " );
                    } else {
                        DEBUG_HEX( "  ", RxWord[ LoopCounter ] );
                    }
                } else {
                        DEBUG_ENDLINE();
                        DEBUG_STRING( "BIT: " );
                        DEBUG_HEX( "", RxWord[ LoopCounter ] );
                        DEBUG_ENDLINE();
                }
          }
          DEBUG_ENDLINE(); // Print a return character
      }
      DelayMs( DelayTime );  // user i pause between readings
  }
  _InitDataReady( DISABLE ); // re-init DR
    _SetSPIClockSpeed( ONE_MHZ );
}




void CmdUserSpi_BurstForLowGainAHRS( uint32_t data )
{
    //uint16_t RxWord[ 128 ];      ///< input sensor data buffer
//    uint8_t  TotalWords     = 0; ///< words read in from the slave

    /// (based on burst-read requirements)
    uint8_t startingAddress = 0x43; // burst mode
    uint8_t endingAddress   = 0x62;
//    uint8_t TotalWords = ( ( endingAddress + 1 ) - startingAddress )/2;

uint16_t value_uint;
//int16_t value_int;

uint8_t count = 0;

//double sf = 1.0;
//double output_double = 0.0;


static uint16_t swStatusBit = 0;


//    ( hex2dec( 'x' ) + 2 - hex2dec( '3f' ) )/2 = 28

    _InitDataReady( ENABLE ); ///< set up external interrupt (Data-Ready DR)

    // get the variables from the console arguments
    CmdLineGetArgUInt( &TotalTests );
    CmdLineGetArgUInt( &DelayTime );

    // Set the SPI clock speed
    _SetSPIClockSpeed( ONE_MHZ );

    ReadFromRegisters( startingAddress, endingAddress, RxWord, 105, ToogleCSFlag );
    DelayMs( 500 );
    ReadFromRegisters( startingAddress, endingAddress, RxWord, 105, ToogleCSFlag );

    DEBUG_STRING( "AHRS Mode (HG/LG): " );
    // LoopCounter == 12
    value_uint = RxWord[ 16 ];
//    DEBUG_HEX( "   ", value_uint );
    swStatusBit = value_uint & 0x0800;

    if( swStatusBit == 0 ) {
        DEBUG_STRING( "Low-Gain Mode (SW Status Bit set)" );
        DEBUG_ENDLINE();
    } else {
        DEBUG_STRING( "High-Gain Mode (SW Status Bit set)" );
        DEBUG_ENDLINE();

        do {
            ReadFromRegisters( startingAddress, endingAddress, RxWord, 105, ToogleCSFlag );
            count++;
            if( count == 4 ) {
                count = 0;
                DEBUG_STRING( "." );
            }
            DelayMs( 250 );
        } while( ( RxWord[ 16 ]  & 0x0800 ) > 0 );

        DEBUG_ENDLINE();
        DEBUG_ENDLINE();
        DEBUG_STRING( "Low-Gain Mode (SW Status Bit set)" );
        DEBUG_ENDLINE();
    }

   _InitDataReady( DISABLE ); // re-init DR

   DelayMs( 100 );
   WriteToRegister( 0x00, 0x0 );
   DelayMs( 100 );
}


/** ***************************************************************************
 * @name    CmdUserSpi_BurstRead_A1()
 * @brief   Initiate a series of reads of the A1 output packet over the SPI
 *
 * This function reads data from the DMU380 over the SPI and displays the data
 *   on the console. Shows the following: status word, rates, accelerations,
 *   and temp. * commands a burst of data through the SPI bus
 *
 * @param [in] TotalTests - number of times to repeat the burst read
 * @param [in] ODR - output data rate of the 380 in Hz
 * @retval console
 *
 * Example Usage:
 * @code
 *    burst 3 10; // return 3 sets of data at a 10 Hz ODR
 * @endcode
 ******************************************************************************/
void CmdUserSpi_BurstRead_A1( uint32_t data )
{
    //uint16_t RxWord[ 128 ];      ///< input sensor data buffer

    /// (based on burst-read requirements)
    uint8_t startingAddress = 0x43; // burst mode
    uint8_t numberOfBytes   = 32;
    uint8_t endingAddress   = startingAddress + ( numberOfBytes - 1 );
    uint8_t TotalWords = ( ( endingAddress + 1 ) - startingAddress )/2;

    // get the variables from the console arguments
    TotalTests = 10;
    ODR_Hz     = 2;
    CmdLineGetArgUInt( &TotalTests );
    CmdLineGetArgUInt( &ODR_Hz );

    //
    ODR_Hz = _SetOutputDataRate( ODR_Hz );

uint16_t value_uint;
int16_t value_int;

//uint8_t count;

double sf = 1.0;
double output_double = 0.0;

uint16_t reg0x36;

    ReadFromRegisters( 0x36, 0x36, RxWord, 105, ToogleCSFlag );
    reg0x36 = ( RxWord[1] & 0xFF00 ) >> 8;
    DEBUG_HEX( "ODR Reg: 0x", reg0x36 );
    DEBUG_ENDLINE();

    _InitDataReady( ENABLE ); ///< set up external interrupt (Data-Ready DR)

    // Set the SPI clock speed
    _SetSPIClockSpeed( ONE_MHZ );

    if( DebugFlag ) {
        DEBUG_INT( "Burst_A1: Reading ", TotalWords );
        if( TotalWords == 1 ) {
            DEBUG_STRING( " word" );
        } else {
            DEBUG_STRING( " words" );
        }
        DEBUG_HEX( " from SPI register 0x", startingAddress );
        DEBUG_INT( " (repeated ", TotalTests );
        if( TotalTests == 1 ) {
            DEBUG_STRING( " time)" );
        } else {
            DEBUG_STRING( " times)" );
        }

        DEBUG_INT( " at ", ODR_Hz );
        DEBUG_STRING( " Hz" );
        DEBUG_ENDLINE();
    }

    /// Perform the read the specified number of times 10 or command line input
    for( uint32_t TrialNumber = 0; TrialNumber < TotalTests; TrialNumber++ )
    {
        /// Wait here until the data-ready (DR) line goes low
        while( !DataReadyContinueFlag )
        {  /* Spin */ ; }
        DataReadyContinueFlag = 0;
        ReadFromRegisters( startingAddress, endingAddress, RxWord, 105, ToogleCSFlag );

        if( DebugFlag )
        {
            /// Display the UUT data from the SPI slave
            for( int LoopCounter = 0; LoopCounter <= TotalWords; LoopCounter++ ) {
                if( LoopCounter == 0 ) {
                    // data left over Tx buffer
                    //DEBUG_HEX( "  ", RxWord[ LoopCounter ] );

//                    DEBUG_STRING( "Reading #" );
//                    DEBUG_INT( " ", TrialNumber );

//                    count = 0;
                } else if( LoopCounter < TotalWords ) {
                    if( LoopCounter == 1 ) {
//                        DEBUG_ENDLINE();
//                        DEBUG_STRING( "Euler angles: " );
                        sf = 0.005493164062500;
                    }
                    if( LoopCounter == 4 ) {
//                        DEBUG_ENDLINE();
//                        DEBUG_STRING( "Rates: " );
                        sf = 0.019226074218750;
                    }
                    if( LoopCounter == 7 ) {
//                        DEBUG_ENDLINE();
//                        DEBUG_STRING( "Accel: " );
                        sf = 3.051757812500000e-04;
                    }
                    if( LoopCounter == 10 ) {
//                        DEBUG_ENDLINE();
//                        DEBUG_STRING( "Mag: " );
                        sf = 3.051757812500000e-04;  // 20/2^16
                    }
                    if( LoopCounter == 13 ) {
//                        DEBUG_ENDLINE();
//                        DEBUG_STRING( "Rate Temp: " );
                        sf = 0.003051757812500;
                    }
                    if( LoopCounter == 14 ) {
//                        DEBUG_ENDLINE();
//                        DEBUG_STRING( "ITOW: " );
                    }
                    if( LoopCounter == 16 ) {
//                        DEBUG_ENDLINE();
//                        DEBUG_STRING( "BIT: " );
                    }

                    value_uint = RxWord[ LoopCounter ];
                    value_int  = (int16_t)( value_uint );

                    if( LoopCounter <= 3 ) {  // angles
                        output_double = sf * (double)value_int;

                        if( output_double >= 0.0 ) {
                            DEBUG_STRING( " " );
                        }

                        double tmp = fabs( output_double );
                        if( tmp < 10.0 ) {
                            DEBUG_STRING( "  " );
                        } else if( tmp < 100.0 ) {
                            DEBUG_STRING( " " );
                        }

                        DEBUG_FLOAT( "   ", output_double, 4 );
                    } else if( LoopCounter <= 6 ) {  // rates
                        output_double = sf * (double)value_int;

                        if( output_double >= 0.0 ) {
                            DEBUG_STRING( " " );
                        }

                        double tmp = fabs( output_double );
                        if( tmp < 10.0 ) {
                            DEBUG_STRING( "  " );
                        } else if( tmp < 100.0 ) {
                            DEBUG_STRING( " " );
                        }

                        DEBUG_FLOAT( "   ", output_double, 4 );
                    } else if( LoopCounter <= 9 ) {  // accels
                        output_double = sf * (double)value_int;

                        if( output_double >= 0.0 ) {
                            DEBUG_STRING( " " );
                        }

                        DEBUG_FLOAT( "   ", output_double, 4 );
                    } else if( LoopCounter <= 12 ) {  // mag
                        output_double = sf * (double)value_int;

                        if( output_double >= 0.0 ) {
                            DEBUG_STRING( " " );
                        }

                        DEBUG_FLOAT( "   ", output_double, 4 );
                    } else if( LoopCounter == 13 ) {  // temps
                        output_double = sf * (double)value_int;

                        if( output_double >= 0.0 ) {
                            DEBUG_STRING( " " );
                        }

                        double tmp = fabs( output_double );
                        if( tmp < 10.0 ) {
                            DEBUG_STRING( "  " );
                        } else if( tmp < 100.0 ) {
                            DEBUG_STRING( " " );
                        }

                        DEBUG_FLOAT( "   ", output_double, 4 );
                    } else if( LoopCounter == 14 ) {
                        DEBUG_INT( "   ", value_uint )
                    } else {
                        DEBUG_HEX( "   ", RxWord[ LoopCounter ] );
                    }
                } else {  // LoopCounter == 12
//                        DEBUG_ENDLINE();
//                        DEBUG_STRING( "BIT: " );
                    value_uint = RxWord[ LoopCounter ];
                        //DEBUG_HEX( "   ", RxWord[ LoopCounter ] );
                        DEBUG_HEX( "   ", value_uint );
                        //swStatusBit = RxWord[ LoopCounter ] & 0x0800;
//                        uint16_t swStatusBit = value_uint & 0x0800;
                }
          }
          DEBUG_ENDLINE(); // Print a return character
      }
      DelayMs( 1 );  // user i pause between readings
    }

    _InitDataReady( DISABLE ); // re-init DR
    _SetSPIClockSpeed( ONE_MHZ );

//   DelayMs( 100 );
//   WriteToRegister( 0x00, 0x0 );
//   DelayMs( 100 );

    TotalTests = 10;
    ODR_Hz = _SetOutputDataRate( 2 );
}


/** ***************************************************************************
 * @name    CmdUserSpi_ReadRegister()
 * @brief   Read regisgters on the UUT
 * "r" or "read"
 * Send a request to read registers on the slave (UUT) over the SPI bus. Always
 * reads a word.
 *
 * @param [in] regAddr - address to read from
 * @param [in] numOfWords - number of words to retrieve
 * @param [in] numOfIterations - number of times to repeat
 * @retval N/A
 ******************************************************************************/
void CmdUserSpi_ReadRegister2( uint32_t data )
{
    // Define the words (16-bit) to hold the data
    uint16_t RxBuffer[ 128 ] = {0};
    uint8_t  startingAddress  = 0x00;
    uint8_t  endingAddress    = 0x00;
    // send the burst-read command and clock in 26 bytes
    //   should match the expected string (0102, 1112, 1314, ... -- ignore the CRC for now)
    uint32_t regAddr         = 0;
    uint32_t numOfWords      = 1;
    uint32_t numOfIterations = 1;
    uint8_t  registerAddress = 0x00;
    float    tempVal         = 0.0;
    uint8_t  signVal         = 1;

    // Populate the variables from the comand line arguments
    CmdLineGetArgUInt( &regAddr );
    CmdLineGetArgUInt( &numOfWords );
    CmdLineGetArgUInt( &numOfIterations );

    startingAddress = (uint8_t)regAddr;
    endingAddress   = startingAddress + 2*( numOfWords-1 );

    _InitDataReady( ENABLE ); // init DR

    // Set the SPI clock speed
    _SetSPIClockSpeed( TWO_MHZ );

    if( DebugFlag ) {
        DEBUG_INT( "Reading ", numOfWords );
        if( numOfWords == 1 ) {
            DEBUG_STRING( " byte" );
        } else {
            DEBUG_STRING( " bytes" );
        }
        DEBUG_HEX( " over SPI from register address 0x", startingAddress );
        DEBUG_INT( " (repeated ", numOfIterations );
        if( numOfIterations == 1 ) {
            DEBUG_STRING( " time)" );
        } else {
            DEBUG_STRING( " times)" );
        }
        DEBUG_ENDLINE();
    }

    for( int IterationNumber = 0; IterationNumber < numOfIterations; IterationNumber++ )
    {
        while( !DataReadyContinueFlag ) ///< DR - B3 ISR hit (active low)
        {  /* Spin */ ; }
        DataReadyContinueFlag = 0;
        ReadFromRegisters( startingAddress, endingAddress, RxBuffer, 105, ToogleCSFlag );

        if( DebugFlag ) {
            for( int LoopCounter = 1; LoopCounter < numOfWords+1; LoopCounter++ )
            {
                registerAddress = startingAddress + 2*( LoopCounter-1 );
                if( registerAddress <= 0x02 )
                {
                    DEBUG_HEX( "0x", RxBuffer[ LoopCounter ] );
                    DEBUG_STRING( "    " );
                }
                else if( registerAddress <= 0x08 )
                {   /// RATE-sensor data
                    tempVal = 0.005 * (float)( (int16_t)RxBuffer[ LoopCounter ] );
                    if( tempVal >= 0 ) {
                        DEBUG_STRING( " " );
                    }

                    signVal = abs( (int)tempVal );
                    if( signVal < 10 ) {
                        DEBUG_STRING( "  " );
                    } else if( signVal < 100 ) {
                        DEBUG_STRING( " " );
                    }
                    DEBUG_FLOAT( "  ", tempVal, 4 );
                }
                else if( registerAddress <= 0x0E )
                {   /// ACCelerometer data
                    tempVal = 0.00025 * (float)( (int16_t)RxBuffer[ LoopCounter ] );
                    if( tempVal > 0 ) {
                        DEBUG_STRING( " " );
                    }
                    DEBUG_FLOAT( "  ", tempVal, 4 );
                }
                else
                {
                    DEBUG_HEX( "0x", RxBuffer[ LoopCounter ] );
                    DEBUG_STRING( "    " );
                }
            }
            DEBUG_ENDLINE();
        }
    }

    _SetSPIClockSpeed( ONE_MHZ );
}


/** ***************************************************************************
 * @name CmdUserSpi_burstReadSwitch() A series of commands that test the burst
 *       read functionality by switching between several different bursts
 *       (JD Burst, S0 Burst, S1 Burst, A1 Burst, F1 Burst, repeat)
 * @brief "j" command
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void CmdUserSpi_burstReadSwitch( uint32_t data )
{

    uint16_t numberOfRuns = 5;
    uint16_t runNumber = 0;

// F1 (54 bytes = 27 words: reg 0x3F --> 0x74)
// F2 (66 bytes = 33 words: reg 0x40 --> 0x81)
// S0 (30 bytes = 15 words: reg 0x41 --> 0x5E)
// S1 (24 bytes = 12 words: reg 0x42 --> 0x59)
// A1 (32 bytes = 16 words: reg 0x43 --> 0x62)

    // Speeds:
    // 0 = 0.25 MHz
    // 1 = 0.50 MHz
    // 2 = 1.00 MHz
    // 3 = 2.00 MHz
    // Increase the SPI clock to 2 MHz (clock rate set at the start of CmdUserSpi_BurstRead(), ...)
    _SetSPIClockSpeed( TWO_MHZ );

    if( 0 ) {
    for( runNumber = 0; runNumber < numberOfRuns; runNumber++ )
    {
        // Can switch between packets with up to a 1 MHz SPI clock, once a packet is selected and
        //   running, the clock can be up to 2 MHz.  Points to a startup artifact.
        DEBUG_INT( "Run #: ", runNumber );
        DEBUG_ENDLINE();
        CmdUserSpi_BurstRead(0);
        CmdUserSpi_BurstRead_S0(0);
        CmdUserSpi_BurstRead_S1(0);
    }
} else {
    for( runNumber = 0; runNumber < numberOfRuns; runNumber++ )
    {
        DEBUG_INT( "Run #: ", runNumber );
        DEBUG_ENDLINE();
        CmdUserSpi_BurstRead(0);
        CmdUserSpi_BurstRead_S0(0);
        CmdUserSpi_BurstRead_S1(0);
        CmdUserSpi_BurstRead_A1(0);
 //       CmdUserSpi_BurstRead_F1(0);
        //CmdUserSpi_BurstRead_F2(0);
    }
}

    // Return the SPI clock to 1 MHz
    _SetSPIClockSpeed( ONE_MHZ );
}

#define TWOPI 6.283185307179586
#define PI 3.141592653589793

void CmdUserSpi_MagAlign( uint32_t data )
{
    uint16_t RxBuffer[ 128 ] = {0};
    uint16_t reg50 = 0x0000;

    uint32_t delay = 250;     // ms

    uint16_t escapeCntr;

    //uint16_t RxWord[ 128 ];
    uint8_t startingAddress = 0; // registertest 126
    uint8_t endingAddress   = 0;

    float hardIronBias[2];    ///< [-1,1) Gauss
    float softIronScaleRatio; ///< [0,2), [0-200%)
    float softIronAngle;

    /// kickoff the mag-align (write 80 0)
    WriteToRegister( 0x50, 0x00 );

    // Set the SPI speed (clock speed of the master)
    spi_go_slightly_faster( SPI3 ); // 1.0 MHz

    DEBUG_STRING( "Begin Mag-Align:" );
    DEBUG_ENDLINE();

    escapeCntr = 0;
    do {
        ReadFromRegisters( 0x50, 0x50, RxBuffer, 425, ToogleCSFlag ); // 425 ~ 27usec
        DEBUG_STRING( "." );
        reg50 = RxBuffer[1];
        escapeCntr++;
        DelayMs( delay );
    } while( ( reg50 & 0x00FF ) != 0x0B );

    DEBUG_ENDLINE();
    DEBUG_STRING( "Mag-Align Complete:" );
    DEBUG_ENDLINE();

    // done: read back the values from the alignment
    // read 0x48, 0x4a, 0x4c, 0x4e
    startingAddress = 0x48; // registertest 126
    endingAddress   = 0x50;
    ReadFromRegisters( startingAddress, endingAddress, RxWord, 150, ToogleCSFlag );
    // raw debug values
    DEBUG_HEX( "0x", RxWord[1] );
    DEBUG_ENDLINE();
    DEBUG_HEX( "0x", RxWord[2] );
    DEBUG_ENDLINE();
    DEBUG_HEX( "0x", RxWord[3] );
    DEBUG_ENDLINE();
    DEBUG_HEX( "0x", RxWord[4] );
    DEBUG_ENDLINE();
    DEBUG_HEX( "0x", RxWord[5] );
    DEBUG_ENDLINE();

    // scaled from NavView scaling to user
    hardIronBias[0] = (float)((int16_t)RxWord[1]) * 20. / (float)pow(2,16);    ///< [-1,1) Gauss
    hardIronBias[1] = (float)((int16_t)RxWord[2]) * 20. / (float)pow(2,16);    ///< [-1,1) Gauss
    softIronScaleRatio = (float)RxWord[3]  * 2. / (float)pow(2,16); ///< [0,2), [0-200%)
    softIronAngle = (float)((int16_t)RxWord[4]) * ( 2.0*PI ) / (float)( pow(2,15) - 1 ) * 180./PI;

    DEBUG_FLOAT( "Hard iron bias 0 = ", hardIronBias[0], 4 );
    DEBUG_ENDLINE();
    DEBUG_FLOAT( "Hard iron bias 1 = ", hardIronBias[1], 4 );
    DEBUG_ENDLINE();
    DEBUG_FLOAT( "Soft iron scale ratio = ", softIronScaleRatio, 4 );
    DEBUG_ENDLINE();
    DEBUG_FLOAT( "Soft iron Angle = ", softIronAngle, 4 );
    DEBUG_ENDLINE();

     WriteToRegister( 0x00, 0x0 );

}



/** ***************************************************************************
 * @name CmdUserSpi_IndraTesting() A series of commands that test the John
 *       Deere initialize
 * @brief "j" command
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void CmdUserSpi_IndraTesting( uint32_t data )
{
    // The goal of this test function is to test the SPI interface and
    //   communication with the 380 as it is used by Indra.  This includes the
    //   following:
    //   1) Set up the unit to output data at 50 Hz
    //   2) Read the A1 packet augmented by data from two additional registers
    //      (y and z-rate: registers 0x06 and 0x08)
    //   3) Should also test (as was done for JD) that toggles of the reset and
    //      nSS lines do not cause problems with the unit.

    float    tempVal;       // convert from uint to float after the read
    uint8_t  signVal;

    //uint16_t RxWord[ 128 ] = { 0 };

    // Specify times in milliseconds
    uint16_t pinLowTime[2]  = {  500, 500 };
    uint16_t pinHighTime[2] = { 2000, 500 };

    /// (based on burst-read requirements)
    uint8_t startingAddress[3] = { 0x43, 0x06, 0x08 }; // burst mode (A1
    uint8_t numberOfBytes[3]   = { 32, 2, 2 };
    uint8_t endingAddress[3];
    uint8_t TotalWords[3];
    for( int cnt = 0; cnt < 3; cnt++ ) {
        endingAddress[cnt] = startingAddress[cnt] + ( numberOfBytes[cnt] - 1 );
        TotalWords[cnt]    = ( ( endingAddress[cnt] + 1 ) - startingAddress[cnt] )/2;
    }

    //uint16_t testNumber;
    uint8_t toggleFlag = 1;

    uint32_t numOfTimeouts = 0;

    // Specify the default number of toggles of the nRst and nSS lines (can be
    //   overwritten via input arguments)
    uint32_t numberOfToggles[2] = { 5, 5 };

    // Specify the default number of tests (consisting of alternating nRst and
    //   nSS toggles).  Can be overwritten via input arguments.
    uint32_t numberOfTests = 3;

    // Get the variables from the console arguments and overwrite the default
    //   values
    CmdLineGetArgUInt( &numberOfTests );
    CmdLineGetArgUInt( &numberOfToggles[0] );
    CmdLineGetArgUInt( &numberOfToggles[1] );

    // Limit the number of tests and toggles
    if( numberOfTests > 65500 ) {
        numberOfTests = 65500;
    }

    if( numberOfToggles[0] > 65500 ) {
        numberOfToggles[0] = 65500;
    }

    if( numberOfToggles[1] > 65500 ) {
        numberOfToggles[1] = 65500;
    }

    // Flag used to wait in place for the data-ready line before reading sensor
    //  data
    DataReadyContinueFlag = 0;

    // Display flag -- display every tenth reading
    uint8_t TrialCounter = 10;

    // Set SPI clock speed (Indra uses 1 MHz)
    _SetSPIClockSpeed( ONE_MHZ );

    // Get the product ID and let the user know if it is correct
    _WhoAmI( 1 );
    DelayMs( 500 );

    // Set up the external data-ready (DR) interrupt.  'DataReadyContinueFlag'
    //   toggles when new data is available from the 380.
    _InitDataReady( ENABLE );
    DelayMs( 200 );

    // Remove power to the board, set nSS high, set nRst low
    DEBUG_STRING( "Remove power from the 380 and set nRst low" );
    DEBUG_ENDLINE();
    GPIOE->BSRRH = GPIO_Pin_0;  // Set E0 Low (remove power)
    SPI3_SLAVE_SELECT_PORT->BSRRL = SPI3_SLAVE_SELECT_PIN;
    GPIOE->BSRRH = GPIO_Pin_2;

    // Wait ...
    DelayMs( 2000 );

    // Power on board
    DEBUG_STRING( "Restore power to the 380 and release nRst 1 second after power applied" );
    DEBUG_ENDLINE();
    GPIOE->BSRRL = GPIO_Pin_0;  // Set E0 high

    // Set reset (E2) high after 400 msec
    DelayMs( 400 );
    GPIOE->BSRRL = GPIO_Pin_2;

    // Wait 100 msec before talking to unit
    DEBUG_ENDLINE();
    DEBUG_STRING( "Begin configuration of 380..." );
    DEBUG_ENDLINE();
    DelayMs( 100 );

    // Send the Indra configuration commands
    _SendIndraConfigCmds();

    uint32_t loopCntr = 0;
    uint32_t loopCntrLimit = 5000;

    // **************** READ DATA ****************
    DEBUG_STRING( "Configuration complete." );
    DEBUG_ENDLINE();
    DelayMs( 500 );

    DEBUG_ENDLINE();
    DEBUG_STRING( "Begin post-configuration data read:" );
    DEBUG_ENDLINE();
    DelayMs( 500 );

    DataReadyContinueFlag = 0;
    /// Burst-read: perform the read a specified number of times
    for( int TrialNumber = 0; TrialNumber < 100; TrialNumber++ )
    {
        while( !DataReadyContinueFlag ) {
            /* Wait here until the data-ready line is set by the 380 */
        }
        DataReadyContinueFlag = 0;

        ReadFromRegisters( startingAddress[0], endingAddress[0], &RxWord[0], 105, ToogleCSFlag );
        ReadFromRegisters( startingAddress[1], endingAddress[1], &RxWord[17], 105, ToogleCSFlag );
        ReadFromRegisters( startingAddress[2], endingAddress[2], &RxWord[19], 105, ToogleCSFlag );

        TrialCounter = TrialCounter + 1;

        if( TrialCounter >= 10 ) {
            TrialCounter = 0;
            /// Display the UUT data from the SPI slave
            for( int LoopCounter = 1; LoopCounter < TotalWords[0]+1; LoopCounter++ ) {
                if( LoopCounter <= 3 ) {
                    /// Angle conversion
                    tempVal = 0.005493164062500 * (float)( (int16_t)RxWord[ LoopCounter ] );
                    if( tempVal >= 0 ) {
                        DEBUG_STRING( " " ); // ' ' instead of '-' for positive data
                    }

                    /// Spaces to adjust the columns based on value (ones/tens/hundreds)
                    signVal = abs( (int8_t)tempVal );
                    if( signVal < 10 ) {
                        DEBUG_STRING( "  " );
                    } else if( signVal < 100 ) {
                        DEBUG_STRING( " " );
                    }

                    /// Write the RATE data to the console
                    DEBUG_FLOAT( "  ", tempVal, 4 );
                } else if ( LoopCounter <= 6 ) {
                    /// RATE sensor conversion
                    tempVal = 0.019226074218750 * (float)( (int16_t)RxWord[ LoopCounter ] );
                    if( tempVal >= 0 ) {
                        DEBUG_STRING( " " ); // ' ' instead of '-' for positive data
                    }

                    /// Spaces to adjust the columns based on value (ones/tens/hundreds)
                    signVal = abs( (int8_t)tempVal );
                    if( signVal < 10 ) {
                        DEBUG_STRING( "  " );
                    } else if( signVal < 100 ) {
                        DEBUG_STRING( " " );
                    }

                    /// Write the RATE data to the console
                    DEBUG_FLOAT( "  ", tempVal, 4 );
                } else if ( LoopCounter <= 9 ) {
                    /// ACCELeration conversion
                    tempVal = 3.051757812500000e-04 * (float)( (int16_t)RxWord[ LoopCounter ] );
                    if( tempVal >= 0 ) {
                        DEBUG_STRING( " " );
                    }
                    DEBUG_FLOAT( "  ", tempVal, 4 );
                } else if ( LoopCounter <= 12 ) {
                    /// Magnetometer conversion
                    tempVal = 3.051757812500000e-04 * (float)( (int16_t)RxWord[ LoopCounter ] );
                    if( tempVal >= 0 ) {
                        DEBUG_STRING( " " );
                    }
                    DEBUG_FLOAT( "  ", tempVal, 4 );
                } else if ( LoopCounter <= 13 ) {
                    /// board TEMPERATURE - conversion
                    DEBUG_FLOAT( "  ", 0.003051757812500 * (float)( (int16_t)RxWord[ LoopCounter ] ), 4 );
                    DEBUG_STRING( "  " );
                } else if ( LoopCounter <= 15 ) {
                    DEBUG_HEX( "", RxWord[ LoopCounter ] );
                } else {
                    DEBUG_HEX( "  ", RxWord[ LoopCounter ] );
                }
            }

            /// RATE sensor Y conversion
            tempVal = 0.005 * (float)( (int16_t)RxWord[18] );
            if( tempVal >= 0 ) {
                DEBUG_STRING( " " ); // ' ' instead of '-' for positive data
            }

            /// Spaces to adjust the columns based on value (ones/tens/hundreds)
            signVal = abs( (int8_t)tempVal );
            if( signVal < 10 ) {
                DEBUG_STRING( "  " );
            } else if( signVal < 100 ) {
                DEBUG_STRING( " " );
            }

            /// Write the RATE data to the console
            DEBUG_FLOAT( "  ", tempVal, 4 );

            /// RATE sensor Z conversion
            tempVal = 0.005 * (float)( (int16_t)RxWord[20] );
            if( tempVal >= 0 ) {
                DEBUG_STRING( " " ); // ' ' instead of '-' for positive data
            }

            /// Spaces to adjust the columns based on value (ones/tens/hundreds)
            signVal = abs( (int8_t)tempVal );
            if( signVal < 10 ) {
                DEBUG_STRING( "  " );
            } else if( signVal < 100 ) {
                DEBUG_STRING( " " );
            }

            /// Write the RATE data to the console
            DEBUG_FLOAT( "  ", tempVal, 4 );

            DEBUG_ENDLINE(); // Print a return character
        }
        DelayMs( 1 );
    }

    // Perform the nSS/nRst toggle tests
    for( uint16_t testNumber = 0; testNumber < 2*numberOfTests; testNumber++ ) {
        toggleFlag = toggleFlag + 1;
        if( toggleFlag > 1 ) {
            toggleFlag = 0;
        }

        DEBUG_ENDLINE();
        if( toggleFlag == 0 ) {
            DEBUG_INT( "Toggling nRst ", numberOfToggles[0] );
        } else {
            DEBUG_INT( "Toggling nSS ", numberOfToggles[1] );
        }
        DEBUG_INT( " times followed by data read (test ", testNumber+1 );
        DEBUG_INT( "/", 2*numberOfTests );
        DEBUG_STRING( "):  " );
        DelayMs( 500 );

        // Toggle the nSS or nRst pin based on input arguments
        _ToggleInputPin( toggleFlag,
                         numberOfToggles[toggleFlag],
                         pinLowTime[toggleFlag],
                         pinHighTime[toggleFlag] );

        DataReadyContinueFlag = 0;
        /// Burst-read: perform the read a specified number of times
        for( int TrialNumber = 0; TrialNumber < 100; TrialNumber++ )
        {
            while( !DataReadyContinueFlag ) {
                /* Wait here until the data-ready line is set by the 380.  If
                   the master waits for longer than 10000 cycles then display
                   a message and syst */
                loopCntr++;
                DelayMs(1);
                if( loopCntr > loopCntrLimit ) {
                    if( toggleFlag == 0 ) {
                        DEBUG_STRING( "   Time-out during nSS toggle ..." );
                        DEBUG_ENDLINE();
                        DEBUG_STRING( "   system being reset (toggling nRst)" );
DelayMs( 50 );
_WhoAmI( 1 );
DelayMs( 50 );

                        //
                        _ToggleInputPin( NRST, 1, pinLowTime[NRST], pinHighTime[NRST] );
                        DelayMs( 2000 );
                    } else {
                        DEBUG_STRING( "   Time-out during nRst toggle ..." );
DelayMs( 50 );
_WhoAmI( 1 );
DelayMs( 50 );
                    }
                    DEBUG_ENDLINE();
                    DEBUG_ENDLINE();

                    numOfTimeouts++;

                    // Exit from the loop and continue with the next test
                    break;
                }
            }
            DataReadyContinueFlag = 0;
            loopCntr = 0;

            ReadFromRegisters( startingAddress[0], endingAddress[0], &RxWord[0], 105, ToogleCSFlag );
            ReadFromRegisters( startingAddress[1], endingAddress[1], &RxWord[17], 105, ToogleCSFlag );
            ReadFromRegisters( startingAddress[2], endingAddress[2], &RxWord[19], 105, ToogleCSFlag );

            TrialCounter = TrialCounter + 1;

            if( TrialCounter >= 10 ) {
                TrialCounter = 0;
                /// Display the UUT data from the SPI slave
                for( int LoopCounter = 1; LoopCounter < TotalWords[0]+1; LoopCounter++ ) {
                    if( LoopCounter <= 3 ) {
                        /// Angle conversion
                        tempVal = 0.005493164062500 * (float)( (int16_t)RxWord[ LoopCounter ] );
                        if( tempVal >= 0 ) {
                            DEBUG_STRING( " " ); // ' ' instead of '-' for positive data
                        }

                        /// Spaces to adjust the columns based on value (ones/tens/hundreds)
                        signVal = abs( (int8_t)tempVal );
                        if( signVal < 10 ) {
                            DEBUG_STRING( "  " );
                        } else if( signVal < 100 ) {
                            DEBUG_STRING( " " );
                        }

                        /// Write the RATE data to the console
                        DEBUG_FLOAT( "  ", tempVal, 4 );
                    } else if ( LoopCounter <= 6 ) {
                        /// RATE sensor conversion
                        tempVal = 0.019226074218750 * (float)( (int16_t)RxWord[ LoopCounter ] );
                        if( tempVal >= 0 ) {
                            DEBUG_STRING( " " ); // ' ' instead of '-' for positive data
                        }

                        /// Spaces to adjust the columns based on value (ones/tens/hundreds)
                        signVal = abs( (int8_t)tempVal );
                        if( signVal < 10 ) {
                            DEBUG_STRING( "  " );
                        } else if( signVal < 100 ) {
                            DEBUG_STRING( " " );
                        }

                        /// Write the RATE data to the console
                        DEBUG_FLOAT( "  ", tempVal, 4 );
                    } else if ( LoopCounter <= 9 ) {
                        /// ACCELeration conversion
                        tempVal = 3.051757812500000e-04 * (float)( (int16_t)RxWord[ LoopCounter ] );
                        if( tempVal >= 0 ) {
                            DEBUG_STRING( " " );
                        }
                        DEBUG_FLOAT( "   ", tempVal, 4 );
                    } else if ( LoopCounter <= 12 ) {
                        /// Magnetometer conversion
                        tempVal = 3.051757812500000e-04 * (float)( (int16_t)RxWord[ LoopCounter ] );
                        if( tempVal >= 0 ) {
                            DEBUG_STRING( " " );
                        }
                        DEBUG_FLOAT( "   ", tempVal, 4 );
                    } else if ( LoopCounter <= 13 ) {
                        /// board TEMPERATURE - conversion
                        DEBUG_FLOAT( "   ", 0.003051757812500 * (float)( (int16_t)RxWord[ LoopCounter ] ), 4 );
                    } else if ( LoopCounter <= 15 ) {
                        DEBUG_HEX( "   ", RxWord[ LoopCounter ] );
                    } else {
                        DEBUG_HEX( "   ", RxWord[ LoopCounter ] );
                    }
                }

                /// RATE sensor Y conversion
                tempVal = 0.005 * (float)( (int16_t)RxWord[18] );
                if( tempVal >= 0 ) {
                    DEBUG_STRING( " " ); // ' ' instead of '-' for positive data
                }

                /// Spaces to adjust the columns based on value (ones/tens/hundreds)
                signVal = abs( (int8_t)tempVal );
                if( signVal < 10 ) {
                    DEBUG_STRING( "  " );
                } else if( signVal < 100 ) {
                    DEBUG_STRING( " " );
                }

                /// Write the RATE data to the console
                DEBUG_FLOAT( "  ", tempVal, 4 );

                /// RATE sensor Z conversion
                tempVal = 0.005 * (float)( (int16_t)RxWord[20] );
                if( tempVal >= 0 ) {
                    DEBUG_STRING( " " ); // ' ' instead of '-' for positive data
                }

                /// Spaces to adjust the columns based on value (ones/tens/hundreds)
                signVal = abs( (int8_t)tempVal );
                if( signVal < 10 ) {
                    DEBUG_STRING( "  " );
                } else if( signVal < 100 ) {
                    DEBUG_STRING( " " );
                }

                /// Write the RATE data to the console
                DEBUG_FLOAT( "  ", tempVal, 4 );

                DEBUG_ENDLINE(); // Print a return character
            }
            DelayMs( 1 );
        }
    }

    DEBUG_ENDLINE();
    DEBUG_STRING( "Initialization test complete" );
    DEBUG_INT( " (number of timeouts: ", numOfTimeouts );
    DEBUG_STRING( ")" );
    DEBUG_ENDLINE();
    DEBUG_ENDLINE();
    DelayMs( 500 );

    // Reset the SPI clock to 1 MHz and disable the data-ready interrupt
    _SetSPIClockSpeed( ONE_MHZ );
    _InitDataReady( DISABLE );
}

uint8_t _WhoAmI( uint8_t debugFlag )
{
    uint16_t RxBuffer[ 65 ] = { 0 };
    uint8_t retVal;

    ReadFromRegisters( 0x56, 0x56, RxBuffer, 150, ToogleCSFlag ); /// product ID (0x3810) 150 ~10usec

    if( debugFlag ) {
        if( RxBuffer[1] == 0x3810 ) {
            DEBUG_STRING( "Product ID correct" );
            retVal = 1;
        } else {
            DEBUG_STRING( "Invalid Product ID" );
            retVal = 0;
        }
        DEBUG_ENDLINE();
        DEBUG_ENDLINE();
    }

    return( retVal );
}

void _SendJDConfigCmds( void )
{
    uint16_t i;
    uint16_t RxBuffer[ 65 ] = { 0 };

    // Check the Status register
    ReadFromRegisters( 0x3C, 0x3C, RxBuffer, 425, ToogleCSFlag );
    for( i = 0; i <= 100; i++ )
    { /* spin */ } // Delay for 23usec

    // Check the self-test register
    ReadFromRegisters( 0x34, 0x34, RxBuffer, 425, ToogleCSFlag );
    for( i = 0; i <= 100; i++ )
    { /* spin */ } // Delay for 23usec

    // Read the product ID
    ReadFromRegisters( 0x56, 0x56, RxBuffer, 425, ToogleCSFlag );
    for( i = 0; i <= 100; i++ )
    { /* spin */ } // Delay for 23usec

    // Read the product ODR/System clock register
    ReadFromRegisters( 0x36, 0x36, RxBuffer, 425, ToogleCSFlag );
    for( i = 0; i <= 100; i++ )
    { /* spin */ } // Delay for 23usec

    // Read the dynamic range/LPF register
    ReadFromRegisters( 0x38, 0x38, RxBuffer, 425, ToogleCSFlag );
    for( i = 0; i <= 100; i++ )
    { /* spin */ } // Delay for 23usec

    // Set the ODR to 200 Hz
//    WriteToRegister( 0x37, 0x01 );
    WriteToRegister( 0x37, 0x03 );
    for( i = 0; i <= 380; i++ )
    { /* spin */ }   // Delay for 23usec
}


void _SendIndraConfigCmds( void )
{
    uint16_t i;
    uint16_t RxBuffer[ 65 ] = { 0 };

    // Check the Status register
    ReadFromRegisters( 0x3C, 0x3C, RxBuffer, 425, ToogleCSFlag );
    for( i = 0; i <= 100; i++ )
    { /* spin */ } // Delay for 23usec

    // Check the self-test register
    ReadFromRegisters( 0x34, 0x34, RxBuffer, 425, ToogleCSFlag );
    for( i = 0; i <= 100; i++ )
    { /* spin */ } // Delay for 23usec

    // Read the product ID
    ReadFromRegisters( 0x56, 0x56, RxBuffer, 425, ToogleCSFlag );
    for( i = 0; i <= 100; i++ )
    { /* spin */ } // Delay for 23usec

    // Read the product ODR/System clock register
    ReadFromRegisters( 0x36, 0x36, RxBuffer, 425, ToogleCSFlag );
    for( i = 0; i <= 100; i++ )
    { /* spin */ } // Delay for 23usec

    // Read the dynamic range/LPF register
    ReadFromRegisters( 0x38, 0x38, RxBuffer, 425, ToogleCSFlag );
    for( i = 0; i <= 100; i++ )
    { /* spin */ } // Delay for 23usec

    // Set the ODR to 50 Hz
    WriteToRegister( 0x37, 0x06 );
    for( i = 0; i <= 380; i++ )
    { /* spin */ }   // Delay for 23usec
}



//
// pinType == 0: toggle nRst pin
// pinType == 1: toggle nSS pin
//
void _ToggleInputPin( uint8_t pinType, uint16_t numberOfToggles, uint16_t lowPeriod, uint16_t highPeriod )
{
    uint32_t cntr = 0;

    // Toggle nRst
    for( int TrialNumber = 0; TrialNumber < numberOfToggles; TrialNumber++ )
    {
        // set pin LOW for prescribed time period
        if( pinType == 0 ) {
            GPIOE->BSRRH = GPIO_Pin_2;
        } else {
            SPI3_SLAVE_SELECT_PORT->BSRRH = SPI3_SLAVE_SELECT_PIN;
        }
        DelayMs( lowPeriod );

        // set pin HIGH for prescribed time period
        if( pinType == 0 ) {
            GPIOE->BSRRL = GPIO_Pin_2;
        } else {
            SPI3_SLAVE_SELECT_PORT->BSRRL = SPI3_SLAVE_SELECT_PIN;
        }
        DelayMs( highPeriod );

        // backup cursor
        if( cntr < 10 ) {
            DebugPrintString("\b");
        } else if( cntr < 100 ) {
            DebugPrintString("\b");
            DebugPrintString("\b");
        } else if( cntr < 1000 ) {
            DebugPrintString("\b");
            DebugPrintString("\b");
            DebugPrintString("\b");
        } else if( cntr < 10000 ) {
            DebugPrintString("\b");
            DebugPrintString("\b");
            DebugPrintString("\b");
            DebugPrintString("\b");
        } else {
            DebugPrintString("\b");
            DebugPrintString("\b");
            DebugPrintString("\b");
            DebugPrintString("\b");
            DebugPrintString("\b");
        }

        cntr++;
        DEBUG_INT( "", cntr );
    }
    DEBUG_ENDLINE();
}


// Set the output data rate of the 380 by writing to register 0x37
uint32_t _SetOutputDataRate( uint32_t ODR_Hz )
{
    if( ODR_Hz >= 150 ) {
        ODR_Hz = 200;
        WriteToRegister( 0x37, 0x01 );   // 200 Hz ODR
    } else if( ODR_Hz >= 75 ) {
        ODR_Hz = 100;
        WriteToRegister( 0x37, 0x02 );   // 100 Hz ODR
    } else if( ODR_Hz >= 37 ) {
        ODR_Hz = 50;
        WriteToRegister( 0x37, 0x03 );   // 50 Hz ODR
    } else if( ODR_Hz >= 23 ) {
        ODR_Hz = 25;
        WriteToRegister( 0x37, 0x04 );   // 25 Hz ODR
    } else if( ODR_Hz >= 15 ) {
        ODR_Hz = 20;
        WriteToRegister( 0x37, 0x05 );   // 20 Hz ODR
    } else if( ODR_Hz >= 8 ) {
        ODR_Hz = 10;
        WriteToRegister( 0x37, 0x06 );   // 10 Hz ODR
    } else if( ODR_Hz >= 5 ) {
        ODR_Hz = 5;
        WriteToRegister( 0x37, 0x07 );   // 5 Hz ODR
    } else if( ODR_Hz >= 3 ) {
        ODR_Hz = 4;
        WriteToRegister( 0x37, 0x08 );   // 4 Hz ODR
    } else if( ODR_Hz >= 2 ) {
        ODR_Hz = 2;
        WriteToRegister( 0x37, 0x09 );   // 2 Hz ODR
    } else if( ODR_Hz >= 1 ) {
        ODR_Hz = 1;
        WriteToRegister( 0x37, 0x0A );   // 1 Hz ODR
    } else if( ODR_Hz == 0 ) {
        ODR_Hz = 0;
        WriteToRegister( 0x37, 0x00 );   // 0 Hz ODR
    }

    return ODR_Hz;
}


int32_t nabs( int32_t signedInteger )
{
    if( signedInteger >= 0 ) {
        return signedInteger;
    } else {
        return -signedInteger;
    }
}

// Print a header at the start of the output data
void _PrintHeader( uint8_t TotalWords, uint8_t startingAddress, uint32_t TotalTests, uint32_t ODR_Hz )
{
    /// Print a header in the terminal window
    DEBUG_INT( "Reading ", TotalWords );
    if( TotalWords == 1 ) {
        DEBUG_STRING( " word" );
    } else {
        DEBUG_STRING( " words" );
    }
    DEBUG_HEX( " from SPI register 0x", startingAddress );
    DEBUG_INT( " (repeated ", TotalTests );
    if( TotalTests == 1 ) {
        DEBUG_STRING( " time)" );
    } else {
        DEBUG_STRING( " times)" );
    }

    DEBUG_INT( " at ", ODR_Hz );
    DEBUG_STRING( " Hz" );
    DEBUG_ENDLINE();
}



// Display the burst message
void _DisplayMesage_Stnd( uint16_t *RxWord )
{
    float    tempVal;       // convert from uint to float after the read
    uint8_t  signVal;
    int16_t  word;
    /// Display the UUT data from the SPI slave
    for( int LoopCounter = 1; LoopCounter < TotalWords+1; LoopCounter++ )
    {
        if( displayHex == 1 ) {
            DEBUG_HEX( " 0x", RxWord[ LoopCounter ] );
        } else {
            word = RxWord[ LoopCounter ];
            if( LoopCounter == 1 ) {
                /// Status word
                DEBUG_HEX( "", RxWord[ LoopCounter ] );
            } else if( LoopCounter <= 4 ) {
                /// RATE sensor conversion
                tempVal = 0.005 * (float)( (int16_t)RxWord[ LoopCounter ] );
                if( tempVal >= 0 ) {
                    DEBUG_STRING( " " ); // ' ' instead of '-' for positive data
                }
            
                /// Spaces to adjust the columns based on value (ones/tens/hundreds)
                signVal = abs( (int8_t)tempVal );
                if( signVal < 10 ) {
                    DEBUG_STRING( "  " );
                } else if( signVal < 100 ) {
                    DEBUG_STRING( " " );
                }
            
                /// Write the RATE data to the console
                DEBUG_FLOAT( "  ", tempVal, 4 );
            } else if ( LoopCounter <= 7 ) {
                /// ACCELeration conversion
                tempVal = 0.00025 * (float)( (int16_t)RxWord[ LoopCounter ] );
                if( tempVal >= 0 ) {
                    DEBUG_STRING( " " );
                }
                DEBUG_FLOAT( "   ", tempVal, 4 );
            } else {
                /// board TEMPERATURE - conversion
                DEBUG_FLOAT( "   ", 0.073111172849435 * (float)( (int16_t)RxWord[ LoopCounter ] ) + 31.0, 4 );
            }
        }
    }
    DEBUG_ENDLINE(); // Print a return character
}

// Display the burst message 2
void _DisplayMesage_Stnd2( uint16_t *RxWord )
{
    float    tempVal;       // convert from uint to float after the read
    uint8_t  signVal;

    /// Display the UUT data from the SPI slave
    for( int LoopCounter = 1; LoopCounter < TotalWords+1; LoopCounter++ )
    {
        if( displayHex == 1 ) {
            DEBUG_HEX( " 0x", RxWord[ LoopCounter ] );
        } else {
            if( LoopCounter == 1 ) {
                /// Status word
                DEBUG_HEX( "", RxWord[ LoopCounter ] );
            } else if( LoopCounter <= 4 ) {
                /// RATE sensor conversion
                tempVal = 0.005 * (float)( (int16_t)RxWord[ LoopCounter ] );
                if( tempVal >= 0 ) {
                    DEBUG_STRING( " " ); // ' ' instead of '-' for positive data
                }
            
                /// Spaces to adjust the columns based on value (ones/tens/hundreds)
                signVal = abs( (int8_t)tempVal );
                if( signVal < 10 ) {
                    DEBUG_STRING( "  " );
                } else if( signVal < 100 ) {
                    DEBUG_STRING( " " );
                }
            
                /// Write the RATE data to the console
                DEBUG_FLOAT( "  ", tempVal, 4 );
            } else if ( LoopCounter <= 7 ) {
                /// ACCELeration conversion
                tempVal = 0.00025 * (float)( (int16_t)RxWord[ LoopCounter ] );
                if( tempVal >= 0 ) {
                    DEBUG_STRING( " " );
                }
                DEBUG_FLOAT( "   ", tempVal, 4 );
            } else if ( LoopCounter < 9 ) {
                /// board TEMPERATURE - conversion
                DEBUG_FLOAT( "   ", 0.073111172849435 * (float)( (int16_t)RxWord[ LoopCounter ] ) + 31.0, 4 );
            }else {
                DEBUG_INT( "   ", (int16_t)RxWord[ LoopCounter ]);
            }
        }
    }
    DEBUG_ENDLINE(); // Print a return character
}



// Display the burst message
void _DisplayMesage_F1( uint16_t *RxWord )
{

uint8_t count;
uint32_t outputWord;
int16_t outputVal;

    /// Display the UUT data from the SPI slave
    for( int LoopCounter = 1; LoopCounter < TotalWords+1; LoopCounter++ )
    {
        if( displayHex == 1 ) {
            DEBUG_HEX( " 0x", RxWord[ LoopCounter ] );
        } else {
                if( LoopCounter == 0 ) {
                    count = 0;
                } else if( LoopCounter < TotalWords ) {
                    if( count == 0 ) {
                        outputWord = RxWord[ LoopCounter ] << 16;
                        count = 1;
                    } else if( count == 1 ) {
                        outputWord = outputWord | RxWord[ LoopCounter ];
                        count = 0;

                        if( LoopCounter < 13 ) {
                            // reading =
                            outputVal = (int16_t)( (int32_t)( outputWord >> 7 ) - 32768 );
                        } else if( LoopCounter < 25 ) {
                            // reading =
                            outputVal = (int16_t)( (int32_t)( outputWord >> 14 ) - 32768 );
                        } else {
                            // reading  = output - 2^15
                            outputVal = (int16_t)( (int32_t)( outputWord ) - 32768 );
                        }

                        // add a space if the value is positive
                        if( outputVal >= 0 ) {
                            DEBUG_STRING( " " );
                        }

                        // add spaces based on value
                        if( nabs( outputVal ) >= 1000 ) {
                            DEBUG_STRING( " " );
                        } else if ( nabs( outputVal ) >= 100 ) {
                            DEBUG_STRING( "  " );
                        } else if ( nabs( outputVal ) >= 10 ) {
                            DEBUG_STRING( "   " );
                        } else {
                            DEBUG_STRING( "    " );
                        }
                        // >= 10000
                        // >= 1000
                        // >= 100
                        // >= 10

                        DEBUG_INT( "   ", outputVal );
                    }
                } else {
                        DEBUG_HEX( "   ", RxWord[ LoopCounter ] );
                }
        }
    }
    DEBUG_ENDLINE(); // Print a return character
}





            


// Display the burst message
void _DisplayMesage_S0( uint16_t *RxWord )
{

double sf = 1.0;
double output_double = 0.0;

uint16_t value_uint;
int16_t value_int;

    /// Display the UUT data from the SPI slave
    for( int LoopCounter = 0; LoopCounter < TotalWords+1; LoopCounter++ ) {
        if( displayHex == 1 ) {
            DEBUG_HEX( " 0x", RxWord[ LoopCounter ] );
        } else {
            if( LoopCounter == 0 ) {
//                    count = 0;
            } else if( LoopCounter < TotalWords ) {
                if( LoopCounter == 1 ) {   // Accel
                    sf = 3.051757812500000e-04;
                }
                if( LoopCounter == 4 ) {   // Rate
                    sf = 0.019226074218750;
                }
                if( LoopCounter == 7 ) {   //Mag
                    sf = 3.051757812500000e-04;
                }
                if( LoopCounter == 10 ) {   // Temp
                    sf = 0.003051757812500;
                }
                if( LoopCounter == 13 ) {
                }
                if( LoopCounter == 14 ) {
                }
                if( LoopCounter == 15 ) {
                }

                value_uint = RxWord[ LoopCounter ];
                value_int  = (int16_t)( value_uint );

                if( LoopCounter < 14 ) {
                    output_double = sf * (double)value_int;
                    DEBUG_FLOAT( "  ", output_double, 4 );
                } else if( LoopCounter == 14 ) {
                    DEBUG_INT( "  ", value_uint )
                } else {
                    DEBUG_HEX( "  ", RxWord[ LoopCounter ] );
                }
            } else {
                    DEBUG_HEX( "  ", RxWord[ LoopCounter ] );
            }
        }
    }
    DEBUG_ENDLINE(); // Print a return character
}


// Display the burst message
void _DisplayMesage_S1( uint16_t *RxWord )
{

    // Define variables to convert the sensor readings to double values
    uint16_t value_uint;
    int16_t value_int;
    double output_double = 0.0;
    double sf = 1.0;
    
    static volatile uint16_t swStatusBit;

    /// Display the UUT data from the SPI slave
    for( int LoopCounter = 0; LoopCounter < TotalWords+1; LoopCounter++ ) {
        if( displayHex == 1 ) {
            DEBUG_HEX( " 0x", RxWord[ LoopCounter ] );
        } else {
            if( LoopCounter == 0 ) {
                ;
            } else if( LoopCounter < TotalWords ) {
                if( LoopCounter == 1 ) {   // Accel
                    sf = 3.051757812500000e-04;
                } else if( LoopCounter == 4 ) {   // Rate
                    sf = 0.019226074218750;
                } else if( LoopCounter == 7 ) {   // Rate Temp
                    sf = 0.003051757812500;
                } else if( LoopCounter == 10 ) {  // Board Temp
                    sf = 0.003051757812500;
                } else if( LoopCounter == 11 ) {
                    ;
                } else if( LoopCounter == 12 ) {
                    ;
                }
            
                value_uint = RxWord[ LoopCounter ];
                value_int  = (int16_t)( value_uint );
            
                if( LoopCounter <= 3 ) {  // accels
                    output_double = sf * (double)value_int;
            
                    // Add a space to align console output
                    if( output_double >= 0.0 ) {
                        DEBUG_STRING( " " );
                    }
            
                    DEBUG_FLOAT( "   ", output_double, 4 );
                } else if( LoopCounter <= 6 ) {  // rates
                    output_double = sf * (double)value_int;
            
                    // Add a space to account for the negative sign on negative values
                    if( output_double >= 0.0 ) {
                        DEBUG_STRING( " " );
                    }
            
                    // Add two spaces for values less than 10 and one space for values less
                    //   than 100
                    double tmp = fabs( output_double );
                    if( tmp < 10.0 ) {
                        DEBUG_STRING( "  " );
                    } else if( tmp < 100.0 ) {
                        DEBUG_STRING( " " );
                    }
            
                    DEBUG_FLOAT( "   ", output_double, 4 );
                } else if( LoopCounter < 11 ) {  // temps
                    output_double = sf * (double)value_int;
            
                    if( output_double >= 0.0 ) {
                        DEBUG_STRING( " " );
                    }
            
                    double tmp = fabs( output_double );
                    if( tmp < 10.0 ) {
                        DEBUG_STRING( "  " );
                    } else if( tmp < 100.0 ) {
                        DEBUG_STRING( " " );
                    }
            
                    DEBUG_FLOAT( "   ", output_double, 4 );
                } else if( LoopCounter == 11 ) {
                    DEBUG_INT( "   ", value_uint )
                } else {
                    DEBUG_HEX( "   ", RxWord[ LoopCounter ] );
                }
            } else {  // LoopCounter == 12
                value_uint = RxWord[ LoopCounter ];
                DEBUG_HEX( "   ", value_uint );
                  swStatusBit = value_uint & 0x0800;
            }
        }
    }
    DEBUG_ENDLINE(); // Print a return character
}


// Display the burst message
void _DisplayMesage_A1( uint16_t *RxWord )
{
    float    tempVal;       // convert from uint to float after the read
    uint8_t  signVal;

    /// Display the UUT data from the SPI slave
    for( int LoopCounter = 1; LoopCounter < TotalWords+1; LoopCounter++ )
    {
        if( displayHex == 1 ) {
            DEBUG_HEX( " 0x", RxWord[ LoopCounter ] );
        } else {
            if( LoopCounter == 1 ) {
                /// Status word
                DEBUG_HEX( "", RxWord[ LoopCounter ] );
            } else if( LoopCounter <= 4 ) {
                /// RATE sensor conversion
                tempVal = 0.005 * (float)( (int16_t)RxWord[ LoopCounter ] );
                if( tempVal >= 0 ) {
                    DEBUG_STRING( " " ); // ' ' instead of '-' for positive data
                }
            
                /// Spaces to adjust the columns based on value (ones/tens/hundreds)
                signVal = abs( (int8_t)tempVal );
                if( signVal < 10 ) {
                    DEBUG_STRING( "  " );
                } else if( signVal < 100 ) {
                    DEBUG_STRING( " " );
                }
            
                /// Write the RATE data to the console
                DEBUG_FLOAT( "  ", tempVal, 4 );
            } else if ( LoopCounter <= 7 ) {
                /// ACCELeration conversion
                tempVal = 0.00025 * (float)( (int16_t)RxWord[ LoopCounter ] );
                if( tempVal >= 0 ) {
                    DEBUG_STRING( " " );
                }
                DEBUG_FLOAT( "   ", tempVal, 4 );
            } else {
                /// board TEMPERATURE - conversion
                DEBUG_FLOAT( "   ", 0.073111172849435 * (float)( (int16_t)RxWord[ LoopCounter ] ) + 31.0, 4 );
            }
        }
    }
    DEBUG_ENDLINE(); // Print a return character
}


// Display the burst message
void _DisplayMesage_A2( uint16_t *RxWord )
{
    float    tempVal;       // convert from uint to float after the read
    uint8_t  signVal;

    /// Display the UUT data from the SPI slave
    for( int LoopCounter = 1; LoopCounter < TotalWords+1; LoopCounter++ )
    {
        if( displayHex == 1 ) {
            DEBUG_HEX( " 0x", RxWord[ LoopCounter ] );
        } else {
            if( LoopCounter == 1 ) {
                /// Status word
                DEBUG_HEX( "", RxWord[ LoopCounter ] );
            } else if( LoopCounter <= 4 ) {
                /// RATE sensor conversion
                tempVal = 0.005 * (float)( (int16_t)RxWord[ LoopCounter ] );
                if( tempVal >= 0 ) {
                    DEBUG_STRING( " " ); // ' ' instead of '-' for positive data
                }
            
                /// Spaces to adjust the columns based on value (ones/tens/hundreds)
                signVal = abs( (int8_t)tempVal );
                if( signVal < 10 ) {
                    DEBUG_STRING( "  " );
                } else if( signVal < 100 ) {
                    DEBUG_STRING( " " );
                }
            
                /// Write the RATE data to the console
                DEBUG_FLOAT( "  ", tempVal, 4 );
            } else if ( LoopCounter <= 7 ) {
                /// ACCELeration conversion
                tempVal = 0.00025 * (float)( (int16_t)RxWord[ LoopCounter ] );
                if( tempVal >= 0 ) {
                    DEBUG_STRING( " " );
                }
                DEBUG_FLOAT( "   ", tempVal, 4 );
            } else {
                /// board TEMPERATURE - conversion
                DEBUG_FLOAT( "   ", 0.073111172849435 * (float)( (int16_t)RxWord[ LoopCounter ] ) + 31.0, 4 );
            }
        }
    }
    DEBUG_ENDLINE(); // Print a return character
}


// Display the burst message
void _DisplayMesage_N1( uint16_t *RxWord )
{
    float    tempVal;       // convert from uint to float after the read
    uint8_t  signVal;

    /// Display the UUT data from the SPI slave
    for( int LoopCounter = 1; LoopCounter < TotalWords+1; LoopCounter++ )
    {
        if( displayHex == 1 ) {
            DEBUG_HEX( " 0x", RxWord[ LoopCounter ] );
        } else {
            if( LoopCounter == 1 ) {
                /// Status word
                DEBUG_HEX( "", RxWord[ LoopCounter ] );
            } else if( LoopCounter <= 4 ) {
                /// RATE sensor conversion
                tempVal = 0.005 * (float)( (int16_t)RxWord[ LoopCounter ] );
                if( tempVal >= 0 ) {
                    DEBUG_STRING( " " ); // ' ' instead of '-' for positive data
                }
            
                /// Spaces to adjust the columns based on value (ones/tens/hundreds)
                signVal = abs( (int8_t)tempVal );
                if( signVal < 10 ) {
                    DEBUG_STRING( "  " );
                } else if( signVal < 100 ) {
                    DEBUG_STRING( " " );
                }
            
                /// Write the RATE data to the console
                DEBUG_FLOAT( "  ", tempVal, 4 );
            } else if ( LoopCounter <= 7 ) {
                /// ACCELeration conversion
                tempVal = 0.00025 * (float)( (int16_t)RxWord[ LoopCounter ] );
                if( tempVal >= 0 ) {
                    DEBUG_STRING( " " );
                }
                DEBUG_FLOAT( "   ", tempVal, 4 );
            } else {
                /// board TEMPERATURE - conversion
                DEBUG_FLOAT( "   ", 0.073111172849435 * (float)( (int16_t)RxWord[ LoopCounter ] ) + 31.0, 4 );
            }
        }
    }
    DEBUG_ENDLINE(); // Print a return character
}


// Commands to set the output display (converted or hexadecimal values)
void CmdUserSpi_DisplayConverted( uint32_t data )
{
    displayHex = 0;
    DebugPrintString("\r\nDisplaying converted reading values\r\n"); ///< debug.c
    DEBUG_ENDLINE(); // Print a return character
}


void CmdUserSpi_DisplayHex( uint32_t data )
{
    displayHex = 1;
    DebugPrintString("\r\nDisplaying hexadecimal reading values\r\n"); ///< debug.c
    DEBUG_ENDLINE(); // Print a return character
}


/** ***************************************************************************
 
 ******************************************************************************/
void CmdUserSpi_HoldChipSelect( uint32_t data )
{
    ToogleCSFlag = HOLD_CS;
    DebugPrintString("\r\nSetting nSS to hold\r\n"); ///< debug.c
}

/** ***************************************************************************
 
 ******************************************************************************/
void CmdUserSpi_ToggleChipSelect( uint32_t data )
{
    ToogleCSFlag = TOGGLE_CS;
    DebugPrintString("\r\nSetting nSS to toggle\r\n"); ///< debug.c
}

/** ***************************************************************************
 
 ******************************************************************************/
void CmdUserSpi_SyncOn( uint32_t data )
{
    ToggleSyncPin = 1;
    DebugPrintString("\r\nStarting to toggle sync pin\r\n"); ///< debug.c
}

void CmdUserSpi_SyncOff( uint32_t data )
{
    ToggleSyncPin = 0;
    DebugPrintString("\r\nStop toggling sync pin\r\n"); ///< debug.c
}

void CmdUserSpi_FreqRatio( uint32_t data )
{
    char str[100];
    int freq;
    CmdLineGetArgInt( &freq);
    int readDelay = 2;
    
    sprintf(str, "\r\nFreq = %d\r\n", freq);
    freqRatio    = 10000 / freq;
    DebugPrintString(str); ///< debug.c
}

static int testNumber = 0;
int doSelfTest()
{
    static uint16_t RxBuffer[65];
    uint16_t reg52, reg34, reg3C;
    int runNumber, escapeCntr;
    
    /// Begin the self-test iterations
    for( runNumber = 0; runNumber < 1; runNumber++ )
    {
        /// read the product ID (like whoami) to verify that the unit is conected
        ReadFromRegisters( 0x56, 0x56, RxBuffer, 150, ToogleCSFlag ); /// product ID (0x3810) 150 ~10usec
 		reg52 = RxBuffer[1];

        // Proceed with self-test if the product ID is correct
		if ( reg52 == 0x3810 ) {
            /// Begin the self-test by writing 0x04 to register 0x35
            WriteToRegister( 0x35, 0x04 );

            // Perform the self-test up to 1500 times before escaping and declaring the self-test
            //   failed.
            escapeCntr = 0;
            do {
                // Wait 1 millisecond before checking for completion (readDelay can also be set to
                //   another value through the command line)
                waitMs();

                // Read the self-test status from register 0x34 (bit 10 is set high when self-test
                //   is executing).
                ReadFromRegisters( 0x34, 0x34, RxBuffer, 425, ToogleCSFlag ); // 425 ~ 27usec
                reg34 = RxBuffer[1];
                escapeCntr++;
            } while( ( reg34 & 0x0400 ) && ( escapeCntr < 100) );

            // After test completes, check the Self-test result register for test results
            ReadFromRegisters( 0x3C, 0x3C, RxBuffer, 425, ToogleCSFlag ); // 425 ~ 27usec
            reg3C = RxBuffer[1];

            waitMs();
                    /// read the product ID (like whoami) to verify that the unit is conected
            ReadFromRegisters( 0x56, 0x56, RxBuffer, 150, ToogleCSFlag ); /// product ID (0x3810) 150 ~10usec
            reg52 = RxBuffer[1];

            waitMs();

            // If more than 1500 milliseconds elapsed after the start of the self-test or the
            //   the result stored in register indicates a failed self-test then report a test
            //   failure.
            if( ( escapeCntr >= 100 ) || ( reg3C & 0x0020 ) ) {
                DEBUG_INT( "Run number ", runNumber+1 );
                DEBUG_HEX( ": Register 0x", 0x3C );
                DEBUG_HEX( " =  0x", reg3C );
                DEBUG_INT( " (escape cntr = ", escapeCntr );
                DEBUG_STRING( ")" );
                DEBUG_ENDLINE();
                return 0;
            } else { /// caution this will hit if UUT is not connected
                DEBUG_INT( "Run number ", testNumber++ );
                DEBUG_INT( ": self-test completed in ", escapeCntr );
                DEBUG_HEX( " cycles (diagnostic register at test completion, 0x", reg3C );
                DEBUG_STRING( ")");
                DEBUG_HEX( ". Data in register 0x56 = ", reg52);
                DEBUG_ENDLINE();
            }
        } 
    }
    return 1;
}
/*
void SPI_baddata_test1(){
    char str[100];
    static int numCycles = 100;
    static uint16_t RxBuffer[65];
    int testNumber = 0;
    int numErrors = 0;
    uint16_t reg52, reg34, reg3C;
    uint16_t data0;
    uint16_t data1;
    
    freqRatio = 10;
    testNumber = 0;
    
    CmdLineGetArgInt( &numCycles);
    if(numCycles == 0){
        numCycles = 2;
    }
    spi_go_slightly_faster( kUserCommunicationSPI ); ///< 1.0 Mhz
    sprintf(str, "\r\nNumCycles = %d\r\n", numCycles);
    DebugPrintString(str); ///< debug.c

    _InitDataReady( ENABLE );
    
    /// read the product ID (like whoami) to verify that the unit is conected
    ReadFromRegisters( 0x56, 0x56, RxBuffer, 150, ToogleCSFlag ); /// product ID (0x3810) 150 ~10usec
    reg52 = RxBuffer[1];

    // Proceed with self-test if the product ID is correct
    if ( reg52 == 0x3810 ) {
      DEBUG_STRING( "Talking to the correct Unit\r\n" );
    }
  
    /// Run Test Cycles
    for(int i = 0; i < numCycles; i++){
        waitMs();
        ToggleSyncPin = 0;
        GPIOE->BSRRH  = GPIO_Pin_2;                                     // Set E2 (nRst) low to reset 380
        DelayMs(1000);
        GPIOE->BSRRL = GPIO_Pin_2;                                      // Set E2 (nRst) high to release 380 from reset
        DelayMs(150);                                                   // delay from reset to clock
        ToggleSyncPin = 1;
        DelayMs(2000);                                                  // delay from sync to initalization
       
        /// Start Sequence
        ReadFromRegisters( 0x52, 0x52, RxBuffer, 200, TOGGLE_CS ); /// MANUF CODE 200 ~14usec
        ReadFromRegisters( 0x54, 0x54, RxBuffer, 200, TOGGLE_CS ); /// UNIT CODE 200 ~14usec
        ReadFromRegisters( 0x56, 0x56, RxBuffer, 200, TOGGLE_CS ); /// PRODUCT ID 200 ~14usec
        ReadFromRegisters( 0x7E, 0x7E, RxBuffer, 200, TOGGLE_CS ); /// HW_SW_VERSION 200 ~14usec
        ReadFromRegisters( 0x58, 0x58, RxBuffer, 200, TOGGLE_CS ); /// SERIAL NUMBER 200 ~14usec
        reg52 = RxBuffer[1];
        
        DelayMs(20);
        /// Second Sequence
        WriteToRegister( 0x36, 0x01 );                                  /// Set Data rate to 200Hz
        WriteToRegister( 0x35, 0x04 );                                  /// Init Self Test
        /// Read Self-test status 16 times (Emulating JD Trace)
        for(int j = 0; j < 16; j++){
          DelayMs(1);      // delay from reset to clock
          ReadFromRegisters( 0x34, 0x34, RxBuffer, 200, TOGGLE_CS );
          data0 = RxBuffer[1];
          DEBUG_INT( " ", data0);
        }
        /// Check for SPI unit stuc with 0x6565
        if ( data0 == 0x6565 ) {
          numErrors++;
        }
        
        /// Current Test Result
        DEBUG_INT( "TestNumber: ", i);
        DEBUG_INT( "Errors: ", numErrors);
        DebugPrintString("\r\n"); ///< debug.c
    }

    /// Wrapping up
    DEBUG_INT( "Self Test failed ", numErrors);
    DebugPrintString(" times\r\n"); ///< debug.c

    ToggleSyncPin = 0;

}

void SPI_baddata_test2(){
    char str[100];
    int testNumber = 0;
    int numErrors = 0;
    int selfTestFailure = 0;
    int timeout = 0;
    int badDataCount = 0;
    uint16_t reg52, reg34, reg3C;
    uint16_t data0;
    uint16_t data1;
    static int numCycles = 100;
    static uint16_t RxBuffer[65];
    
    freqRatio = 10;             // Set 1Khz External Clock Ratio
    testNumber = 0;
    
    CmdLineGetArgInt( &numCycles);
    if(numCycles == 0){
        numCycles = 2;
    }
    
    spi_go_slightly_faster( kUserCommunicationSPI ); ///< 1.0 Mhz
    sprintf(str, "\r\nNumCycles = %d\r\n", numCycles);
    DebugPrintString(str); ///< debug.c

    _InitDataReady( ENABLE );
    
    /// read the product ID (like whoami) to verify that the unit is conected
    ReadFromRegisters( 0x56, 0x56, RxBuffer, 150, ToogleCSFlag ); /// product ID (0x3810) 150 ~10usec
    reg52 = RxBuffer[1];

    // Proceed with self-test if the product ID is correct
    if ( reg52 == 0x3810 ) {
      DEBUG_STRING( "Talking to the correct Unit\r\n" );
    }
    
    /// Run Test Cycles
    for(int i = 0; i < numCycles; i++){
        timeout = 0;
        badDataCount = 0;
        data0 = 0;
        
        waitMs();
        ToggleSyncPin = 0;
        GPIOE->BSRRH  = GPIO_Pin_2;                                     // Set E2 (nRst) low to reset 380
        DelayMs(1000);
        GPIOE->BSRRL = GPIO_Pin_2;                                      // Set E2 (nRst) high to release 380 from reset
        DelayMs(150);                                                   // delay from reset to clock
        ToggleSyncPin = 1;
        DelayMs(2000);                                                  // delay from sync to initalization
        
        /// Start Sequence
        ReadFromRegisters( 0x52, 0x52, RxBuffer, 200, TOGGLE_CS ); /// MANUF CODE 200 ~14usec
        ReadFromRegisters( 0x54, 0x54, RxBuffer, 200, TOGGLE_CS ); /// UNIT CODE 200 ~14usec
        ReadFromRegisters( 0x56, 0x56, RxBuffer, 200, TOGGLE_CS ); /// PRODUCT ID 200 ~14usec
        ReadFromRegisters( 0x7E, 0x7E, RxBuffer, 200, TOGGLE_CS ); /// HW_SW_VERSION 200 ~14usec
        ReadFromRegisters( 0x58, 0x58, RxBuffer, 200, TOGGLE_CS ); /// SERIAL NUMBER 200 ~14usec
        reg52 = RxBuffer[1];
        
        DelayMs(20);
        /// Second Sequence
        
        WriteToRegister( 0x36, 0x01 );                                  /// Set Data rate to 200Hz
        for(int j = 0; j < 300; j++){;}                                 /// Delay ~15uS
        WriteToRegister( 0x35, 0x04 );                                  /// Init Self Test
        
        /// Wait for self test status and look for 0x6565 pattern
        while((data0 != 0x0004 && timeout < 200) && badDataCount < 50)
        {
          DelayMs(1);      // delay from reset to clock
          ReadFromRegisters( 0x34, 0x34, RxBuffer, 200, TOGGLE_CS );
          data0 = RxBuffer[1];
          if(data0 == 0x6565){
            badDataCount++;
          }
          timeout++;
          DEBUG_INT( " ", data0);
        }
        
        /// Increment Error by one if even one 0x6565 is received
        if ( badDataCount != 0 ) {
          numErrors++;
        }
        DelayMs(1);
        
        /// Check if there was a self test failure in the unit
        ReadFromRegisters( 0x3C, 0x3C, RxBuffer, 200, TOGGLE_CS );
        
        /// 0xFA30 is a self test bit mask
        if((RxBuffer[1] & 0xFA30) != 0){
          selfTestFailure++;
          DEBUG_INT( "Self Test failed ", RxBuffer[1]);
          DebugPrintString("\r\n"); ///< debug.c
        }
        
        /// Report Test result at the en of each test
        DEBUG_INT( "TestNumber: ", i);
        DEBUG_INT( " DataErrors: ", numErrors);
        DEBUG_INT( " SelftTestFails: ", selfTestFailure);
        DebugPrintString("\r\n"); ///< debug.c
    }
    
    /// Wrapping up
    DEBUG_INT( "Self Test failed ", numErrors);
    DebugPrintString(" times\r\n"); ///< debug.c

    ToggleSyncPin = 0;
}

void SPI_baddata_test3(){
    char str[100];
    int testNumber = 0;
    int numErrors = 0;
    int selfTestFailure = 0;
    int timeout = 0;
    int badDataCount = 0;
    uint16_t reg52, reg34, reg3C;
    uint16_t data0;
    uint16_t data1;
    static int numCycles = 100;
    static uint16_t RxBuffer[65];
    
    freqRatio = 10;             // Set 1Khz External Clock Ratio
    testNumber = 0;
    
    CmdLineGetArgInt( &numCycles);
    if(numCycles == 0){
        numCycles = 2;
    }
    
    spi_go_slightly_faster( kUserCommunicationSPI ); ///< 1.0 Mhz
    sprintf(str, "\r\nNumCycles = %d\r\n", numCycles);
    DebugPrintString(str); ///< debug.c

    _InitDataReady( ENABLE );
    
    /// read the product ID (like whoami) to verify that the unit is conected
    ReadFromRegisters( 0x56, 0x56, RxBuffer, 150, ToogleCSFlag ); /// product ID (0x3810) 150 ~10usec
    reg52 = RxBuffer[1];

    // Proceed with self-test if the product ID is correct
    if ( reg52 == 0x3810 ) {
      DEBUG_STRING( "Talking to the correct Unit\r\n" );
    }
    
    waitMs();
    ToggleSyncPin = 0;
    GPIOE->BSRRH  = GPIO_Pin_2;                                     // Set E2 (nRst) low to reset 380
    DelayMs(1000);
    GPIOE->BSRRL = GPIO_Pin_2;                                      // Set E2 (nRst) high to release 380 from reset
    DelayMs(150);                                                   // delay from reset to clock
    ToggleSyncPin = 1;
    DelayMs(2000);                                                  // delay from sync to initalization
    
    /// Run Test Cycles
    for(int i = 0; i < numCycles; i++){
        timeout = 0;
        badDataCount = 0;
        data0 = 0;
        
        /// Start Sequence
        ReadFromRegisters( 0x52, 0x52, RxBuffer, 200, TOGGLE_CS ); /// MANUF CODE 200 ~14usec
        ReadFromRegisters( 0x54, 0x54, RxBuffer, 200, TOGGLE_CS ); /// UNIT CODE 200 ~14usec
        ReadFromRegisters( 0x56, 0x56, RxBuffer, 200, TOGGLE_CS ); /// PRODUCT ID 200 ~14usec
        ReadFromRegisters( 0x7E, 0x7E, RxBuffer, 200, TOGGLE_CS ); /// HW_SW_VERSION 200 ~14usec
        ReadFromRegisters( 0x58, 0x58, RxBuffer, 200, TOGGLE_CS ); /// SERIAL NUMBER 200 ~14usec
        reg52 = RxBuffer[1];
        
        DelayMs(20);
        /// Second Sequence
        
        WriteToRegister( 0x36, 0x01 );                                  /// Set Data rate to 200Hz
        for(int j = 0; j < 300; j++){;}                                 /// Delay ~15uS
        WriteToRegister( 0x35, 0x04 );                                  /// Init Self Test
        
        /// Wait for self test status and look for 0x6565 pattern
        while((data0 != 0x0004 && timeout < 200) && badDataCount < 50)
        {
          DelayMs(1);      // delay from reset to clock
          ReadFromRegisters( 0x34, 0x34, RxBuffer, 200, TOGGLE_CS );
          data0 = RxBuffer[1];
          if(data0 == 0x6565){
            badDataCount++;
          }
          timeout++;
          DEBUG_INT( " ", data0);
        }
        
        /// Increment Error by one if even one 0x6565 is received
        if ( badDataCount != 0 ) {
          numErrors++;
        }
        DelayMs(1);
        
        /// Check if there was a self test failure in the unit
        ReadFromRegisters( 0x3C, 0x3C, RxBuffer, 200, TOGGLE_CS );
        
        /// 0xFA30 is a self test bit mask
        if((RxBuffer[1] & 0xFA30) != 0){
          selfTestFailure++;
          DEBUG_INT( "Self Test failed ", RxBuffer[1]);
          DebugPrintString("\r\n"); ///< debug.c
        }
        
        /// Report Test result at the en of each test
        DEBUG_INT( "TestNumber: ", i);
        DEBUG_INT( " DataErrors: ", numErrors);
        DEBUG_INT( " SelftTestFails: ", selfTestFailure);
        DebugPrintString("\r\n"); ///< debug.c
        
        DelayMs(20);
    }
    
    /// Wrapping up
    DEBUG_INT( "Self Test failed ", numErrors);
    DebugPrintString(" times\r\n"); ///< debug.c

    ToggleSyncPin = 0;
}


void SPI_baddata_test4(){
    char str[100];
    int numErrors = 0;
    int badDataCount = 0;
    uint16_t reg56 = 0;
    uint16_t data[5];
    uint16_t counter = 0;
    static int numCycles = 100;
    static uint16_t RxBuffer[65];
    
    freqRatio = 10;             // Set 1Khz External Clock Ratio
    testNumber = 0;
    
    CmdLineGetArgInt( &numCycles);
    if(numCycles == 0){
        numCycles = 2;
    }
    
    spi_go_slightly_faster( kUserCommunicationSPI ); ///< 1.0 Mhz
    sprintf(str, "\r\nNumCycles = %d\r\n", numCycles);
    DebugPrintString(str); ///< debug.c

    _InitDataReady( ENABLE );
    
    /// read the product ID (like whoami) to verify that the unit is conected
    ReadFromRegisters( 0x56, 0x56, RxBuffer, 150, ToogleCSFlag ); /// product ID (0x3810) 150 ~10usec
    reg56 = RxBuffer[1];

    // Proceed with self-test if the product ID is correct
    if ( reg56 == 0x3810 ) {
      DEBUG_STRING( "Talking to the correct Unit\r\n" );
    }
    
    waitMs();
    ToggleSyncPin = 0;
    GPIOE->BSRRH  = GPIO_Pin_2;                                     // Set E2 (nRst) low to reset 380
    DelayMs(1000);
    GPIOE->BSRRL = GPIO_Pin_2;                                      // Set E2 (nRst) high to release 380 from reset
    DelayMs(150);                                                   // delay from reset to clock
    ToggleSyncPin = 1;
    DelayMs(2000);                                                  // delay from sync to initalization
    
    /// Run Test Cycles
    for(int i = 0; i < numCycles; i++){
        badDataCount = 0;
        counter = 0;
        
        /// Start Sequence
        ReadFromRegisters( 0x52, 0x52, RxBuffer, 200, TOGGLE_CS ); /// MANUF CODE 200 ~14usec
        data[counter++] = RxBuffer[1];
        ReadFromRegisters( 0x54, 0x54, RxBuffer, 200, TOGGLE_CS ); /// UNIT CODE 200 ~14usec
        data[counter++] = RxBuffer[1];
        ReadFromRegisters( 0x56, 0x56, RxBuffer, 200, TOGGLE_CS ); /// PRODUCT ID 200 ~14usec
        data[counter++] = RxBuffer[1];
        ReadFromRegisters( 0x7E, 0x7E, RxBuffer, 200, TOGGLE_CS ); /// HW_SW_VERSION 200 ~14usec
        data[counter++] = RxBuffer[1];
        ReadFromRegisters( 0x58, 0x58, RxBuffer, 200, TOGGLE_CS ); /// SERIAL NUMBER 200 ~14usec
        data[counter++] = RxBuffer[1];
        
        for(int j = 0 ; j < 5; j++){
          switch(j){
            case 0:
              if(data[j] != 0x1808){
                DEBUG_INT( "BadDataReceived for command:", 0x5200);
                DEBUG_INT( " DataReceived:", data[j]);
                badDataCount++;
              }
              break; 
            case 1:
              if(data[j] != 0x0700){
                DEBUG_INT( "BadDataReceived for command:", 0x5400);
                DEBUG_INT( " DataReceived:", data[j]);
                badDataCount++;
              }
              break; 
            case 2:
              if(data[j] != 0x3810){
                DEBUG_INT( "BadDataReceived for command:", 0x5600);
                DEBUG_INT( " DataReceived:", data[j]);
                badDataCount++;
              }
              break; 
            case 3:
              if(data[j] != 0x0480){
                DEBUG_INT( "BadDataReceived for command:", 0x7E00);
                DEBUG_INT( " DataReceived:", data[j]);
                badDataCount++;
              }
              break; 
            case 4:
              if(data[j] != 0x0061){
                DEBUG_INT( "BadDataReceived for command:", 0x5800);
                DEBUG_INT( " DataReceived:", data[j]);
                badDataCount++;
              }
              break; 
            default:
            }
          data[j] = 0;
        }
        /// Increment Error by one if even one 0x6565 is received
        if ( badDataCount != 0 ) {
          numErrors++;
        }
        DelayMs(1);
        
        /// Report Test result at the en of each test
        DEBUG_INT( " TestNumber: ", i);
        DEBUG_INT( " DataErrors: ", numErrors);
        DebugPrintString("\r\n"); ///< debug.c
    }
    
    /// Wrapping up
    DEBUG_INT( "Self Test failed ", numErrors);
    DebugPrintString(" times\r\n"); ///< debug.c

    ToggleSyncPin = 0;
}
*/
static void enableSync(bool enable){
  if(enable){
    ToggleSyncPin = 1;
    DelayMs(2000);                                                  // delay from sync to initalization
  }
  else{
    waitMs();
    ToggleSyncPin = 0;
  }
}

static void restartUUT(){
  //enableSync(0);
  uint32_t OnemS_Count = 99996>>1;
  uint32_t TwomS_Count = 120000;
  pwmEnabled = 0;       // disable sync signal
  
  
  // ----- Reset the power to the 380 and drop the reset line -----
  // turn off power to the board, set nSS high, set nRst low
  DEBUG_STRING( "Remove power from the 380 and set nRst low" );
  DEBUG_ENDLINE();
  GPIOE->BSRRH = GPIO_Pin_0;  // Set E0 Low (remove power)
  SPI3_SLAVE_SELECT_PORT->BSRRL = SPI3_SLAVE_SELECT_PIN;
  GPIOE->BSRRH = GPIO_Pin_2;

  // Wait ...
  DelayMs( 2000 );

  // Power on board
  DEBUG_STRING( "Restore power to the 380 and release nRst 1 second after power applied" );
  DEBUG_ENDLINE();
  GPIOE->BSRRL = GPIO_Pin_0;  // Set E0 high

  // Set reset (E2) high after 100 msec
  DelayMs( 1000 );
  GPIOE->BSRRL = GPIO_Pin_2;

  // Wait 100 msec before talking to unit
  DEBUG_ENDLINE();
  DEBUG_STRING( "starting next test..." );
  DEBUG_ENDLINE();
  
  TIM5_UpdateTimerTop(OnemS_Count);
  pwmEnabled = 1; // enable sync signal
  pwmDivider = 2; // 500Hz sync signal
  DelayMs(2000);                                                  // delay from sync to initalization
  pwmDivider = 1; // 1KHz sync signal
}

static int burstReadSequence(int numCycles){
  int burstErrors = 0;
  /// (based on burst-read requirements)
  startingAddress = 0x3E; // burst mode
  numberOfBytes   = 16;
  endingAddress   = startingAddress + ( numberOfBytes - 1 );
  TotalWords = ( ( endingAddress + 1 ) - startingAddress )/2;
  
  for(int i = 0; i < numCycles; i++){ 
    burstErrors = 0;
    /// Wait here until the data-ready (DR) line goes low (set in the data-
    ///   ready interrupt handler)before proceeding
    if( WaitForDRDY ) {
      //DebugPrintString("Waiting for data ready\r\n"); ///< debug.c
      while( !DataReadyContinueFlag )
      {  /* Wait*/   }
    } else {
      DelayMs( 1 );  // user i pause between readings
    }
    //DebugPrintString("Done waiting for data ready\r\n"); 
    DataReadyContinueFlag = 0;
    BurstReadFromRegisters( startingAddress, endingAddress, RxWord, 100); // 25 usec
    //ReadFromRegisters( startingAddress, endingAddress, RxWord, 0, TOGGLE_CS );
    for(int i = 0 ; i < 16; i++){
      if(RxWord[i] == 0x6565)
        burstErrors++;
    }
    
  }
  return burstErrors;
}

static int selfTestSequence(){
  static uint16_t RxBuffer[65];
  int timeout = 0;
  int badDataCount = 0;
  int numErrors = 0;
  uint16_t data0 = 0;
  
  /// Second Sequence
  WriteToRegister( 0x36, 0x01 );                                  /// Set Data rate to 200Hz
  for(int j = 0; j < 300; j++){;}                                 /// Delay ~15uS
  WriteToRegister( 0x35, 0x04 );                                  /// Init Self Test
  
  /// Wait for self test status and look for 0x6565 pattern
  while((data0 != 0x0004 && timeout < 200) && badDataCount < 50)
  {
    DelayMs(1);
    ReadFromRegisters( 0x34, 0x34, RxBuffer, 200, TOGGLE_CS );
    data0 = RxBuffer[1];
    if(data0 == 0x6565){
      badDataCount++;
    }
    timeout++;
    DEBUG_INT( " ", data0);
  }

  /// Increment Error by one if even one 0x6565 is received
  if ( badDataCount != 0 ) {
    numErrors++;
  }
  DelayMs(1);

  /// Check if there was a self test failure in the unit
  ReadFromRegisters( 0x3C, 0x3C, RxBuffer, 200, TOGGLE_CS );

  /// 0xFA30 is a self test bit mask
  if((RxBuffer[1] & 0xFA30) != 0){
    DEBUG_INT( "Self Test failed ", RxBuffer[1]);
    DebugPrintString("\r\n"); ///< debug.c
  }
  return badDataCount;
}

static int startSequence(int numCycles){
  static uint16_t RxBuffer[64];
  int data[64];
  int numErrors = 0;
  int badDataCount = 0;
  int counter = 0;
  
  for(int i = 0; i < numCycles; i++){
    /// Start Sequence
    ReadFromRegisters( 0x52, 0x52, RxBuffer, 200, TOGGLE_CS ); /// MANUF CODE 200 ~14usec
    data[counter++] = RxBuffer[1];
    ReadFromRegisters( 0x54, 0x54, RxBuffer, 200, TOGGLE_CS ); /// UNIT CODE 200 ~14usec
    data[counter++] = RxBuffer[1];
    ReadFromRegisters( 0x56, 0x56, RxBuffer, 200, TOGGLE_CS ); /// PRODUCT ID 200 ~14usec
    data[counter++] = RxBuffer[1];
    ReadFromRegisters( 0x7E, 0x7E, RxBuffer, 200, TOGGLE_CS ); /// HW_SW_VERSION 200 ~14usec
    data[counter++] = RxBuffer[1];
    ReadFromRegisters( 0x58, 0x58, RxBuffer, 200, TOGGLE_CS ); /// SERIAL NUMBER 200 ~14usec
    data[counter++] = RxBuffer[1];

    for(int j = 0 ; j < 5; j++){
      switch(j){
        case 0:
          if(data[j] != 0x1808){
            DEBUG_INT( "BadDataReceived for MFCode:", 0x5200);
            DEBUG_INT( " DataReceived:", data[j]);
            badDataCount++;
          }
          break; 
        case 1:
          if(data[j] != 0x0700){
            DEBUG_INT( " BadDataReceived for UnitCode:", 0x5400);
            DEBUG_INT( " DataReceived:", data[j]);
            badDataCount++;
          }
          break; 
        case 2:
          if(data[j] != 0x3810){
            DEBUG_INT( " BadDataReceived for ProdID:", 0x5600);
            DEBUG_INT( " DataReceived:", data[j]);
            badDataCount++;
          }
          break; 
        case 3:
          if(data[j] != 0x0480){
            DEBUG_INT( " BadDataReceived for HWSWVer:", 0x7E00);
            DEBUG_INT( " DataReceived:", data[j]);
            badDataCount++;
          }
          break; 
        case 4:
          if(data[j] != 0x0061 && data[j] != 0x0092){
            DEBUG_INT( " BadDataReceived for SerialNum:", 0x5800);
            DEBUG_INT( " DataReceived:", data[j]);
            badDataCount++;
          }
          break; 
        default:
        }
      data[j] = 0;
    }
    /// Increment Error by one if even one 0x6565 is received
    if ( badDataCount != 0 ) {
      numErrors++;
    }
    DelayMs(1);
  }
  return numErrors;
}

static void writeSeqence(){
  WriteToRegister( 0xB6, 0x00 );                                  /// Set Data rate to 200Hz
  for(int j = 0; j < 300; j++){;}                                 /// Delay ~15uS
  WriteToRegister( 0xB7, 0x01 );                                  /// Init Self Test
  for(int j = 0; j < 300; j++){;}                                 /// Delay ~15uS
  WriteToRegister( 0xB8, 0x06 );                                  /// Init Self Test
  for(int j = 0; j < 300; j++){;}                                 /// Delay ~15uS
  WriteToRegister( 0xB9, 0x02 );                                  /// Init Self Test
  for(int j = 0; j < 300; j++){;}                                 /// Delay ~15uS
  WriteToRegister( 0xB2, 0x01 );                                  /// Init Self Test
  for(int j = 0; j < 300; j++){;}                                 /// Delay ~15uS
  WriteToRegister( 0xB4, 0x04 );                                  /// Init Self Test
  for(int j = 0; j < 300; j++){;}                                 /// Delay ~15uS
  WriteToRegister( 0xF4, 0x00 );                                  /// Init Self Test
  for(int j = 0; j < 300; j++){;}                                 /// Delay ~15uS
  WriteToRegister( 0xF5, 0x6B );                                  /// Init Self Test
}

void SPI_baddata_test5(){
    char str[100];
    int startSequenceErrors = 0;
    int selfTestErrors = 0;
    int badDataCount = 0;
    uint64_t burstErrors = 0;
    uint16_t reg56 = 0;
    uint16_t data[5];
    uint16_t counter = 0;
    static int numCycles = 100;
    static uint16_t RxBuffer[65];
    testNumber = 0;
    
    CmdLineGetArgInt( &numCycles);
    if(numCycles == 0){
        numCycles = 2;
    }
    
    spi_go_slightly_faster( kUserCommunicationSPI ); ///< 1.0 Mhz
    sprintf(str, "\r\nNumCycles = %d\r\n", numCycles);
    DebugPrintString(str); ///< debug.c

        /// Store original DRDY state and enable DRDY
    int drdyOriginalVal = WaitForDRDY;
    WaitForDRDY = 1;
    // Flag used to wait in place for the data-ready line before reading sensor
    //  data
    DataReadyContinueFlag = 0;
    _InitDataReady( ENABLE );
    
    /// read the product ID (like whoami) to verify that the unit is conected
    ReadFromRegisters( 0x56, 0x56, RxBuffer, 150, ToogleCSFlag ); /// product ID (0x3810) 150 ~10usec
    reg56 = RxBuffer[1];

    // Proceed with self-test if the product ID is correct
    if ( reg56 == 0x3810 ) {
      DEBUG_STRING( "Talking to the correct Unit\r\n" );
    }
    
    /// Run Test Cycles
    for(int i = 0; i < numCycles; i++){
      badDataCount = 0;
      counter = 0;
      
      //freqRatio = 20;             // Set 500hz External Clock Ratio
      restartUUT();
      //freqRatio = 10;             // Set 1Khz External Clock Ratio
      DelayMs(1000);
      writeSeqence();
      DelayMs(20);
      startSequenceErrors += startSequence(1);
      DelayMs(20);
      startSequenceErrors += startSequence(1);
      DelayMs(2);
      burstErrors += burstReadSequence(7477);
      DelayMs(20);
      startSequenceErrors += startSequence(1);
      DelayMs(20);
      selfTestErrors = selfTestSequence();

     
      /// Report Test result at the en of each test
      DEBUG_INT( " TestNumber: ", i);
      DEBUG_INT( " StartSequenceErrors: ", startSequenceErrors);
      DEBUG_INT( " selftestErrors: ", selfTestErrors);
      DEBUG_INT( " TotalburstErrors: ", burstErrors);
      DebugPrintString("\r\n"); ///< debug.c
    }
    
    /// Wrapping up
    DEBUG_INT( "Self Test failed ", selfTestErrors);
    DebugPrintString(" times\r\n"); ///< debug.c

    /// Restore Original DRDY state
    WaitForDRDY = drdyOriginalVal;
    ToggleSyncPin = 0;
    pwmEnabled = 0;
}

void oriRetention()
{
  static uint16_t RxBuffer[65];
  /// Read Current Orientation Setting
  ReadFromRegisters( 0x74, 0x75, RxBuffer, 200, TOGGLE_CS );
  DEBUG_INT( "Current Ori: ", RxBuffer[1]);
  DebugPrintString("\r\n"); ///< debug.c
  
  // Update Orientation
  WriteToRegister( 0xF4, 0x01 );                                  /// Set Data rate to Ori
  for(int j = 0; j < 300; j++){;}                                 /// Delay ~15uS
  
  WriteToRegister( 0xF5, 0x11 );                                  /// Set Data rate to Ori
  for(int j = 0; j < 300; j++){;}                                 /// Delay ~15uS
  
  /// confirm that orientation is updated
  ReadFromRegisters( 0x74, 0x75, RxBuffer, 200, TOGGLE_CS );
  DEBUG_INT( "New Ori: ", RxBuffer[1]);
  DebugPrintString("\r\n"); ///< debug.c
  
  // Save Ori to EEPROM
  WriteToRegister( 0xB6, 0x00 );                                  /// Set Data rate to Ori
  for(int j = 0; j < 300; j++){;}                                 /// Delay ~15uS
  DelayMs(2000);       // delay from sync to initalization
  
  //freqRatio = 20;             // Set 500hz External Clock Ratio
  restartUUT();
  DelayMs(2000); 
  
  /// confirm that orientation is updated
  ReadFromRegisters( 0x74, 0x75, RxBuffer, 200, TOGGLE_CS );
  DEBUG_INT( "New Ori: ", RxBuffer[1]);
  DebugPrintString("\r\n"); ///< debug.c
  
}


static uint16_t repeatSelfTestSequence(){
  static uint16_t RxBuffer[65];
  int timeout = 0;
  uint8_t timeoutError = 0;
  int badDataCount = 0;
  int numErrors = 0;
  uint16_t data0 = 0;
  
  /// Second Sequence
  WriteToRegister( 0x36, 0x01 );                                  /// Set Data rate to 200Hz
  for(int j = 0; j < 300; j++){;}                                 /// Delay ~15uS
  WriteToRegister( 0x35, 0x04 );                                  /// Init Self Test
  tTime startTime =  TimeNow();
  
  /// Wait for self test status and look for 0x6565 pattern
  while(data0 != 0x0004 && timeout < 200)
  {
    DelayMs(1);
    ReadFromRegisters( 0x34, 0x34, RxBuffer, 200, TOGGLE_CS );
    data0 = RxBuffer[1];
    if(data0 == 0x6565){
      badDataCount++;
    }
    timeout = TimePassed(startTime);
    DEBUG_INT( " ", data0);
  }
  
  DelayMs(1);
  tTime endTime = TimePassed(startTime);
  DEBUG_INT( "\r\SelfTestTime: ", endTime);
  
  // SelfTest tool longer than half second
  if(endTime > 500){
    timeoutError = 1;
  }
  
  /// Increment Error by one if even one 0x6565 is received
  if ( badDataCount != 0 ) {
    numErrors++;
  }
  DelayMs(1);

  /// Check if there was a self test failure in the unit
  ReadFromRegisters( 0x3C, 0x3C, RxBuffer, 200, TOGGLE_CS );

  /// 0xFA30 is a self test bit mask
  if((RxBuffer[1] & 0xFA30) != 0){
    DEBUG_INT( "Self Test failed ", RxBuffer[1]);
    DebugPrintString("\r\n"); ///< debug.c
  }
  return ((timeoutError << 8)| numErrors);
}

void repeatSelfTest()
{

char str[100];
    int startSequenceErrors = 0;
    uint16_t selfTestErrors = 0;
    uint32_t selfTestTimeOuts = 0;
    int badDataCount = 0;
    uint64_t burstErrors = 0;
    uint16_t reg56 = 0;
    uint16_t data[5];
    uint16_t counter = 0;
    static int numCycles = 100;
    static uint16_t RxBuffer[65];
    testNumber = 0;
    
    CmdLineGetArgInt( &numCycles);
    if(numCycles == 0){
        numCycles = 2;
    }
    
    spi_go_slightly_faster( kUserCommunicationSPI ); ///< 1.0 Mhz
    sprintf(str, "\r\nNumCycles = %d\r\n", numCycles);
    DebugPrintString(str); ///< debug.c

        /// Store original DRDY state and enable DRDY
    int drdyOriginalVal = WaitForDRDY;
    WaitForDRDY = 1;
    // Flag used to wait in place for the data-ready line before reading sensor
    //  data
    DataReadyContinueFlag = 0;
    _InitDataReady( ENABLE );
    
    /// read the product ID (like whoami) to verify that the unit is conected
    ReadFromRegisters( 0x56, 0x56, RxBuffer, 150, ToogleCSFlag ); /// product ID (0x3810) 150 ~10usec
    reg56 = RxBuffer[1];

    // Proceed with self-test if the product ID is correct
    if ( reg56 == 0x3810 ) {
      DEBUG_STRING( "Talking to the correct Unit\r\n" );
    }
    
    /// Run Test Cycles
    for(int i = 0; i < numCycles; i++){
      badDataCount = 0;
      counter = 0;
      restartUUT();
      DelayMs(1000);
      for(int i = 0; i < 100; i++){
        DelayMs(100);
        uint16_t result = repeatSelfTestSequence();
        selfTestErrors += (result & 0xFF);
        selfTestTimeOuts += (result>>8);
      }
      DelayMs(100);
      /// Report Test result at the en of each test
      DEBUG_INT( " TestNumber: ", i);
      DEBUG_INT( " selftestErrors: ", selfTestErrors);
      DEBUG_INT( " selftestTimeOuts: ", selfTestTimeOuts);
      DebugPrintString("\r\n"); ///< debug.c
    }
    
    /// Wrapping up
    DEBUG_INT( "Self Test failed ", selfTestErrors);
    DebugPrintString(" times\r\n"); ///< debug.c
    DEBUG_INT( "Selftest Timeed Out: ", selfTestTimeOuts);
    DebugPrintString(" times\r\n"); ///< debug.c
    
    /// Restore Original DRDY state
    WaitForDRDY = drdyOriginalVal;
    ToggleSyncPin = 0;
    pwmEnabled = 0;
}

void CmdUserSpi_CycleTest( uint32_t data )
{
    char str[100];
    static int numCycles = 100;
    int testNumber = 0;
    int numErrors = 0;
    
    
    CmdLineGetArgInt( &numCycles);
    if(numCycles == 0){
        numCycles = 100;
    }
    
    sprintf(str, "\r\nNumCycles = %d\r\n", numCycles);
    DebugPrintString(str); ///< debug.c

    _InitDataReady( ENABLE );
    
    testNumber = 0;

    for(int i = 0; i < numCycles; i++){
        waitMs();
        freqRatio = 50;
//        GPIOE->BSRRH = GPIO_Pin_0;   // Set E0 low to remove power from 380 (if power switch is connected)
        ToggleSyncPin = 0;
        GPIOE->BSRRH  = GPIO_Pin_2;   // Set E2 (nRst) low to reset 380
        DelayMs(1000);
//        GPIOE->BSRRL = GPIO_Pin_0;   // Set E0 high to apply power
        GPIOE->BSRRL = GPIO_Pin_2;   // Set E2 (nRst) high to release 380 from reset
        DelayMs(150);      // delay from reset to clock
        ToggleSyncPin = 1;
        DelayMs(2000);       // delay from sync to initalization
        freqRatio = 10;
        DelayMs(1000); 
        WriteToRegister(0x36, 0x01 );
        int res = doSelfTest();
        if(res == 0){
            DebugPrintString("Self Test failed\r\n"); ///< debug.c
            numErrors++;
        }
        testNumber++;
    }

    DEBUG_INT( "Self Test failed ", numErrors);
    DebugPrintString(" times\r\n"); ///< debug.c

    ToggleSyncPin = 0;

}

void CmdUserSpi_CyclePower( uint32_t data )
{
    char str[100];
    static int numCycles = 100;
    int testNumber = 0;
    
    
    CmdLineGetArgInt( &numCycles);
    if(numCycles == 0){
        numCycles = 2;
    }
    
    sprintf(str, "\r\nNumCycles = %d\r\n", numCycles);
    DebugPrintString(str); ///< debug.c

    _InitDataReady( ENABLE );
    
//    if(!ToggleSyncPin){
//        ToggleSyncPin = 1;
//    }
    testNumber = 0;

    for(int i = 0; i < numCycles; i++){
        freqRatio = 20;
        GPIOE->BSRRH = GPIO_Pin_0;   // Set E0 low to remove power from 380 (if power switch is connected)
        ToggleSyncPin = 0;
//        GPIOE->BSRRH  = GPIO_Pin_2;   // Set E2 (nRst) low to reset 380
        DelayMs(5000);
        GPIOE->BSRRL = GPIO_Pin_0;   // Set E0 high to apply power
//        GPIOE->BSRRL = GPIO_Pin_2;   // Set E2 (nRst) high to release 380 from reset
        DataReadyContinueFlag = 0;
        DelayMs(150);      // delay from reset to clock
        ToggleSyncPin = 1;
        DelayMs(2000);       // delay from sync to initalization
        DataReadyContinueFlag = 0;
//        ToggleSyncPin = 1;
        uint32_t  startTime = GetSysTime(); 
        uint32_t  stopTime;
        int drdyCount = 0;
        while(1){
            if(DataReadyContinueFlag){
                DataReadyContinueFlag = 0;
                drdyCount++;
            }
            stopTime = GetSysTime();
            if(stopTime - startTime >= 21){
                break;
            }  // 20 ms
        }
        
        if(drdyCount < 2){
            DEBUG_INT( "First Test Data Ready signal Corrupted ", drdyCount);
            DebugPrintString("\r\n"); ///< debug.c
            break;
        }
/*
        DataReadyContinueFlag = 0;
        int res = doSelfTest();
        if(res == 0){
            DebugPrintString("Self Test failed\r\n"); ///< debug.c
            if(!DataReadyContinueFlag){
                DebugPrintString("Data Ready signal missing\r\n"); ///< debug.c
            }
            break;
        }
*/
        freqRatio = 10;
        DelayMs(1000); 
        DataReadyContinueFlag = 0;
        startTime = GetSysTime(); 
        drdyCount = 0;
        while(1){
            if(DataReadyContinueFlag){
                DataReadyContinueFlag = 0;
                drdyCount++;
            }
            stopTime = GetSysTime();
            if(stopTime - startTime >= 21){
                break;
            }  // 20 ms
        }
        if(drdyCount < 4){
            DEBUG_INT( "Second Test Data Ready signal Corrupted ", drdyCount);
            DebugPrintString("\r\n"); ///< debug.c
            break;
        }
        DEBUG_INT( "Run number ", ++testNumber);
        DebugPrintString("\r\n"); ///< debug.c
    }

    ToggleSyncPin = 1;

}





/** ***************************************************************************
 * @name CmdSpeed() set the user spi3 bus speed (baudrate) LOCAL
 * @brief   display returned values
 * "speed value"
 * @param [in] speedIndex - the enumeration for the speed selectd
 * @retval N/A
 ******************************************************************************/
void CmdSpeed( uint32_t data )
{
    //
    float SpiClockSpeed = 2.0;
    CmdLineGetArgFloat( &SpiClockSpeed );
    
    DebugPrintFloat( "SPI Clock Speed: ", SpiClockSpeed, 3 );
    DEBUG_ENDLINE();

    if( SpiClockSpeed < 0.2 ) {
        spi_go_really_slow( kUserCommunicationSPI );   ///< 0.125 Mhz
        DEBUG_STRING( "Really slow SPI clock selected\r\n");
    } else if( SpiClockSpeed < 0.3 ) {
        spi_go_slow( kUserCommunicationSPI );          ///< 0.25 Mhz
        DEBUG_STRING( "Slow SPI clock selected\r\n");
    } else if( SpiClockSpeed < 0.6 ) {
        spi_go_faster( kUserCommunicationSPI );        ///< 0.5 Mhz
        DEBUG_STRING( "Faster SPI clock selected\r\n");
    } else if( SpiClockSpeed < 1.1 ) {
        spi_go_slightly_faster( kUserCommunicationSPI ); ///< 1.0 Mhz
        DEBUG_STRING( "Slighlty faster SPI clock selected\r\n");
    } else if( SpiClockSpeed < 2.1 ) {
        spi_go_even_faster( kUserCommunicationSPI );   ///< 2.0 Mhz
        DEBUG_STRING( "Even faster SPI clock selected\r\n");
    } else if( SpiClockSpeed < 4.1 ) {
        spi_go_fast( kUserCommunicationSPI );          ///< 4.0 Mhz
        DEBUG_STRING( "Fast SPI clock selected\r\n");
    }
}


// Set the SPI master clock rate based on the argument to this function
void _SetSPIClockSpeed( uint8_t clockSpeed )
{
    if( clockSpeed == 0 ) {
        spi_go_slow( SPI3 ); ///< 0.25 MHz
    } else if( clockSpeed == 1 ) {
        spi_go_faster( SPI3 ); ///< 0.5 MHz
    } else if( clockSpeed == 2 ) {
        spi_go_slightly_faster( SPI3 ); ///< 1.0 MHz
    } else if( clockSpeed == 3 ) {
        spi_go_even_faster( SPI3 );   /// 2 MHz
    } else {
        spi_go_slightly_faster( SPI3 ); ///< 1.0 MHz
    }
}



void CmdUserSpi_WaitForDRDY( uint32_t data )
{
    WaitForDRDY = 1;
}

int imageLength = 0;

void CmdUserSpi_DontWaitForDRDY( uint32_t data )
{
    WaitForDRDY = 0;
}

BOOL validateApplicationImage(uint32_t *startAddr, uint32_t *length)
{
    uint32_t size;
    uint32_t *addr = (uint32_t *)0x08010000;
    if(*addr == 0xffffffff){
        return FALSE;
    }
    
    addr = (uint32_t*)0x08010184;
    if(*addr == 0xffffffff){
        return FALSE;
    }

    addr = (uint32_t*)0x0803FFF0;
    
    for(int i = 0; i < 0x00020000/4; i++){
       if(
            addr[0] == 0x01234567 &&
            addr[1] == 0x89abcdef &&
            addr[2] == 0xaa55aa55 &&
            addr[3] == 0x55aa55aa){
            imageLength = (uint32_t)(addr + 4) - 0x08010000;
            *startAddr = 0x08010000;
            return TRUE;
       }
       addr--;
    }
    return FALSE;
}

BOOL sendEnterBootModeCommand()
{
    uint16_t res;
    SendCommand(0x7a, 0x01);
    DelayMs(400);
    res = ReadFromRegister(0x78);
    DelayMs(10);
    if(res != 0x46AB){
        return FALSE;
    }
    return TRUE;
}

/*
BOOL sendEraseCommand()
{
    uint16_t res;
    WriteToRegister(0x7a, 0x02);
    DelayMs(2000);
    res = ReadFromRegister(0x78);
    DelayMs(10);
    if(res == 0x402B){
        return TRUE;
    }
    // reset status register
    WriteToRegister(0x7a, 0x03);
    DelayMs(10);

    return FALSE;
}
*/

BOOL sendResetStatusCommand()
{
    uint16_t res;
    SendCommand(0x7a, 0x03);
    DelayMs(10);
    res = ReadFromRegister(0x78);
    DelayMs(10);
    if(res == 0x46AB){
        return TRUE;
    }
    return FALSE;
}


BOOL activateApplication()
{
    uint16_t res;
    SendCommand(0x7a, 0x05);
    DelayMs(1000);
    res = ReadFromRegister(0x78);
    DelayMs(10);
    if(res == 0x46AB){
        return TRUE;
    }
    // reset status register
//    WriteToRegister(0x7a, 0x03);
//    DelayMs(10);

    return FALSE;
}

BOOL programBlock(int segment)
{
    uint16_t res;
    SendCommand(0x7a, 0x02);
    if(segment <= 16 ){ // first block - include erase
        DelayMs(4000);
    }else{
        DelayMs(100);
    }
    res = ReadFromRegister(0x78);
    DelayMs(10);

    if(res == 0x46AB){
        return TRUE;
    }

    return FALSE;
}


BOOL resetDevice()
{
    uint16_t res;
    SendCommand(0x7a, 0x06);
    DelayMs(1000);
    res = ReadFromRegister(0x78);
    DelayMs(10);
    if(res == 0x06ab){
        return TRUE;
    }
    return FALSE;
}



uint8_t fragment[256];

BOOL programApplicationImage(uint32_t startAddr, uint32_t length)
{
    
    int num, blockNum = 0, res;
    uint16_t status;
    uint8_t *txBuffer = (uint8_t*)startAddr;
    int length1;
    
// load first 4K of image
    while(length > 0){
        if(length > 4096){
            length1 = 4096;
        }else {
            length1 = length;
        }
        length -= length1;
        while(length1 > 0){
            if(length1 < 256){
                num = length1;
            }else{
                num = 256;
            }
            length1 -= num;
            memset(fragment, 0xff, sizeof(fragment));
            memcpy(fragment, txBuffer, num);
            status = SendBlock(0x7c, blockNum, fragment, 256);
            DelayMs(10);
            res = ReadFromRegister(0x78);
            DelayMs(10);
            if(res != 0x46AB){
                return FALSE;
            }
            DEBUG_STRING( ".");
            txBuffer += num;
            blockNum += 1;
        }
        // here we have first block sent to the device - and status seems OK. So - issue program command     
        if(!programBlock(blockNum)){
            return FALSE;
        }
    }
    return TRUE;
}

void CmdUserSpi_BootStatus( uint32_t data )
{
    int res = ReadFromRegister(0x78);
    DEBUG_HEX( "Boot Status: 0x", res);
    DEBUG_STRING( "\r\n");
}

void CmdUserSpi_BootReset( uint32_t data )
{
    resetDevice();
}


void CmdUserSpi_BootLoad( uint32_t data )
{
    uint32_t startAddr;
    uint32_t length;
    BOOL res = validateApplicationImage(&startAddr, &length);
    
    if(res != TRUE){
        DEBUG_STRING( "Application image invalid or missing\r\n");
        return;
    }
    res = sendEnterBootModeCommand();
    if(res != TRUE){
        DEBUG_STRING( "Enter Boot Mode command failed\r\n");
        return;
    }
    res = programApplicationImage(startAddr, imageLength);
    if(res != TRUE){
        DEBUG_STRING( "Programming Failed\r\n");
        return;
    }
    res = activateApplication();
    if(res != TRUE){
        DEBUG_STRING( "Activate application Failed\r\n");
        return;
    }
    res = resetDevice();
//    if(res != TRUE){
//        DEBUG_STRING( "Application failed to restart\r\n");
//        return;
//    }

    DEBUG_STRING( "\r\nApplication programming succeded\r\n");
}


