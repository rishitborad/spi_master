/******************************************************************************
* File name:  main.c
*
* File description:  main is the center of the system universe,
* it all starts here. And never ends.
*
******************************************************************************/

/* Includes */
#include <stddef.h>

#include "boardDefinition.h"
#include "salvodefs.h"
#include "stm32f2xx.h"
#include "bsp.h"
#include "dmu.h"

#include "debug_usart.h"
#include "debug.h"
#include "commandLine.h"

#include "UserCommunication_SPI.h"

#include "spi.h"

#include "ConfigureGPIOPins.h"
#include "SyncTimer.h"

#define GPIO_INPUT  0
#define GPIO_OUTPUT 1

void TaskHeartbeat( void );
void TaskCommandLine( void );

// Timer control and informational variables
extern uint8_t ToggleSyncPin;
extern uint16_t freqRatio;
extern uint32_t ElapsedTime_sec, ElapsedTime_usec;

/** ***************************************************************************
 * @name    main()
 * @brief   entrypoint to the DMU380 SPI master testing application. Initilizes
 * the BSP, starts execution tasks and the main loop
 *
 * This version sets up the SPI bus as a master to test the bus funtionality
 * on standard production units under test.
 *
 * @param N/A
 * @retval exit status.
 *
 ************************

******************************************************/

int main(void)
{
    uint32_t i;
 
    uint16_t TimerFreq_Hz, SyncFreq_Hz;

    static uint32_t count = 0;
    static uint8_t JD_Init_Seq = 1;
    static uint8_t UseNRstPin = 1;

    static uint8_t BeginCmdSeq = 1;

    /// 1. Initialize the system clock, PLL, etc
    SystemInit();            ///< system_stm32f2xx.c
    SystemCoreClockUpdate(); ///< system_stm32f2xx.c
    BSP_init();              ///< bsp.c

    /// 2. Initialize the USART (serial debug) port
    InitDebugSerialCommunication(); ///< debug_usart.c
    DebugPrintString("\r\nDMU380 system SPI MASTER v1.300.5.RB\r\n"); ///< debug.c

    /// 3. Set up the GPIO pins
    /// a) Initialize USARTB Rx (pin D2) and SPI3 nSS (pin A15) as INPUT pins (the proper setting of
    ///    these pins are performed later)
    InitGPIOPin_SharedCommunication();

    /// b) Set up the Data-Ready SPI3 pin (B3: 380, C13: IAR) as INPUT
    InitGPIOPin_DataReady();

    /// c) Set up other pins (power line control: E0 & nRst: E2)
    InitGPIOPin_Ex();
    GPIOE->BSRRH = GPIO_Pin_0;   // Set E0 low to remove power from 380 (if power switch is connected)
    GPIOE->BSRRL = GPIO_Pin_2;   // Set E2 (nRst) high to release 380 from reset

    /// d) Initialize the sync line (A0) as OUTPUT. Set the line high.
    InitGPIOPin_Sync();
    ONE_PPS_PORT->BSRRL = ONE_PPS_PIN;

    /// 4. Set up the RTOS’s data structures, pointers, and counters
    OSInit(); ///< salvofpt.h

    /// SPI is in main task loop
    //   Col. 1: callback name, Col. 2: task control block, Col. 3: task priority
    ///< salvofpt.h
    OSCreateTask( TaskCommandLine, COMMAND_LINE_TASK, COMMAND_LINE_PRIORITY );
    OSCreateTask( TaskHeartbeat,   HEARTBEAT_TASK,    HEARTBEAT_PRIORITY   );

    /// Signaling a waiting task is done via the RTOS function, OSSignalBinSem();
    ///   semsphore reset upon call of WaitBinSem().
    OSCreateBinSem( BINSEM_SERIAL_RX, 0 );

    /// 5. Set the timer and sync frequencies (a timer freq of 10 kHz can provide a dt of 50 usec
    ///    for scheduling purposes).  Do not enable toggling of the sync line at start (this is done
    ///    by the test script).
    ToggleSyncPin = 0;
    TimerFreq_Hz = 10000;
    SyncFreq_Hz  = 500;
    
    ///RB: Default
    //TimerFreq_Hz = 10000;
    //SyncFreq_Hz  = 1000;
    freqRatio    = TimerFreq_Hz / SyncFreq_Hz;
    
    /// Create a timer interrupt with a specified frequency (the frequency corresponds to the
    ///   frequency of a square wave -- a complete transition from low to high and back)
    InitTimer_TIM5( TimerFreq_Hz );

    // Reset timer variables (with a timer frequency of 10 kHz, the usec counter increments in
    //   50 usec increments; a 5 kHz timer will cause the usec counter to increment in 100 usec
    //   increments, etc)
    ElapsedTime_sec  = 0;
    ElapsedTime_usec = 0;

    /// Select UART or SPI (if DR pin is pulled LOW then select UART, else SPI).
//    if( GPIO_ReadInputDataBit( DATA_READY_PORT, DATA_READY_PIN ) == GPIO_INPUT ) {
//        // Declare and initialize the GPIO Structure
//        GPIO_InitTypeDef GPIO_InitStructure; // stm32fxx_gpio.h
//        GPIO_StructInit( &GPIO_InitStructure );
//
//        // Enable GPIO clocks
//        RCC_AHB1PeriphClockCmd( SPI3_GPIO_MOSI_CLK, ENABLE );
//        RCC_AHB1PeriphClockCmd( SPI3_GPIO_MISO_CLK, ENABLE );
//        RCC_AHB1PeriphClockCmd( SPI3_GPIO_SCK_CLK,  ENABLE );
//        RCC_AHB1PeriphClockCmd( SPI3_SLAVE_SELECT_CLK, ENABLE );
//
//        // Initialize USER-DRDY (B3: 380, C13: IAR) pin as an INPUT pin
//        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;     /// input/output/alt func/analog
//        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    /// push-pull or open-drain
//        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;     /// Up/Down/NoPull
//        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; /// low/med/fast/high speed
//
//        // Specify the data-ready/UART selection pin (from boardDefinition.h)
//        GPIO_InitStructure.GPIO_Pin = SPI3_SCK_PIN;
//
//        // Initialize the DR pin
//        GPIO_Init( SPI3_SCK_PORT, &GPIO_InitStructure);
//
//        GPIO_InitStructure.GPIO_Pin = SPI3_MOSI_PIN; // MOSI
//        GPIO_Init( SPI3_MOSI_PORT, &GPIO_InitStructure );
//        GPIO_InitStructure.GPIO_Pin = SPI3_MISO_PIN; // MISO
//        GPIO_Init( SPI3_MISO_PORT, &GPIO_InitStructure );
//
//        GPIO_InitStructure.GPIO_Pin  = SPI3_SLAVE_SELECT_PIN; // nSS
//        GPIO_Init( SPI3_SLAVE_SELECT_PORT, &GPIO_InitStructure );
//    } else { /// SPI
        /// 6. Configure SPI pins. Set SPI clock to 1 MHz.
        InitCommunication_UserSPI();
        spi_go_slightly_faster( SPI3 );
//    }

    // 7. Apply power to the unit (pause before and after power is applied)
    DelayMs( 2500 );
    GPIOE->BSRRL = GPIO_Pin_0;  // Set E0 high
    DelayMs( 500 );

    // 8. Run the tasks...
    //
    // Spooling data: the spool at the start of the program is due to the debug
    //   statements in TaskDataAcquisition. May want to stop this in order to
    //   debug why the rate-sensor doesn't seem to initialize properly.
    while (1) {
        OSSched();
    }
}


/** ***************************************************************************
 * @name    TaskHeartbeat()
 * @brief   task callback main loop to peridoicly turnthe a led on and off to
 * indicate the software is running
 *
 * @param N/A
 * @retval N/A.
 ******************************************************************************/
void TaskHeartbeat(void)
{
    // Set the toggling frequency with the divisor of OS_TICKS_PER_SECOND
    //   fToggle = divisior/2 (i.e. divisor = 2 --> 1 Hz, divisor == 8 --> 4 Hz)

    static uint8_t ledState = 0;
    static uint8_t shiftValue = 1;   // 1 Hz rate

	while( 1 )
	{
        // Toggle the LED
        if( ledState ) {
            ledState = 0;
		led_off(HEARTBEAT_LED);
        } else {
            ledState = 1;
            led_on(HEARTBEAT_LED);
        }

        // Wait
        OS_Delay( OS_TICKS_PER_SECOND >> shiftValue );
    }
}


/** ***************************************************************************
 * @name    TaskCommandLine()
 * @brief   task callback main loop to handle the serial communication to
 * command SPI messages or tests.
 *
 * When the USART interrupt handler indicates a RX on the USART line, toggle the
 * led to show the task is alive and compare the received message to what is in
 * the commandTable
 *
 * @param N/A
 * @retval N/A.
 ******************************************************************************/
void TaskCommandLine(void)
{
    /// Place a prompt at the start of the command line
    InitCommandLine(); ///< commandline.c

    while (1) {
        /// Wait for the semaphore to change state: occurs in the USART interrupt
        OS_WaitBinSem( BINSEM_SERIAL_RX, OSNO_TIMEOUT );
        led_toggle( LED2 ); // bsp.c
        CmdLineLookup( gCommands ); // commandLine.c
    }
}

