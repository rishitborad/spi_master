#include "ConfigureGPIOPins.h"

#include "boardDefinition.h"
#include "stm32f2xx.h"

/** ***************************************************************************
 * @name    _InitGPIOPin_DataReady()
 * @brief   Initialize the Data-Ready/UART-select GPIO pin
 *
 * Note: the pin will be pulled low at start-up to select UART communications
 *       (Nav-View) or left floating to select the SPI communications (internal
 *       pull-up will pull pin high).  Once detected, the pin will be configured
 *       as a data-ready pin if SPI is selected.
 *
 * @param [in] N/A
 * @retval N/A.
 ******************************************************************************/
void InitGPIOPin_DataReady(void)
{
    // Declare and initialize the GPIO Structure
    GPIO_InitTypeDef GPIO_InitStructure; // stm32fxx_gpio.h
    GPIO_StructInit( &GPIO_InitStructure );

    // Set up the peripheral clocks
    RCC_AHB1PeriphClockCmd( DATA_READY_CLK, ENABLE );

    // Initialize USER-DRDY (B3: 380, C13: IAR) pin as an INPUT pin
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;     /// input/output/alt func/analog
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    /// push-pull or open-drain
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;     /// Up/Down/NoPull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; /// low/med/fast/high speed

    // Specify the data-ready/UART selection pin (from boardDefinition.h)
    GPIO_InitStructure.GPIO_Pin = DATA_READY_PIN;

    // Initialize the DR pin
    GPIO_Init( DATA_READY_PORT, &GPIO_InitStructure);
}


/** ***************************************************************************
 * @name    _InitGPIOPin_Sync()
 * @brief   Initialize the GPIO pin used to provide a sync signal from the SPI
 *          master
 *
 * Note: The SPI master provides a sync signal (nominally 1 kHz) over A0 to
 *       mimic a customer's device.
 *
 * @param [in] N/A
 * @retval N/A.
 ******************************************************************************/
void InitGPIOPin_Sync(void)
{
    // Declare and initialize the GPIO Structure
    GPIO_InitTypeDef GPIO_InitStructure; // stm32fxx_gpio.h
    GPIO_StructInit( &GPIO_InitStructure );

    // Set up the peripheral clocks
    RCC_AHB1PeriphClockCmd( ONE_PPS_CLK, ENABLE );

    // Initialize SYNC (A0) pin as an OUTPUT pin
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;     /// input/output/alt func/analog
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    /// push-pull or open-drain
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;     /// Up/Down/NoPull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; /// low/med/fast/high speed

    // Specify the sync pin (from boardDefinition.h)
    GPIO_InitStructure.GPIO_Pin = ONE_PPS_PIN;

    // Initialize the sync pin
    GPIO_Init( ONE_PPS_PORT, &GPIO_InitStructure);
}


void InitGPIOPin_Sync_asPWM(){
  // Declare and initialize the GPIO Structure
    GPIO_InitTypeDef GPIO_InitStructure; // stm32fxx_gpio.h
    GPIO_StructInit( &GPIO_InitStructure );

    // Set up the peripheral clocks
    RCC_AHB1PeriphClockCmd( ONE_PPS_CLK, ENABLE );

    // Initialize SYNC (A0) pin as an OUTPUT pin
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;     /// input/output/alt func/analog
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    /// push-pull or open-drain
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;     /// Up/Down/NoPull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; /// low/med/fast/high speed
    
    GPIO_PinAFConfig(ONE_PPS_PORT, 0, GPIO_AF_TIM5);

    // Specify the sync pin (from boardDefinition.h)
    GPIO_InitStructure.GPIO_Pin = ONE_PPS_PIN;

    // Initialize the sync pin
    GPIO_Init( ONE_PPS_PORT, &GPIO_InitStructure);
}

/** ***************************************************************************
 * @name    _InitGPIOPin_Ex()
* @brief   Initialize the GPIO pins in GPIOE
 *
 * Note: GPIOE pins are used to toggle power to the slave device as well as assert
 *       and de-assert the reset pin on the UUT.
 *
 * @param [in] N/A
 * @retval N/A.
 ******************************************************************************/
void InitGPIOPin_Ex(void)
{
    // Declare and initialize the GPIO Structure
    GPIO_InitTypeDef GPIO_InitStructure; // stm32fxx_gpio.h
    GPIO_StructInit( &GPIO_InitStructure );

    // Set up the peripheral clocks
    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOE, ENABLE );

    // Initialize SYNC (A0) pin as an OUTPUT pin
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;     /// input/output/alt func/analog
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    /// push-pull or open-drain
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;     /// Up/Down/NoPull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; /// low/med/fast/high speed

    // E0 (set low after init)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init( GPIOE, &GPIO_InitStructure);
    GPIOE->BSRRH = GPIO_Pin_0;

    // E1 (set low after init)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init( GPIOE, &GPIO_InitStructure);
    GPIOE->BSRRH = GPIO_Pin_1;

    // E2 (set low after init)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init( GPIOE, &GPIO_InitStructure);
    GPIOE->BSRRH = GPIO_Pin_2;
}


/** ***************************************************************************
 * @name    _InitGPIOPin_SharedCommunication()
 * @brief   Initialize the two shared GPIO pins (A15 & D2)
 *
 * Note: Used on the 380 to select between UART and SPI communications based on
 *       state of pin (unit configured to communicate via SPI when Data-Ready
 *       pin is pulled low)pins in GPIOE are used to toggle power to the slave
 *       device as well as assert and de-assert the reset pin on the UUT.
 *
 * @param [in] N/A
 * @retval N/A.
 ******************************************************************************/
void InitGPIOPin_SharedCommunication( void )
{
    // Declare and initialize the GPIO Structure
    GPIO_InitTypeDef GPIO_InitStructure; // stm32fxx_gpio.h
    GPIO_StructInit( &GPIO_InitStructure );

    // Set up the peripheral clocks
    RCC_AHB1PeriphClockCmd( SPI3_SLAVE_SELECT_CLK, ENABLE );
    RCC_AHB1PeriphClockCmd( USER_B_UART_RX_GPIO_CLK, ENABLE );

    // Initialize UARTB Rx (pin D2) and SPI3 nSS (pin A15) as INPUT pins so, regardless of how SPI
    //   is configured (in 'spi_configure'), neither interferes with the other
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;     /// input/output/alt func/analog
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    /// push-pull or open-drain
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;     /// Up/Down/NoPull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; /// low/med/fast/high speed

    // Initialize the SPI3 nSS pin (A15)
    GPIO_InitStructure.GPIO_Pin = SPI3_SLAVE_SELECT_PIN;
    GPIO_Init( SPI3_SLAVE_SELECT_PORT, &GPIO_InitStructure);

    // Initialize the UART5 Rx pin(D2)
    GPIO_InitStructure.GPIO_Pin = USER_B_UART_RX_PIN;
    GPIO_Init( USER_B_UART_RX_GPIO_PORT, &GPIO_InitStructure);
}


///** ***************************************************************************
// * @name    _InitGPIOPin_A4()
// * @brief   Initialize GPIO pin A$
// *
// * Note: Used for debugging purposes (can remove when complete)
// *
// * @param [in] N/A
// * @retval N/A.
// ******************************************************************************/
//void InitGPIOPin_A4(void)
//{
//    // Declare and initialize the GPIO Structure
//    GPIO_InitTypeDef GPIO_InitStructure; // stm32fxx_gpio.h
//    GPIO_StructInit( &GPIO_InitStructure );
//
//    // Set up the peripheral clocks
//    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE );
//
//    // Initialize SYNC (A4) pin as an INPUT pin (IAR board only)
//    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;     /// input/output/alt func/analog
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    /// push-pull or open-drain
//    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;     /// Up/Down/NoPull
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; /// low/med/fast/high speed
//
//    // Specify the sync pin (from boardDefinition.h)
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
//
//    // Initialize the sync pin
//    GPIO_Init( GPIOA, &GPIO_InitStructure);
//}

