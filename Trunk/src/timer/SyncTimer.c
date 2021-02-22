#include "SyncTimer.h"

//#include <salvohook_interrupt.h>
#include "salvodefs.h"

// A 32-bit counter incremented at 5 kHz rolls over after 9+ days
//static uint32_t CycleCount = 0;
uint32_t ElapsedTime_sec = 0;
uint32_t ElapsedTime_usec = 0;
uint32_t dt_usec = 0;
//
//static uint8_t IncrementCounter = 0;
uint8_t ToggleSyncPin = 0;
volatile uint8_t pwmEnabled = 0;
uint8_t pwmDivider = 0;
//
//static uint16_t TimerFreq_Hz = 10000;  // 10 kHz
//static uint16_t SyncFreq_Hz = 1000;
uint16_t freqRatio = 1;

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
 ******************************************************************************/
void InitTimer_TIM5( uint32_t TimerFreq_Hz )
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef         NVIC_InitStructure;

    uint32_t period;

    // 1. Enable TIM clock using RCC_APBxPeriphClockCmd(RCC_APBxPeriph_TIMx, ENABLE) function
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM5, ENABLE );

    // Specify the timer period from the desired clock frequency.
    //   One division-by-two is due to the fact that we want a square wave with a period of
    //   TimerFreq_Hz.  To get this, the timer must reach its limit at twice the freq of the square
    //   wave.  The other is ...
    //   ( 120000000 / 10000 ) >> 2 = 12000 >> 2 = 12000 / 4 = 3000
    //period = ( SystemCoreClock / TimerFreq_Hz ) >> 2;
    period = 99996>>1;
    dt_usec = ( 1000000 / TimerFreq_Hz ) >> 1;

    // 2. Fill the TIM_TimeBaseInitStruct with the desired parameters.
    TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );
    TIM_TimeBaseStructure.TIM_Period      = period;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    
    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCStructInit(&TIM_OCInitStruct);
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 300;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
    
    TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
    TIM_OC1Init(TIM5, &TIM_OCInitStruct);
    // 3. Call TIM_TimeBaseInit(TIMx, &TIM_TimeBaseInitStruct) to configure the Time Base unit
    //    with the corresponding configuration
    TIM_TimeBaseInit( TIM5, &TIM_TimeBaseStructure );

    // 4. Enable the NVIC if you need to generate the update interrupt.
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init( &NVIC_InitStructure );

    // 5. Enable the corresponding interrupt using the function TIM_ITConfig(TIMx, TIM_IT_Update)
    TIM_ITConfig( TIM5, TIM_IT_Update, ENABLE );
    TIM_ITConfig(TIM5, TIM_IT_CC1, ENABLE);

    // 6. Call the TIM_Cmd(ENABLE) function to enable the TIM counter.
    TIM_Cmd( TIM5, ENABLE );
}

volatile uint32_t capture_count = 0;
//
// called every 50 usec = toggling A0 at this rate creates a 100 usec period square wave
void TIM5_IRQHandler( void )
{
    static uint8_t pinState = 0;
    static uint16_t TimerCounter = 0;
    static uint8_t counter = 0;
    OSDisableHook();

    if(TIM_GetITStatus(TIM5, TIM_IT_Update) == SET){
      TimerCounter++;

      ElapsedTime_usec = ElapsedTime_usec + dt_usec;
      if( ElapsedTime_usec >= 1000000 ) {
          ElapsedTime_usec = 0;
          ElapsedTime_sec++;
      }
      
      if(pwmEnabled){
        counter++;
        if(counter % pwmDivider == 0){
          counter = 0;
          ONE_PPS_PORT->BSRRH = ONE_PPS_PIN;
        }
      }else
        // Toggle the sync pin at the prescribed rate (if desired)
      if( TimerCounter >= freqRatio ) {
          TimerCounter = 0;

          if( ToggleSyncPin ) {
              if( pinState ) {
                  pinState = 0;
                  ONE_PPS_PORT->BSRRH = ONE_PPS_PIN;
              } else {
                  pinState = 1;
                  ONE_PPS_PORT->BSRRL = ONE_PPS_PIN;
              }
          }
      }
    }
    if(TIM_GetITStatus(TIM5, TIM_IT_CC1) == SET){
      //Compare Channel1 Interrupt
      if(pwmEnabled){
        ONE_PPS_PORT->BSRRL = ONE_PPS_PIN;
      }
    }
    
    // Clear interrupt pending bit
    TIM5->SR = (uint16_t)~(TIM_IT_CC1 | TIM_IT_Update);
    OSEnableHook();
}

// Updates the frequency of the timer5
void TIM5_UpdateTimerTop(uint32_t counts){
  TIM_SetCounter(TIM5, counts);
}


// Duty Cycle in percentage relative to the timer frequency
void TIM5_UpdatePWMDutyCycle(uint8_t dutyCycle){
  uint32_t autoReloadValue = TIM5->ARR;
  uint32_t dutyCycleCount = autoReloadValue * (dutyCycle/100);
  TIM_SetCompare1(TIM5, dutyCycleCount);
}

