
#include <stm32f2xx.h>
#include "boardDefinition.h"

void InitTimer_TIM5( uint32_t TimerFreq_Hz );
void TIM5_UpdateTimerTop(uint32_t counts);
// Duty Cycle in percentage relative to the timer frequency
void TIM5_UpdatePWMDutyCycle(uint8_t dutyCycle);

extern uint8_t ToggleSyncPin;
extern uint16_t freqRatio;
extern volatile uint8_t pwmEnabled;
extern uint8_t pwmDivider;

extern uint32_t ElapsedTime_sec;
extern uint32_t ElapsedTime_usec;

