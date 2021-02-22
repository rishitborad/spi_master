/******************************************************************************
* @File name: bsp.c
*
* @brief Use the file to Configure Cortex M3
*
* $Rev: 16166 $
* @date 2011-03-09 11:53:45 -0800 (Wed, 09 Mar 2011)
* @Author  whpeng $
******************************************************************************/
#include "bsp.h"

/** ****************************************************************************
 * @name  led_on()
 *
 * @brief Turns selected LED On.
 *
 * @param [in]Led_TypeDef - led
 * @retal N/A
 ******************************************************************************/
void led_on(Led_TypeDef led)
{
	STM_EVAL_LEDOn(led);
}

/** ****************************************************************************
 * @name  led_off()
 *
 * @brief Turns selected LED off.
 *
 * @param [in}]	Led_TypeDef 	led
 * @retval N/A
 ******************************************************************************/
void led_off(Led_TypeDef led)
{
	STM_EVAL_LEDOff(led);
}

/** ****************************************************************************
* @name  led_toggle
*
* @brief Toggles the selected LED.
*
* @param	Led_TypeDef 	led
* @retval N/A
*******************************************************************************/
void led_toggle(Led_TypeDef led)
{
	STM_EVAL_LEDToggle(led);
}

/** ****************************************************************************
 * @name  BSP_init
 *
 * @brief This function should be called by your application code before you make
 *        use of any of the functions found in this module
 *
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void BSP_init(void)
{
    STM_EVAL_LEDInit(HEARTBEAT_LED);
    STM_EVAL_LEDInit(LED2);
    STM_EVAL_LEDInit(LED3);
    STM_EVAL_LEDInit(LED4);

	/// System clocks configuration
	RCC_config(); // local
	GPIO_config();
	InitSystemTimer();

}

/** ****************************************************************************
 * @name  GPIO_config
 *
 * @brief configuration GPIO
 *
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void GPIO_config(void)
{
  STM_EVAL_LEDInit(HEARTBEAT_LED);
}

/** ****************************************************************************
 * @name  RCC_config
 *
 * @brief configures the different system clocks and Enable peripherals.
 *
 * @param N/A
 * @retval
 ******************************************************************************/
void RCC_config(void)
{
#define SYSTICK_RCC_CONF
#if defined(SYSTICK_RCC_CONF) ///< temporary reserves
	ErrorStatus HSEStartUpStatus;

	RCC_DeInit(); /// RCC system reset(for debug purpose)
	RCC_HSEConfig(RCC_HSE_ON); /// Enable HSE

	/// Wait till HSE is ready
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if(HSEStartUpStatus == SUCCESS)
	{
		/// Enable Prefetch Buffer
        FLASH_PrefetchBufferCmd(ENABLE);
        FLASH_InstructionCacheCmd(ENABLE);
        FLASH_DataCacheCmd(ENABLE);

		FLASH_SetLatency(FLASH_Latency_3);
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div2);
		RCC_PCLK1Config(RCC_HCLK_Div4);
        
                 // 119068800Hz
                //RCC_PLLConfig( RCC_PLLSource_HSE, 25, 240, 2, 5 );
                
                // 99224000Hz
                RCC_PLLConfig( RCC_PLLSource_HSE, 16, 128, 2, 5 );
		RCC_PLLCmd(ENABLE); /// Enable PLL

		/// Wait till PLL is ready
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		{ /* spin */ }

		/// Select PLL as system clock source
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/// Wait till PLL is used as system clock source
		while(RCC_GetSYSCLKSource() != 0x08)
		{ /* spin */ }
	}
    SystemCoreClockUpdate();
#elif defined(SYSTICK_STM32_CONF)
	SystemInit();/* temporary reserves */
#endif

}

/**********************************************************************
* Function name:  NVIC_config
*
* Description:
*		 Configures Vector Table base location.
*
* Trace:
*
* Input parameters:
*
* Output parameters:
*
* Return value:
**********************************************************************/
void NVIC_config(void)
{
#if defined (VECT_TAB_RAM)
	/* Set the Vector Table base location at 0x20000000 */
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#elif defined(VECT_TAB_FLASH_IAP)
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, NVIC_FLASH_IAP);
#else  /* VECT_TAB_FLASH  */
	/* Set the Vector Table base location at 0x08000000 */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif
	/* Configure the NVIC Preemption Priority Bits */
    // FIXME ECW: main.c sets this to _2
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

}

