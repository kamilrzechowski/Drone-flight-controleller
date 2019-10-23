/*
 * utils_blue_button.c
 *
 *  Created on: 15 sty 2016
 *      Author: r.rzechowski
 */

/** @addtogroup F4_RR
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "utils.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Button_Setup
  * 	Setups blue button on the board.
  * 	Button can generate interrupts if BUTTON_GENERATES_INT is defined
  * @param  None
  * @retval None
  */
void Button_Setup() {
	/* Initiate blue button */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
#ifdef BUTTON_GENERATES_INT
	// Enable clock for SYSCFG. All STM32 peripherals after the pawer-on are not clocked.
	// For clock the peripherals you must look where the peripherals are connected (APB1, APB2, AHB - see the block diagram of mcu).
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#endif

	/* Blue button is connected to Pin 0 */
	/* Configure pins in output push-pull mode */
	GPIO_InitStructure.GPIO_Pin = BUTTON_BLUE_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* Initialize pin on GPIOA port */
	GPIO_Init(BUTTON_BLUE_PORT, &GPIO_InitStructure);

#ifdef BUTTON_GENERATES_INT
	// There are 16 interrupt lines for (GPIOs) - line0 to line15 and they also represent pin number.
	// This means, PA0 is connected to Line0 and PA13 is connected to Line13.
	// PB0 is also connected to Line0 and PC0 also and so on. This is for all pins on board.
	// All Px0 (where x is GPIO name) pins are connected to Line0 and let’s say all Px3 are connected to Line3 on the Interrupt channel.
	// STM32F4 has 7 interrupt handlers for GPIO pins. They are in table below:
	//
	// Irq			Handler				Description
	// EXTI0_IRQn 	EXTI0_IRQHandler 	Handler for pins connected to line 0
	// EXTI1_IRQn 	EXTI1_IRQHandler 	Handler for pins connected to line 1
	// EXTI2_IRQn 	EXTI2_IRQHandler 	Handler for pins connected to line 2
	// EXTI3_IRQn 	EXTI3_IRQHandler 	Handler for pins connected to line 3
	// EXTI4_IRQn 	EXTI4_IRQHandler 	Handler for pins connected to line 4
	// EXTI9_5_IRQn 	EXTI9_5_IRQHandler 	Handler for pins connected to line 5 to 9
	// EXTI15_10_IRQn 	EXTI15_10_IRQHandler 	Handler for pins connected to line 10 to 15
	//
	// This table shows, which IRQ to set for NVIC (first column) and function names to handle your interrupts (second column).
	// Only lines 0 to 4 have own IRQ handler, lines 5-9 have the same interrupt handler and this is also for lines 10 to 15.
	// 	After settings for EXTI, they need to be added into NVIC.
	//
	// NVIC or Nested Vector Interrupt Controller is used to dynamically tell which interrupt is more important and for enabling or disabling interrupts.
	// It supports up to 256 different interrupt vectors.
	// NVIC_IRQChannel - Here you tell for which interrupt you will set settings.
	// NVIC_IRQChannelPreemptionPriority - With this parameter you set interrupt priority, number from 0x00 to 0x0F.
	//		Let’s say we have to use USART receive interrupt and ADC conversion finished interrupt.
	//		USART is more important than ADC, so USART will have lower number than ADC.
	// NVIC_IRQChannelSubPriority - With this parameter you set interrupt priority, number from 0x00 to 0x0F.
	//		Let’s say you have 2 USARTs enabled, but USART1 is more important than USART2. So USART1 will have lower number than USART2.
	// NVIC_IRQChannelCmd - With that you select if interrupt is enabled or disabled.
	// -----------------------------------------------------------------------------------------------------------------------------------------

	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	// Tell system that you will use PD3 for EXTI_Line3
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	// PD3 is connected to EXTI_Line3
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	// Interrupt mode
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	// Triggers on rising and falling edge
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising; //_Falling;
	// Enable interrupt
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	// Add to EXTI
	EXTI_Init(&EXTI_InitStruct);

	// Add IRQ vector to NVIC
	// PD0 is connected to EXTI_Line3, which has EXTI3_IRQn vector
	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
	// Set priority
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x0F;
	// Set sub priority
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x0F;
	// Enable interrupt
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	// Add to NVIC
	NVIC_Init(&NVIC_InitStruct);
#endif
}

/**
  * @brief  Button_Read
  * 	Reads button state
  * @param  None
  * @retval None
  */
uint8_t Button_Read() {
	/* Reads blue button */
	return GPIO_ReadInputDataBit(BUTTON_BLUE_PORT, BUTTON_BLUE_PIN);
}

/**
  * @}
  */
