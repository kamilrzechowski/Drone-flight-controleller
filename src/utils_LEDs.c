/*
 * utils_LEDs.c
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
  * @brief  Leds_Setup
  * 	Setups LEDs.
  * @param  None
  * @retval None
  */
void Leds_Setup() {
	/* Initiate leds pins */
	/* GPIOG Peripheral clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Configure pins in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = LED_GREEN | LED_RED | LED_BLUE | LED_ORANGE;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	/* Initiate leds pins */
	GPIO_Init(LED_PORT, &GPIO_InitStructure);
}

/**
  * @brief  Leds_Light
  * 	Can be one or more LEDs to light.
  * 	Available options:
  * 	- LED_GREEN
  * 	- LED_ORANGE
  * 	- LED_RED
  * 	- LED_BLUE
  * @param  None
  * @retval None
  */
void Leds_Light(uint16_t pin) {
	if ((pin & LED_GREEN))
		GPIO_SetBits(GPIOD, LED_GREEN);
	else
		GPIO_ResetBits(GPIOD, LED_GREEN);
	if ((pin & LED_ORANGE))
		GPIO_SetBits(GPIOD, LED_ORANGE);
	else
		GPIO_ResetBits(GPIOD, LED_ORANGE);
	if ((pin & LED_RED))
		GPIO_SetBits(GPIOD, LED_RED);
	else
		GPIO_ResetBits(GPIOD, LED_RED);
	if ((pin & LED_BLUE))
		GPIO_SetBits(GPIOD, LED_BLUE);
	else
		GPIO_ResetBits(GPIOD, LED_BLUE);
}

/**
  * @brief  Leds_OnOff
  * 	Turns the LEDS on,off or togle depending on the switch.
  * 	Available options:
  * 	- Led_On
  * 	- Led_Off
  * 	- Led_Toggle
  * @param  None
  * @retval None
  */
void Leds_OnOff(uint16_t pin, LED_OnOff_t state) {
	if (state == Led_On) {
		switch (pin) {
		case LED_GREEN:
			GPIO_SetBits(GPIOD, LED_GREEN);
			break;
		case LED_ORANGE:
			GPIO_SetBits(GPIOD, LED_ORANGE);
			break;
		case LED_RED:
			GPIO_SetBits(GPIOD, LED_RED);
			break;
		case LED_BLUE:
			GPIO_SetBits(GPIOD, LED_BLUE);
			break;
		}
	} else if (state == Led_Off) {
		switch (pin) {
		case LED_GREEN:
			GPIO_ResetBits(GPIOD, LED_GREEN);
			break;
		case LED_ORANGE:
			GPIO_ResetBits(GPIOD, LED_ORANGE);
			break;
		case LED_RED:
			GPIO_ResetBits(GPIOD, LED_RED);
			break;
		case LED_BLUE:
			GPIO_ResetBits(GPIOD, LED_BLUE);
			break;
		}
	} else {
		switch (pin) {
		case LED_GREEN:
			GPIO_ToggleBits(GPIOD, LED_GREEN);
			break;
		case LED_ORANGE:
			GPIO_ToggleBits(GPIOD, LED_ORANGE);
			break;
		case LED_RED:
			GPIO_ToggleBits(GPIOD, LED_RED);
			break;
		case LED_BLUE:
			GPIO_ToggleBits(GPIOD, LED_BLUE);
			break;
		}
	}
}

/**
  * @}
  */
