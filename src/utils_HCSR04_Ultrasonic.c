/*
 * utils_HCSR04.c
 *
 *  Created on: 23 sty 2016
 *      Author: r.rzechowski
 */

/** @addtogroup F4_RR
  * @{
  */

/** @defgroup HCSR04
 * @brief    Measure distance with HC-SR04 Ultrasonic distance sensor
 */

/* Includes ------------------------------------------------------------------*/
#include "utils.h"
#include "stm32f4xx_tim.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define GPIO_SetPinLow(GPIOx, GPIO_Pin)			((GPIOx)->BSRRH = (GPIO_Pin))
#define GPIO_SetPinHigh(GPIOx, GPIO_Pin) 		((GPIOx)->BSRRL = (GPIO_Pin))
#define GPIO_GetInputPinValue(GPIOx, GPIO_Pin)	(((GPIOx)->IDR & (GPIO_Pin)) == 0 ? 0 : 1)

/* Default timeout pulses */
#ifndef HCSR04_TIMEOUT
#define HCSR04_TIMEOUT			1000000
#endif
/**
 * @brief  Time in microseconds to centimeters conversion
 */
#define HCSR04_DIST_FACT		58.0f
#define HCSR04_PULSE			10

#define HCSR04_ECHO_PIN			GPIO_Pin_4
#define HCSR04_ECHO_PORT		GPIOC
#define HCSR04_TRIGGER_PIN		GPIO_Pin_2
#define HCSR04_TRIGGER_PORT		GPIOA

/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
HCSR04_Status_t HCSR04_Status = HCSR04_NOT_INITIALIZED;
int32_t HCSR04_Distance = -1;              /*!< Distance measured from sensor in minimizers*/
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
uint32_t countRaising = 0x000000;
uint32_t countFalling = 0xFFFFFF;
typedef enum {
	HCSR04_INIT,
	HCSR04_TRIGER,
	HCSR04_RAISING,
	HCSR04_FALLING,
	HCSR04_PROBLEM
} HCSR04_state_t;
HCSR04_state_t hcsr04state = HCSR04_INIT;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Ultrasonic_HCSR04_Setup
  * 	Distance measurement unit setup.
  * @brief  Initializes HC-SR04 sensor
  * @param  None
  * @retval HCSR04_Status:
  * 	- HCSR04_OK - initialized properly (echo received)
  * 	- HCSR04_NOT_INITIALIZED - error (no echo)
  */
HCSR04_Status_t Ultrasonic_HCSR04_Setup() {
	hcsr04state = HCSR04_INIT;
	// ************************************************************************
	// Initialize trigger pin
	// ************************************************************************
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = HCSR04_TRIGGER_PIN;
#ifndef ULTRASONIC_USE_TIMER_AND_INT
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
#else
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
#endif
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(HCSR04_TRIGGER_PORT, &GPIO_InitStructure);
#ifdef ULTRASONIC_USE_TIMER_AND_INT
	// Connect TIM5 pins to AF
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
#endif

	// ************************************************************************
	// Initialize echo pin
	//*************************************************************************
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = HCSR04_ECHO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
#ifndef ULTRASONIC_USE_TIMER_AND_INT
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
#else
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
#endif
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(HCSR04_ECHO_PORT, &GPIO_InitStructure);

	//Trigger set to low
	GPIO_SetPinLow(HCSR04_TRIGGER_PORT, HCSR04_TRIGGER_PIN);
	//GPIO_ResetBits(HCSR04_TRIGGER_PORT, HCSR04_TRIGGER_PIN);
	// Start measurement, check if sensor is working
#ifndef ULTRASONIC_USE_TIMER_AND_INT
	if (Ultrasonic_HCSR04_Read() >= 0) {
		// Sensor OK
		HCSR04_Status = HCSR04_OK;
	} else {
		// Sensor error
		HCSR04_Status = HCSR04_ERROR;
	}
#else
	// ************************************************************************
	// Setup interrupt on TIM5 update event
	// ************************************************************************
	NVIC_InitTypeDef NVIC_InitStructure;
	// Set the Vector Table base address at 0x08000000
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x00);
	// Configure the Priority Group to 2 bits
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//Enable the TIM5 global Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	// ************************************************************************
	// Setup timer - PWM signal on tiger pin
	// ************************************************************************
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5 , ENABLE);
	// configure timer
	// PWM frequency = 100 hz with 84'000'000 hz (system clock / 2)
	// 24 ,000 ,000/240 = 1'000'000
	// 1'000'000/10'000 = 100
	// As the period was to short (the raising edge was triggered, but falling was not detected before next tiger) I have increased the TIM_Period 3 times.
	TIM_TimeBaseStructInit (& TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler	= SystemCoreClock /(2*1000000) - 1; // 0..2399
	TIM_TimeBaseStructure.TIM_Period = 3*10000 - 1; // 0..9999 increased 3 times 29999
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5 , &TIM_TimeBaseStructure);
	// PWM1 Mode configuration: Channel2
	// Edge -aligned; not single pulse mode
	TIM_OCStructInit (& TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // PWM mode 1 = Clear on compare match ; PWM mode 2 = Set on compare match
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = HCSR04_PULSE;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM5 , &TIM_OCInitStructure); // as PA2 is assigned to channel 3 of TIM5
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM5, ENABLE);

	// TIM IT enable
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	//TIM_ITConfig(TIM5, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 |  TIM_IT_COM | TIM_IT_Trigger | TIM_IT_Break, ENABLE);
	// Enable Timer
	TIM_Cmd(TIM5 , ENABLE);
	// Trigger for 10us = 1 cycle period
	// TIM_SetCompare3(TIM5 , 10);

	// ************************************************************************
	// Setup interrupt on echo pin (PC4)
	// ************************************************************************
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource4); // Tell system that you will use PD3 for EXTI_Line3
	EXTI_InitStruct.EXTI_Line = EXTI_Line4; // PC4 is connected to EXTI_Line4
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt; // Interrupt mode
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling; // Triggers on rising and falling edge
	EXTI_InitStruct.EXTI_LineCmd = ENABLE; // Enable interrupt
	EXTI_Init(&EXTI_InitStruct); // Add to EXTI

	// Add IRQ vector to NVIC
	// PC4 is connected to EXTI_Line4, which has EXTI4_IRQn vector
	NVIC_InitStruct.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x0F; // Set priority
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x0F; // Set sub priority
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE; // Enable interrupt
	NVIC_Init(&NVIC_InitStruct); // Add to NVIC

	HCSR04_Status = HCSR04_OK;
#endif
	return HCSR04_Status;
}

/**
* @brief  Echo pin interrupt callback
* @param  None
* @retval none
*/
#ifdef ULTRASONIC_USE_TIMER_AND_INT
void Ultrasonic_HCSR04_callback()
{
	if (GPIO_GetInputPinValue(HCSR04_ECHO_PORT, HCSR04_ECHO_PIN) == 1) {
		if (hcsr04state == HCSR04_TRIGER)
		{
			hcsr04state = HCSR04_RAISING;
			countRaising = TIM5->CNT;
		}
		else {
			hcsr04state = HCSR04_PROBLEM;
			if (HCSR04_Distance >= 0)
				HCSR04_Distance = -2;
		}
	} else {
		if (hcsr04state == HCSR04_RAISING)
		{
			hcsr04state = HCSR04_FALLING;
			countFalling = TIM5->CNT;
			HCSR04_Distance =  (int32_t)((countFalling - countRaising)*10) / HCSR04_DIST_FACT;
		}
		else {
			hcsr04state = HCSR04_PROBLEM;
			if (HCSR04_Distance >= 0)
				HCSR04_Distance = -3;
		}

	}
}

void Deyaly_TimerIRQ (void) {
	if (hcsr04state != HCSR04_FALLING) {
		if (HCSR04_Distance >= 0)
			HCSR04_Distance = -4;
	}
	hcsr04state = HCSR04_TRIGER;
}
#endif

/**
* @brief  Starts sensor measurement and read it's data
* @param  None
* @retval Distance in float:
*            - > 0: Valid distance in cm (centimeters)
*            -  -1: Error
*/
#ifndef ULTRASONIC_USE_TIMER_AND_INT
int32_t Ultrasonic_HCSR04_Read() {
	uint32_t time, timeout;
	/* Trigger set to low */
	GPIO_SetPinLow(HCSR04_TRIGGER_PORT, HCSR04_TRIGGER_PIN);
	//GPIO_ResetBits(HCSR04_TRIGGER_PORT, HCSR04_TRIGGER_PIN);
	/* Delay 2 us */
	Delay_uSec(2);
	/* Trigger high for 10us */
	GPIO_SetPinHigh(HCSR04_TRIGGER_PORT, HCSR04_TRIGGER_PIN);
	//GPIO_SetBits(HCSR04_TRIGGER_PORT, HCSR04_TRIGGER_PIN);
	/* Delay 10 us */
	Delay_uSec(10);
	/* Trigger set to low */
	GPIO_SetPinLow(HCSR04_TRIGGER_PORT, HCSR04_TRIGGER_PIN);
	//GPIO_ResetBits(HCSR04_TRIGGER_PORT, HCSR04_TRIGGER_PIN);

	/* Give some time for response */
	timeout = HCSR04_TIMEOUT;
	while (!GPIO_ReadInputDataBit(HCSR04_ECHO_PORT, HCSR04_ECHO_PIN)) {
		if ((timeout--) == 0x00) {
			HCSR04_Distance = -1;
			return HCSR04_Distance;
		}
	}

	/* Start time */
	time = 0;
	/* Wait till signal is low */
	while (GPIO_GetInputPinValue(HCSR04_ECHO_PORT, HCSR04_ECHO_PIN)) {
	//while (GPIO_ReadInputDataBit(HCSR04_ECHO_PORT, HCSR04_ECHO_PIN)) {
		/* Increase time */
		time++;
		/* Delay 1us */
		Delay_uSec(1);
	}

	/* Convert us to millimeters */
	HCSR04_Distance =  time / HCSR04_DIST_FACT * 1000;

	/* Return distance */
	return HCSR04_Distance;
}
#endif

/**
  * @}
  */
