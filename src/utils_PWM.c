/*
 * utils_PWM.c
 *
 *  Created on: 16 sty 2016
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

/*The PWM TIM variables*/
#define PWM_TIM_COUNTER_CLOCK 	1000000
#define PWM_PERIOD				20000
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
uint16_t PrescalerValue = 0;

/*PWM signal to drive brush-less motor */
#define PWM_Motor1 TIM3->CCR1
#define PWM_Motor2 TIM3->CCR2
#define PWM_Motor3 TIM3->CCR3
#define PWM_Motor4 TIM3->CCR4
//#define PWM_MOTOR_MIN 810
//#define PWM_MOTOR_MAX 1650

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  PWM_TIM_Setup
  * 	Setups TIM PWM driver for TIM3. Can steer 4 motors on the Timer using
  * 	pre-scaler and comparer counter.
  * @param  None
  * @retval None
  */
void PWM_TIM_Setup(uint32_t initialSpeed) {
	GPIO_InitTypeDef GPIO_InitStructure;
	// TIM3 clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	/// GPIOC and GPIOB clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOB, ENABLE);

	// --------------------------------------------------------------------------------------------
	// PWM pins are connected to fixed possible pins
	//
	// TIMER   |CHANNEL 1            |CHANNEL 2            |CHANNEL 3            |CHANNEL 4
	//         |PP1    PP2    PP3    |PP1    PP2    PP3    |PP1    PP2    PP3    |PP1    PP2    PP3
	//
	// TIM 1   |PA8    PE9    -      |PA9    PE10   -      |PA10   PE13   -      |PA11   PE14   -
	// TIM 2   |PA0    PA5    PA15   |PA1    PB3    -      |PA2    PB10   -      |PA3    PB11   -
	// TIM 3   |PA6    PB4    PC6    |PA7    PB5    PC7    |PB0    PC8    -      |PB1    PC9    -
	// TIM 4   |PB6    PD12   -      |PB7    PD13   -      |PB8    PD14   -      |PB9    PD15    -
	// TIM 5   |PA0    PH10   -      |PA1    PH11   -      |PA2    PH12   -      |PA3    PI0    -
	// TIM 8   |PC6    PI5    -      |PC7    PI6    -      |PC8    PI7    -      |PC9    PI2    -
	// TIM 9   |PA2    PE5    -      |PA3    PE6    -      |-      -      -      |-      -      -
	// TIM 10  |PB8    PF6    -      |-      -      -      |-      -      -      |-      -      -
	// TIM 11  |PB9    PF7    -      |-      -      -      |-      -      -      |-      -      -
	// TIM 12  |PB14   PH6    -      |PB15   PH9    -      |-      -      -      |-      -      -
	// TIM 13  |PA6    PF8    -      |-      -      -      |-      -      -      |-      -      -
	// TIM 14  |PA7    PF9    -      |-      -      -      |-      -      -      |-      -      -
	//
	// 	- PPx: Pins Pack 1 to 3, for 3 possible channel outputs on timer.
	//
	// Notes on table above:
	// 	- Not all timers are available on all STM32F4xx devices
	// 	- All timers have 16-bit prescaler
	// 	- TIM6 and TIM7 don't have PWM feature, they are only basic timers
	// 	- TIM2 and TIM5 are 32bit timers
	// 	- TIM9 and TIM12 have two PWM channels
	// 	- TIM10, TIM11, TIM13 and TIM14 have only one PWM channel
	// 	- All channels at one timer have the same PWM frequency!
	// --------------------------------------------------------------------------------------------

	// GPIOC Configuration: TIM3 CH1 (PC6) and TIM3 CH2 (PC7)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//GPIOB Configuration:  TIM3 CH3 (PB0) and TIM3 CH4 (PB1)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// Connect TIM3 pins to AF2
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);

	// -----------------------------------------------------------------------
	// TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles.
	// In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1),
	// since APB1 prescaler is different from 1.
	// TIM3CLK = 2 * PCLK1
	// PCLK1 = HCLK / 4
	// => TIM3CLK = HCLK / 2 = SystemCoreClock /2
	// for this example:
	// PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / PWM_TIM_COUNTER_CLOCK) - 1;
	//
	// To get TIM3 counter clock at 28 MHz, the prescaler is computed as follows:
	// Prescaler = (TIM3CLK / TIM3 counter clock) - 1
	// Prescaler = ((SystemCoreClock /2) /28 MHz) - 1
	//
	// To get TIM3 output clock at 30 KHz, the period (ARR)) is computed as follows:
	// ARR = (TIM3 counter clock / TIM3 output clock) - 1 = 665
	//
	// TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
	// TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
	// TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
	// TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%
	//
	// Note1:
	// SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
	// Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
	// function to update SystemCoreClock variable value. Otherwise, any configuration
	// based on this variable will be incorrect.
	//
	// Note2:
	// For Motor driving the prescaler could be set to 1kHz
	// -----------------------------------------------------------------------

	// -----------------------------------------------------------------------
	// Compute the pre-scaler value to be 1000000 Hz.
	// (for 28 MHz example it was --> #define PWM_TIM_COUNTER_CLOCK 28000000)
	// for 1MHz it is --> #define PWM_TIM_COUNTER_CLOCK 1000000
	// -----------------------------------------------------------------------
	PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / PWM_TIM_COUNTER_CLOCK) - 1;

	// -----------------------------------------------------------------------
	// Time base configuration
	// We need a 50 Hz period (1000 / 20ms = 50), thus divide 100000 by 50 = 20000 (us).
	// (for 28 MHz example it was --> #define PWM_PERIOD 665)
	// for 50 Hz it is --> #define PWM_PERIOD 20000
	// -----------------------------------------------------------------------

	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	// PWM1 Mode configuration: Channel1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	// (for 28 MHz example TIM_Pulse was --> 333)
	TIM_OCInitStructure.TIM_Pulse = initialSpeed;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	// PWM1 Mode configuration: Channel2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	// (for 28 MHz example TIM_Pulse was --> 249)
	TIM_OCInitStructure.TIM_Pulse = initialSpeed;
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

	// PWM1 Mode configuration: Channel3
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	// (for 28 MHz example TIM_Pulse was --> 166)
	TIM_OCInitStructure.TIM_Pulse = initialSpeed;
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	// PWM1 Mode configuration: Channel4
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	// (for 28 MHz example TIM_Pulse was --> 83)
	TIM_OCInitStructure.TIM_Pulse = initialSpeed;
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);

	// TIM3 enable counter
	TIM_Cmd(TIM3, ENABLE);
}

/**
  * @brief  PWM_SetSpeed
  * This can be used as alternative for PWM_MotorControl.
  * @param  Channel (motor) for which the speed should be set
  * and percentage of the full power.
  * The possible values for channel: Ch1, Ch2, Ch3, Ch4
  * The possible valued for percent: 0-100%
  * WARNING: small % values need to be checked with physical
  * motor device.
  * @retval None
  */
void PWM_SetSpeed(PWM_TIM_Channel_t TIM_Output, float percent) {
	uint32_t TIM_PulseVal;

	if (percent >= 100.0) {
		TIM_PulseVal = SERVO_MAX; //PrescalerValue;
	} else if (percent == 0) {
		TIM_PulseVal = SERVO_MIN; //0;
	} else {
		TIM_PulseVal = SERVO_MIN +(uint32_t)((float)(SERVO_MAX-SERVO_MIN) * percent); //(uint32_t) ((float) (PrescalerValue) * percent);
	}

	//GPIOB Configuration: CH1(PC6), CH2 (PC7), CH3(PB0),CH4(PB1)
	switch (TIM_Output) {
	case Ch1:
		TIM_SetCompare1(TIM3, TIM_PulseVal);
		break;
	case Ch2:
		TIM_SetCompare2(TIM3, TIM_PulseVal);
		break;
	case Ch3:
		TIM_SetCompare3(TIM3, TIM_PulseVal);
		break;
	case Ch4:
		TIM_SetCompare4(TIM3, TIM_PulseVal);
		break;
	}
}

/**
  * @brief  PWM_MotorControl
  * This can be used as alternative for PWM_SetSpeed.
  * @param  Set actual values of comparer counter for each individual
  * channel (motor).
  * WARNING: small values need to be checked with physical
  * motor device.
  * @retval None
  */
void PWM_MotorControl(uint32_t Motor1, uint32_t Motor2, uint32_t Motor3,
		uint32_t Motor4) {
	if (Motor1 > SERVO_MAX) //PWM_MOTOR_MAX)
		Motor1 = SERVO_MAX; // PWM_MOTOR_MAX;
	if (Motor1 < SERVO_MIN) //PWM_MOTOR_MIN)
		Motor1 = SERVO_MIN; //PWM_MOTOR_MIN;

	if (Motor2 > SERVO_MAX) //PWM_MOTOR_MAX)
		Motor2 = SERVO_MAX; // PWM_MOTOR_MAX;
	if (Motor2 < SERVO_MIN) //PWM_MOTOR_MIN)
		Motor2 = SERVO_MIN; //PWM_MOTOR_MIN;

	if (Motor3 > SERVO_MAX) //PWM_MOTOR_MAX)
		Motor3 = SERVO_MAX; // PWM_MOTOR_MAX;
	if (Motor3 < SERVO_MIN) //PWM_MOTOR_MIN)
		Motor3 = SERVO_MIN; //PWM_MOTOR_MIN;

	if (Motor4 > SERVO_MAX) //PWM_MOTOR_MAX)
		Motor4 = SERVO_MAX; // PWM_MOTOR_MAX;
	if (Motor4 < SERVO_MIN) //PWM_MOTOR_MIN)
		Motor4 = SERVO_MIN; //PWM_MOTOR_MIN;

	PWM_Motor1 = (__IO uint32_t)Motor1;
	PWM_Motor2 = (__IO uint32_t)Motor2;
	PWM_Motor3 = (__IO uint32_t)Motor3;
	PWM_Motor4 = (__IO uint32_t)Motor4;

	//TODO: Rafal - dopisac obsluge motorow
}

/**
  * @brief  PWM_GetMotorControlVal
  * This can be used for reporting
  * @param  Motor1, Motor2, Motor3, Motor4  - individual motor channels.
  * @retval None
  */
void PWM_GetMotorControlVal(uint32_t *Motor1, uint32_t *Motor2, uint32_t *Motor3,
		uint32_t *Motor4) {
	*Motor1 = (uint32_t)PWM_Motor1;
	*Motor2 = (uint32_t)PWM_Motor2;
	*Motor3 = (uint32_t)PWM_Motor3;
	*Motor4 = (uint32_t)PWM_Motor4;
}

#ifdef LEDS_TOGLE
#define TIM_ARR                          ((uint16_t)1900)
#define TIM_CCR                          ((uint16_t)1000)
#endif
/**
  * @brief  PWM_Led_test
  * If the LEDS_TOGLE is defined this test should blink the
  * LEDs using TIM4 as reference counter.
  * @param  None
  * @retval None
  */
void PWM_Led_test() {
#ifdef LEDS_TOGLE
	// -------------------------------------------------------------------------
	// The example for for LEDs flashing
	// -------------------------------------------------------------------------
	// System Clocks Configuration - TIM4 clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	// System Clocks Configuration - GPIOD clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	// GPIOD Configuration: Pins 12, 13, 14 and 15 in output push-pull
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	// Connect TIM4 pins to AF2
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);

	// -------------------------------------------------------------------------
	// The example for for LEDs flashing
	// -------------------------------------------------------------------------
	// TIM4 Configuration: Output Compare Timing Mode:
	// In this example TIM4 input clock (TIM4CLK) is set to 2 * APB1 clock (PCLK1),
	// since APB1 prescaler is different from 1 (APB1 Prescaler = 4, see system_stm32f4xx.c file).
	// TIM4CLK = 2 * PCLK1
	// PCLK1 = HCLK / 4
	// => TIM4CLK = 2*(HCLK / 4) = HCLK/2 = SystemCoreClock/2
	//
	// To get TIM4 counter clock at 2 KHz, the prescaler is computed as follows:
	// Prescaler = (TIM4CLK / TIM1 counter clock) - 1
	// Prescaler = (168 MHz/(2 * 2 KHz)) - 1 = 41999
	//
	// To get TIM4 output clock at 1 Hz, the period (ARR)) is computed as follows:
	// ARR = (TIM4 counter clock / TIM4 output clock) - 1 = 1999
	// TIM4 Channel1 duty cycle = (TIM4_CCR1/ TIM4_ARR)* 100 = 50%
	// TIM4 Channel2 duty cycle = (TIM4_CCR2/ TIM4_ARR)* 100 = 50%
	// TIM4 Channel3 duty cycle = (TIM4_CCR3/ TIM4_ARR)* 100 = 50%
	// TIM4 Channel4 duty cycle = (TIM4_CCR4/ TIM4_ARR)* 100 = 50%
	// ==> TIM4_CCRx = TIM4_ARR/2 = 1000  (where x = 1, 2, 3 and 4).
	//
	// Note:
	// SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
	// Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
	// function to update SystemCoreClock variable value. Otherwise, any configuration
	// based on this variable will be incorrect.
	// -------------------------------------------------------------------------
	//Compute the prescaler value
	PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 2000) - 1;
	// Time base configuration
	TIM_TimeBaseStructure.TIM_Period = TIM_ARR;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	// Enable TIM4 Preload register on ARR
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	// TIM PWM1 Mode configuration: Channel
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = TIM_CCR;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	// Output Compare PWM1 Mode configuration: Channel1
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_CCxCmd(TIM4, TIM_Channel_1, DISABLE);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	//Output Compare PWM1 Mode configuration: Channel2
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_CCxCmd(TIM4, TIM_Channel_2, DISABLE);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	// Output Compare PWM1 Mode configuration: Channel3
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_CCxCmd(TIM4, TIM_Channel_3, DISABLE);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	//Output Compare PWM1 Mode configuration: Channel4
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_CCxCmd(TIM4, TIM_Channel_4, DISABLE);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	//TIM4 enable counter
	TIM_Cmd(TIM4, ENABLE);
#endif
}

/**
  * @}
  */
