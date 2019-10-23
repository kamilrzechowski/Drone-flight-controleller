/*
 * utils.c
 *
 *  Created on: 14 sty 2016
 *      Author: r.rzechowski
 */

/** @addtogroup F4_RR
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "utils.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define TIM_MSEC_DELAY                     0x01
#define TIM_USEC_DELAY                     0x02
//#define DEF_TIM_PERIOD					   5
/* Private variables ---------------------------------------------------------*/
__IO uint16_t CCR_Val = 16826;
RCC_ClocksTypeDef RCC_Clocks;
__IO uint32_t Time_delay;
uint32_t multiplier;

#ifdef SYSTICK_DELAY
__IO uint32_t SysTick_TimingDelay = 0;
#endif
/* Exported variables --------------------------------------------------------*/
// systick_10ms_cnt is used in:
// MPU_9250.c - get_tick_count() for timestamp
// btc_usbh_core.c - for USB operations timestamp used in timeout calculations
// btstack_run_loop_embedded.c - for BTstack timeout calculations
volatile uint32_t systick_10ms_cnt = 0;
/* Private function prototypes -----------------------------------------------*/
#ifndef ULTRASONIC_USE_TIMER_AND_INT
static void ACCURATE_TIME_Delay(uint32_t nTime, uint8_t unit);
static void ACCURATE_TIME_SetTime(uint8_t unit);
#endif
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  SysTick_Setup
  * @param  None
  * @retval None
  */
void SysTick_Setup() {
	  /* SysTick end of count event each 10ms */
	  RCC_GetClocksFreq(&RCC_Clocks);
	  SysTick_Config(RCC_Clocks.HCLK_Frequency / 100);
	  systick_10ms_cnt = 0;
}

#ifdef SYSTICK_DELAY
/**
  * @brief  SysTick_Delay
  * 	Inserts a delay time.
  * @param  Specifies the delay time length, in milliseconds.
  * @retval None
  */
void SysTick_Delay(__IO uint32_t nTime) {
	SysTick_TimingDelay = nTime;

	while (SysTick_TimingDelay != 0)
	;
}

/**
  * @brief  SysTick_TimingDecrement
  * 	Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void SysTick_TimingDecrement(void) {
	if (SysTick_TimingDelay != 0x00) {
		SysTick_TimingDelay --;
	}
}
#endif


#ifndef ULTRASONIC_USE_TIMER_AND_INT
/**
  * @brief  Inserts a delay time. Implemented based on TIM5.
  * @param  nTime: specifies the delay time length, in ms.
  * @retval None
  */
void Delay_mSec(uint32_t nTime)
{
	ACCURATE_TIME_Delay(nTime, TIM_MSEC_DELAY);
}

/**
  * @brief  Inserts a delay time. Implemented based on TIM5.
  * @param  nTime: specifies the delay time length, in us.
  * @retval None
  */
void Delay_uSec(uint32_t nTime)
{
	ACCURATE_TIME_Delay(nTime, TIM_USEC_DELAY);
}

/**
  * @brief  ACCURATE_TIME_Delay
  *         Delay routine based on TIM5
  * @param  nTime : Delay Time
  * @param  unit : Delay Time unit : mili sec / micro sec
  * @retval None
  */
static void ACCURATE_TIME_Delay(uint32_t nTime, uint8_t unit)
{

  Time_delay = nTime;
  ACCURATE_TIME_SetTime(unit);
  while (Time_delay != 0);
  TIM_Cmd(TIM5, DISABLE);
}

/**
  * @brief  ACCURATE_TIME_SetTime
  *         Configures TIM5 for delay routine based on TIM5
  * @param  unit : msec /usec
  * @retval None
  */
static void ACCURATE_TIME_SetTime(uint8_t unit)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  TIM_Cmd(TIM5, DISABLE);
  TIM_ITConfig(TIM5, TIM_IT_Update, DISABLE);


  if (unit == TIM_USEC_DELAY)
  {
    TIM_TimeBaseStructure.TIM_Period = 11;
  }
  else if (unit == TIM_MSEC_DELAY)
  {
    TIM_TimeBaseStructure.TIM_Period = 11999;
  }
  TIM_TimeBaseStructure.TIM_Prescaler = 5;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
  TIM_ClearITPendingBit(TIM5, TIM_IT_Update);

  TIM_ARRPreloadConfig(TIM5, ENABLE);

  /* TIM IT enable */
  TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

  /* TIM5 enable counter */
  TIM_Cmd(TIM5, ENABLE);
}

/**
  * @brief  USB_OTG_BSP_TimeInit
  *         Initializes delay unit using Timer2
  * @param  None
  * @retval None
  */
void Delay_Setup()
{
//	RCC_GetClocksFreq(&RCC_Clocks);
//	uSecPrescaller = (uint32_t) ((RCC_Clocks.PCLK2_Frequency) / (1000000 * DEF_TIM_PERIOD)) - 1;
//	mSecPrescaller = (uint32_t) ((RCC_Clocks.PCLK2_Frequency) / (1000 * DEF_TIM_PERIOD)) - 1;

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Set the Vector Table base address at 0x08000000 */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x00);

	/* Configure the Priority Group to 2 bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* Enable the TIM5 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
}

/**
  * @brief  USB_OTG_BSP_TimeInit
  *         Initializes delay unit using Timer2
  * @param  None
  * @retval None
  */
void Deyaly_TimerIRQ (void)
{
    if (Time_delay > 0x00)
    {
      Time_delay--;
    }
    else
    {
      TIM_Cmd(TIM5, DISABLE);
    }
}
#else
void Delay_Setup() {
    /* Get system clocks */
    RCC_GetClocksFreq(&RCC_Clocks);

    /* While loop takes 4 cycles */
    /* For 1 us delay, we need to divide with 4M */
    multiplier = RCC_Clocks.HCLK_Frequency / 4000000;
}
void Delay_mSec(uint32_t millis) {
    /* Multiply millis with multipler */
    /* Substract 10 */
    millis = 1000 * millis * multiplier ;//- 10;
    /* 4 cycles for one loop */
    while (millis--);
}
void Delay_uSec(uint32_t micros) {
    /* Multiply micros with multipler */
    /* Substract 10 */
    micros = micros * multiplier; //- 10;
    /* 4 cycles for one loop */
    while (micros--);
}

#endif

/**
  * @}
  */
