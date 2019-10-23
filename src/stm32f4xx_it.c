/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    28-October-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "utils.h"

/** @addtogroup F4_RR
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t capture = 0;
extern __IO uint16_t CCR_Val;
extern USB_OTG_CORE_HANDLE USB_OTG_Core;
extern USBH_HOST USB_Host;
extern uint32_t systick_10ms_cnt;

#ifdef ACCELEROMETER_LIS3DSH_USES_SYSTICK
extern uint8_t Acclerometer_LIS3DSH_initialized = 0;
extern uint8_t CounterLIS3DSH;
#endif

#ifdef SYSTICK_DELAY
extern __IO uint32_t SysTick_TimingDelay;
#endif
extern LIS3DSH_OutXYZTypeDef AxisValLIS3DSH[LIS3DSH_NUM_OF_PROBES];
extern uint8_t ArrayElementLIS3DSH;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  prvGetRegistersFromStack
  *   The procedure is used to discover the last instruction before the exception.
  *   The CMSIS names for the fault handlers are as follows:
  *   	- UsageFault_Handler()
  *   	- BusFault_Handler()
  *   	- MemMang_Handler()
  *   	- HardFault_Handler()
  * The procedure is based on
  * http://www.freertos.org/Debugging-Hard-Faults-On-Cortex-M-Microcontrollers.html
  * @param  pulFaultStackAddress
  * @retval None
  */
void prvGetRegistersFromStack( uint32_t *pulFaultStackAddress )
{
	// These are volatile to try and prevent the compiler/linker optimising them
	// away as the variables never actually get used.  If the debugger won't show the
	// values of the variables, make them global my moving their declaration outside
	// of this function.
	#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
	volatile uint32_t r0;
	volatile uint32_t r1;
	volatile uint32_t r2;
	volatile uint32_t r3;
	volatile uint32_t r12;
	volatile uint32_t lr; // Link register.
	volatile uint32_t pc; // Program counter.
	volatile uint32_t psr;// Program status register.
	#pragma GCC diagnostic warning "-Wunused-but-set-variable"

    r0 = pulFaultStackAddress[ 0 ];
    r1 = pulFaultStackAddress[ 1 ];
    r2 = pulFaultStackAddress[ 2 ];
    r3 = pulFaultStackAddress[ 3 ];

    r12 = pulFaultStackAddress[ 4 ];
    lr = pulFaultStackAddress[ 5 ];
    pc = pulFaultStackAddress[ 6 ];
    psr = pulFaultStackAddress[ 7 ];

    // When the following line is hit, the variables contain the register values.
    // =============================================================================
    // The first register of interest is the program counter. In the code above, the
    // variable pc contains the program counter value. pc holds the address of the
    // instruction that was executing when the hard fault (or other fault) occurred.
    //
    // Knowing the instruction that was being executed when the fault occurred allows
    // you to know which other register values are also of interest. For example, if the
    // instruction was using the value of R7 as an address, then the value of R7 needs to
    // be know. Further, examining the assembly code, and the C code that generated the
    // assembly code, will show what R7 actually holds (it might be the value of a
    // variable, for example).
    // =============================================================================
    // The mapping between program counter and C calls can be discovered based on
    // memory map file /Debug/*.map
    // http://community.silabs.com/t5/32-bit-MCU-Knowledge-Base/Interpreting-the-GCC-map-file-in-Simplicity-IDE/ta-p/164031

    for( ;; );
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
	 __asm volatile
	    (
	        " tst lr, #4                                                \n"
	        " ite eq                                                    \n"
	        " mrseq r0, msp                                             \n"
	        " mrsne r0, psp                                             \n"
	        " ldr r1, [r0, #24]                                         \n"
	        " ldr r2, handler2_address_const                            \n"
	        " bx r2                                                     \n"
	        " handler2_address_const: .word prvGetRegistersFromStack    \n"
	    );
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	// systick_10ms_cnt is used in:
	// MPU_9250.c - get_tick_count() for timestamp
	// btc_usbh_core.c - for USB operations timestamp used in timeout calculations
	// btstack_run_loop_embedded.c - for BTstack timeout calculations
	systick_10ms_cnt++;

#ifdef SYSTICK_DELAY
	if (SysTick_TimingDelay != 0x00) {
		SysTick_TimingDecrement();
	}
#endif

#ifdef ACCELEROMETER_LIS3DSH_USES_SYSTICK
	if (Acclerometer_LIS3DSH_initialized != 0) {
		CounterLIS3DSH++;
		if (CounterLIS3DSH >= 10) {
			if (ArrayElementLIS3DSH >= LIS3DSH_NUM_OF_PROBES) ArrayElementLIS3DSH = 0;
			lis3dsh_ReadAxes(&(AxisValLIS3DSH[ArrayElementLIS3DSH]));
			ArrayElementLIS3DSH ++;
			// Update autoreload and capture compare registers value
			CounterLIS3DSH = 0x00;
		}
	}
#endif
}


/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  EXTI0_IRQHandler
  *         This function handles External line 0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
  /* Checks whether the User Button EXTI line is asserted*/
  if (EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
#ifdef BUTTON_GENERATES_INT
#endif

#ifdef ACCELEROMETER_LIS3DSH_USES_INT
	  if (ArrayElementLIS3DSH >= LIS3DSH_NUM_OF_PROBES)
		  ArrayElementLIS3DSH = 0;
	  lis3dsh_ReadAxes(&(AxisValLIS3DSH[ArrayElementLIS3DSH]));
	  ArrayElementLIS3DSH++;
#endif
	  /* Clears the EXTI's line pending bit.*/
	  EXTI_ClearITPendingBit(EXTI_Line0);
  }
}

/**
  * @brief  This function handles External line 1 interrupt request.
  * @param  None
  * @retval None
  */

void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		gyro_data_ready_cb();
		/* Clear the EXTI line 1 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

/**
  * @brief  This function handles External line 4 interrupt request.
  * @param  None
  * @retval None
  */

void EXTI4_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line4) != RESET)
	{
		Ultrasonic_HCSR04_callback();
		/* Clear the EXTI line 4 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line4);
	}
}

/**
  * @brief  This function handles External line 5 interrupt request.
  * @param  None
  * @retval None
  */

void EXTI5_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line5) != RESET)
  {
    USB_Host.usr_cb->OverCurrentDetected();
    /* Clear the EXTI line 5 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line5);
  }
}

/**
  * @brief  This function handles TIM2 global interrupt request.
  * @param  None
  * @retval None
  */

void TIM2_IRQHandler(void)
{
  USB_OTG_BSP_TimerIRQ();
}

/**
  * @brief  This function handles TIM4 global interrupt request.
  * @param  None
  * @retval None
  */

void TIM4_IRQHandler(void)
{
  /* Checks whether the TIM interrupt has occurred */
  if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)
  {
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);

    capture = TIM_GetCapture1(TIM4);
    TIM_SetCompare1(TIM4, capture + CCR_Val);
  }
}

/**
  * @brief  This function handles TIM5 global interrupt request.
  * @param  None
  * @retval None
  */

void TIM5_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
		Deyaly_TimerIRQ();
	}
}

/**
  * @brief  This function handles USB-On-The-Go FS global interrupt request.
  * @param  None
  * @retval None
  */

void OTG_FS_IRQHandler(void)
{
  USBH_OTG_ISR_Handler(&USB_OTG_Core);
}

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/


/**
  * @}
  */ 
  
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
