/**
  ******************************************************************************
  * @file    main.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    28-October-2011
  * @brief   Header for main.c module
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "stm32f4xx.h"
#pragma GCC diagnostic warning "-Wunused-parameter"
#include <stdio.h>
#include "stm32f4xx_it.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "stm32f4xx_exti.h"
#include "usb_hcd_int.h"
#include "usb_bsp.h"
#include "usbh_core.h"
#include "usbh_usr.h"
#include "btc_usbh_core.h"
#include "utils_CMD_RES.h"
#include "MPU_9250.h"
 
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void TIM_timer_Config(void);
void TimingDelay_Decrement(void);
void Delay(__IO uint32_t nTime);

#endif /* __MAIN_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
