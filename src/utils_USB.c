/*
 * utils_USB.c
 *
 *  Created on: 16 sty 2016
 *      Author: r.rzechowski
 */

/** @addtogroup F4_RR
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "utils.h"
#include "trace.h"
#include <string.h>


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define FILE_NAME 			"%sUSB_F%03X.TXT"
#define FILE_NAME_SEARCH	"USB*.TXT"

/* Private variables ---------------------------------------------------------*/
USB_OTG_CORE_HANDLE          USB_OTG_Core;
USBH_HOST                    USB_Host;

char ManufacturerStr[64];
char ProductStr[64];
char SerialNumStr[64];
char SpeedStr[64];

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  USB_CDC_host_Setup
  * @param  None
  * @retval None
  */
void USB_BTC_host_Setup()
{
	/* Init Host Library */
	USBH_Init(&USB_OTG_Core, USB_OTG_FS_CORE_ID, &USB_Host, &BTC_cb, &USR_Callbacks);
}


/**
  * @brief  USB_tasksHandler
  * @param  None
  * @retval None
  */
void USB_HostTasksHandler()
{
	/* Host Task handler */
	USBH_Process(&USB_OTG_Core, &USB_Host);
}

