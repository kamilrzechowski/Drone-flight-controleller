/*
 * utils_MPU9250_accelerometer.c
 *
 *  Created on: 16 sty 2016
 *      Author: r.rzechowski
 */

/* Includes ------------------------------------------------------------------*/
#include "MPU_9250.h"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "stm32f4xx_rcc.h"
#pragma GCC diagnostic warning "-Wunused-parameter"
#include <stdio.h>
#include "trace.h"
#include "packet.h"
#include "utils_CMD_RES.h"
#include "PID.h"

/** @addtogroup F4_RR
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

// ******************************************************************************
// For MPU9250 to compile for the different parts default symbols needed are:
// MPL_LOG_NDEBUG =1
// MPU9250
// EMPL
// USE_DMP
// EMPL_TARGET_STM32F4
// they are defined in pre-processor section of eclipse, but
// also I have included them in in MPU_9250.h but
// ******************************************************************************
#define MPL_LOGE(...)       diag_log(dbg_level_error, __VA_ARGS__)
#define MPL_LOGI(...)       diag_log(dbg_level_info, __VA_ARGS__)
int8_t MPU9250_initialised = 0;

/* Private variables ---------------------------------------------------------*/
extern uint8_t cmdResBuf[];
GPIO_InitTypeDef GPIO_InitStructure;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

int8_t isMPU9250_Initialised() {
	return MPU9250_initialised;
}

void Accelerometer_MPU_9250_Setup() {
	inv_error_t result;
	// --------------------------------------------------------------------------------------------
	// power on MPU-9250
	// --------------------------------------------------------------------------------------------
/*	Delay_mSec(500);
	// GPIOG Peripheral clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	// Configure pins in output pushpull mode
	GPIO_InitStructure.GPIO_Pin |= MPU9250_PWR;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	// Initiate leds pins
	GPIO_Init(MPU9250_PORT, &GPIO_InitStructure);
	// set the power on
	GPIO_SetBits(MPU9250_PORT, MPU9250_PWR);
	// give the time to initialize*/
	Delay_mSec(2000);
	// --------------------------------------------------------------------------------------------
	// initialize the board to communicate with MPU-9250
	// --------------------------------------------------------------------------------------------
	Set_I2C_Retry(5);
	// Enable PWR APB1 Clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	// Allow access to Backup
	PWR_BackupAccessCmd(ENABLE);
	// Reset RTC Domain
	RCC_BackupResetCmd(ENABLE);
	RCC_BackupResetCmd(DISABLE);
	// Configure Interrupts
	GPIO_Config();
	// Configure I2C
	I2cMaster_Init();
	Delay_mSec(500);
	// --------------------------------------------------------------------------------------------
	// initialize the MPU-9250
	// --------------------------------------------------------------------------------------------
	result = MPU_9250_Init();
	if (result) {
		MPL_LOGE("the MPU-9250 was not initialized properly (%i)\n", result);
		MPU9250_initialised = -1;
		return;
	}
	result = run_self_test();
	if (result)
	{
		MPL_LOGE("SelfTest failed (%i)\n", result);
		MPU9250_initialised = -2;
		return;
	}
	MPU9250_initialised = 1;

    /* Depending on your application, sensor data may be needed at a faster or
     * slower rate. These code can speed up or slow down the rate at which
     * the sensor data is pushed to the MPL.
     *
     * In this example, the compass rate is never changed.
     */
    if (MPU_9250_GetDmpOn()) {
    	// 20 is default on initiation. 200 is max (defined as DMP_SAMPLE_RATE). Desired fifo rate (Hz).
    	dmp_set_fifo_rate(200);
    	inv_set_quat_sample_rate(10000L);
    } else {
    	mpu_set_sample_rate(200);
    }
    inv_set_gyro_sample_rate(10000L); // sample_rate_us Set Gyro Sample rate in us
    inv_set_accel_sample_rate(10000L); // sample_rate_us Set Accel Sample rate in us
    inv_set_compass_sample_rate(10000L); //sample_rate_us Set Compass Sample rate in us

    dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);

    /* Set hardware to interrupt periodically. */
    SensorsOnOff(ModeOn, ModeOn, ModeOn);
}

//#define eMPL_send_data(x, ...) ((void)0)

void eMPL_send_data(unsigned char type, long *data) {
	uint16_t cmdResLen = 0;

    if (data == NULL) return;

    switch (type) {
    case PACKET_DATA_ROT:
		cmdResLen = snprintf((char*)cmdResBuf, CMD_RES_BUF_MAX_LENGTH,"%x,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
				type, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);
        break;
    case PACKET_DATA_QUAT:
		cmdResLen = snprintf((char*)cmdResBuf, CMD_RES_BUF_MAX_LENGTH,"%x,%ld,%ld,%ld,%ld\r\n", type, data[0], data[1], data[2],data[3]);
        break;
    case PACKET_DATA_ACCEL:
    case PACKET_DATA_GYRO:
    case PACKET_DATA_COMPASS:
    case PACKET_DATA_EULER:
		cmdResLen = snprintf((char*)cmdResBuf, CMD_RES_BUF_MAX_LENGTH,"%x,%ld,%ld,%ld\r\n", type, data[0], data[1], data[2]);
        break;
    case PACKET_DATA_HEADING:
		cmdResLen = snprintf((char*)cmdResBuf, CMD_RES_BUF_MAX_LENGTH,"%x,%ld\r\n", type, data[0]);
        break;
    case PACKET_DATA_ALL_KEY:
		cmdResLen = snprintf((char*)cmdResBuf, CMD_RES_BUF_MAX_LENGTH,"%x,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
				type, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);
    	break;
    default:
        return;
    }
    CMD_RES_add(CMD_RES_MPU9250, cmdResLen, cmdResBuf, CMD_RES_queue_res_str);
}

/**
  * @}
  */
