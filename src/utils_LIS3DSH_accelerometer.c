/*
 * utils_LIS3DSH_accelerometer.c
 *
 *  Created on: 16 sty 2016
 *      Author: r.rzechowski
 */

/** @addtogroup F4_RR
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "stm32f4_discovery_lis3dsh.h"
#include "stm32f4xx_it.h"
#pragma GCC diagnostic warning "-Wunused-parameter"
#include "utils.h"
#include "trace.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#ifdef ACCELEROMETER_LIS3DSH_USES_SYSTICK
uint8_t CounterLIS3DSH = 0;
#endif
LIS3DSH_OutXYZTypeDef AxisValLIS3DSH[LIS3DSH_NUM_OF_PROBES], AxisOffsetLIS3DSH;
uint8_t ArrayElementLIS3DSH = 0;
#ifdef ACCELEROMETER_LIS3DSH_USES_SYSTICK
uint8_t Acclerometer_LIS3DSH_initialized = 0;
#endif

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Accelerometer_LIS3DSH_Setup
  * 	Accelerometer setup and test.
  * @param  None
  * @retval None
  */
void Accelerometer_LIS3DSH_Setup() {
	uint8_t temp = 0;
	uint32_t i = 0;
	lis3dsh_init();
	if (LIS302DL_LIS3DSH_Detect() != LIS302DL_LIS3DSH_Device_LIS3DSH) {
		UsageFault_Handler();
	}
	lis3dsh_set_OutputDataRate(LIS3DSH_ODR_100_HZ);
	lis3dsh_set_filter(LIS3DSH_FILTER_800Hz, LIS3DSH_SENS_2G);
	lis3dsh_AxesEnable(LIS3DSH_XYZ_ENABLE);

	//wait for accelerometer heating
	while ((temp < LISDSH_TEMP_ACCEPT) && (i < 500)) {
		temp = lis3dsh_ReadTemperature();
		i++;
	}
	Accelerometer_LIS3DSH_selftest();

	// eliminate offset
	AxisOffsetLIS3DSH.x = 0;
	AxisOffsetLIS3DSH.y = 0;
	AxisOffsetLIS3DSH.z = 0;
	for (uint8_t i = 0; i < LIS3DSH_NUM_OF_PROBES; i++) {
		lis3dsh_ReadAxes(&(AxisValLIS3DSH[i]));
		AxisOffsetLIS3DSH.x += AxisValLIS3DSH[i].x;
		AxisOffsetLIS3DSH.y += AxisValLIS3DSH[i].y;
		AxisOffsetLIS3DSH.z += AxisValLIS3DSH[i].z;
	}
	AxisOffsetLIS3DSH.x /= LIS3DSH_NUM_OF_PROBES;
	AxisOffsetLIS3DSH.y /= LIS3DSH_NUM_OF_PROBES;
	AxisOffsetLIS3DSH.z /= LIS3DSH_NUM_OF_PROBES;

#ifdef ACCELEROMETER_LIS3DSH_USES_INT
	Delay_mSec(300);
	lis3dsh_SetInrerrupt(3);
#endif

#ifdef ACCELEROMETER_LIS3DSH_USES_SYSTICK
	SysTick_Delay(30);
	SysTick_Setup();
	Acclerometer_LIS3DSH_initialized = 1;
#endif
}

/**
  * @brief  Accelerometer_LIS3DSH_selftest
  * @param  None
  * @retval result of the test
  */
uint8_t Accelerometer_LIS3DSH_selftest() {
	uint8_t memsTestStatus = 0x00;
	LIS3DSH_OutXYZTypeDef AccelerometerValue;
	int TimingDelay;

	lis3dsh_SetSelfTestMode(LIS3DSH_TST_POSITIVE);
	TimingDelay = SystemCoreClock / 20;
	/* Wait until detecting all MEMS direction or timeout */
	while ((memsTestStatus != 0x01) && (TimingDelay != 0x00)) {
		lis3dsh_ReadAxes(&AccelerometerValue);
		/* Check test PASS condition */

		if ((AccelerometerValue.x < LIS3DSH_PASSCONDITION)
				&& (AccelerometerValue.y < LIS3DSH_PASSCONDITION)) {
			/* MEMS Test PASS */
			memsTestStatus = 0x01;
			diag_log(dbg_level_log, "MEMS test POSITIVE passed\n");
		}
		TimingDelay--;
	}
	lis3dsh_SetSelfTestMode(LIS3DSH_TST_NEGATIVE);
	TimingDelay = SystemCoreClock / 20;
	/* Wait until detecting all MEMS direction or timeout */
	while ((memsTestStatus != 0x02) && (TimingDelay != 0x00)) {
		lis3dsh_ReadAxes(&AccelerometerValue);
		/* Check test PASS condition */
		if ((AccelerometerValue.x > LIS3DSH_PASSCONDITION)
				&& (AccelerometerValue.y > LIS3DSH_PASSCONDITION)
				&& (AccelerometerValue.z > LIS3DSH_PASSCONDITION)) {
			/* MEMS Test PASS */
			memsTestStatus = 0x02;
			diag_log(dbg_level_log, "MEMS test NEGATIVE passed\n");
		}
		TimingDelay--;
	}
	if (memsTestStatus != 0x02) {
		diag_log(dbg_level_error, "MEMS test failed\n");
	}
	lis3dsh_SetSelfTestMode(LIS3DSH_TST_DISABLE);
	return memsTestStatus;
}

/**
  * @brief  Accelerometer_LIS3DSH_direct_read
  * @param  None
  * @retval None
  */
void Accelerometer_LIS3DSH_direct_read(short *x, short *y, short *z,
		LIS3DSH_Direct_ReadMode_t mode) {
	uint8_t i;
	LIS3DSH_OutXYZTypeDef AxisData;

	if (mode == LIS3DSH_ProcessedDataRead) {
		if (ArrayElementLIS3DSH >= LIS3DSH_NUM_OF_PROBES)
			ArrayElementLIS3DSH = 0;
		lis3dsh_ReadAxes(&(AxisValLIS3DSH[ArrayElementLIS3DSH]));
		ArrayElementLIS3DSH++;

		*x = 0;
		*y = 0;
		*z = 0;
		for (i = 0; i < LIS3DSH_NUM_OF_PROBES; i++) {
			*x += AxisValLIS3DSH[i].x;
			*y += AxisValLIS3DSH[i].y;
			*z += AxisValLIS3DSH[i].z;
		}
		*x = (*x / LIS3DSH_NUM_OF_PROBES);
		*y = (*y / LIS3DSH_NUM_OF_PROBES);
		*z = (*z / LIS3DSH_NUM_OF_PROBES);
	} else {
		lis3dsh_ReadAxes(&AxisData);
		*x = AxisData.x;
		*y = AxisData.y;
		*z = AxisData.z;
	}
}

/**
  * @brief  Accelerometer_LIS3DSH_read
  * @param  None
  * @retval None
  */
void Accelerometer_LIS3DSH_read(short *x, short *y, short *z, LIS3DSH_Direct_ReadMode_t mode) {
	uint8_t i;
#if !defined(ACCELEROMETER_LIS3DSH_USES_INT) && !defined(ACCELEROMETER_LIS3DSH_USES_SYSTICK)
	if (ArrayElementLIS3DSH >= LIS3DSH_NUM_OF_PROBES)
		ArrayElementLIS3DSH = 0;
	lis3dsh_ReadAxes(&(AxisValLIS3DSH[ArrayElementLIS3DSH]));
	ArrayElementLIS3DSH++;
#endif

	*x = 0;
	*y = 0;
	*z = 0;
	for (i = 0; i < LIS3DSH_NUM_OF_PROBES; i++) {
		*x += AxisValLIS3DSH[i].x;
		*y += AxisValLIS3DSH[i].y;
		*z += AxisValLIS3DSH[i].z;
	}
	if (mode == LIS3DSH_NotNormalized) {
		*x = (*x / LIS3DSH_NUM_OF_PROBES);
		*y = (*y / LIS3DSH_NUM_OF_PROBES);
		*z = (*z / LIS3DSH_NUM_OF_PROBES);
	} else {
		*x = (*x / LIS3DSH_NUM_OF_PROBES) - AxisOffsetLIS3DSH.x;
		*y = (*y / LIS3DSH_NUM_OF_PROBES) - AxisOffsetLIS3DSH.y;
		*z = (*z / LIS3DSH_NUM_OF_PROBES) - AxisOffsetLIS3DSH.z;
	}
	//trace_printf("x_val = %hi y_val = %hi z_val = %hi\n", *x, *y, *z);
}

/**
  * @}
  */
