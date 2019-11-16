/**
 ******************************************************************************
 * @file    main.c
 * @author  Kamil Rzechowski
 * @version V1.0.0
 * @brief   Main program body
 ******************************************************************************
 
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "utils.h"
#include "trace.h"
#include "PID.h"

/** @addtogroup F4_RR
 * @{
 */

/**
 * GPIO the Pin Allocation table. The name is constructed as:
 * P<port name><pin number>
 * ================================================================================================
 * | PIN  | used to
 * ================================================================================================
 * | PA0  | blue button													|
 * | PA1  | MPU_9250 (invensense sensor interrupt pin)					| INVEN_INT_PIN
 * | PA2  | HCSR04 (TRIGGER pin) - Ultrasonic distance measurement		| HCSR04_TRIGGER_PIN
 * | PA3  | USB (D0) - not used (#define USE_ULPI_PHY)					|
 * | PA4  | Audio codec DAC (I2S_WS) - not used							| CODEC_I2S_WS_PIN
 * | PA4  | Audio codec DAC (DAC channel 1 & 2) - not used				|
 * | PA5  | LIS3DSH (SPI SC) - accelerometer							| LIS3DSH_SPI_SCK_PIN
 * | PA5  | USB (CLK) - not used (#define USE_ULPI_PHY)					|
 * | PA6  | LIS3DSH (SPI MISO) - accelerometer							| LIS3DSH_SPI_MISO_PIN
 * | PA7  | LIS3DSH (SPI_MOSI) - accelerometer							| LIS3DSH_SPI_MOSI_PIN
 * | PA9  | USB (SOF VBUS)												|
 * | PA10 | USB (SOF ID)												|
 * | PA11 | USB (SOF DM)												|
 * | PA12 | USB (SOF DP)												|
 * | PB0  | PWM	- Motor 3 / CH3											|
 * | PB0  | USB (D1) - not used (#define USE_ULPI_PHY)					|
 * | PB1  | PWM	- Motor 4 / CH4															|
 * | PB1  | USB (D2) - not used (#define USE_ULPI_PHY)					|
 * | PB5  | USB (D3) - not used (#define USE_ULPI_PHY)					|
 * | PB6  | Audio codec DAC (I2C_SCL) - not used						| CODEC_I2C_SCL_PIN
 * | PB9  | Audio codec DAC (I2C_SDA) - not used						| CODEC_I2C_SDA_PIN
 * | PB10 | MPU_9250 (I2Cx pin: SCL)									| SENSORS_I2C_SCL_GPIO_PIN
 * | PB10 | USB (D4) - not used (#define USE_ULPI_PHY)					|
 * | PB10 | Audio codec DAC (SPI SCK) - not used						| SPI_SCK_PIN
 * | PB10 | Audio codec DAC (I2S_SCK) - not used						|
 * | PB11 | MPU_9250 (I2Cx pin: SDA)									| SENSORS_I2C_SDA_GPIO_PIN
 * | PB11 | USB (D5) - not used (#define USE_ULPI_PHY)					|
 * | PB12 | USB (D6) - not used (#define USE_ULPI_PHY)					|
 * | PB12 | Audio codec DAC (I2S_SD) - not used							|
 * | PB13 | USB (D7) - not used (#define USE_ULPI_PHY)					|
 * | PB14 | USB - not used												|
 * | PC0  | USB (STP) - not used (#define USE_ULPI_PHY)					|
 * | PC0  | USB (HOST_POWERSW_VBUS)										| HOST_POWERSW_VBUS
 * | PC1  | MPU_9250 power												|
 * | PC3  | Audio codec DAC (SPI MOSI) - not used						| SPI_MOSI_PIN
 * | PC4  | HCSR04 (ECHO pin)- Ultrasonic distance measurement			| HCSR04_ECHO_PIN
 * | PC6  | PWM	- Motor 1 / CH1											|
 * | PC7  | PWM	- Motor 2 / CH2											|
 * | PC7  | Audio codec DAC (I2S_MCK) - not used						| CODEC_I2S_MCK_PIN
 * | PC10 | Audio codec DAC (I2S_SCK) - not used						| CODEC_I2S_SCK_PIN
 * | PC12 | Audio codec DAC (I2S_SD) - not used							| CODEC_I2S_SD_PIN
 * | PD4  | Audio codec DAC (Reset)										| AUDIO_RESET_PIN
 * | PD12 | LED green													| LED_GREEN
 * | PD13 | LED orange													| LED_RED
 * | PD14 | LED red														| LED_BLUE
 * | PD15 | Led blue													| LED_ORANGE
 * | PE0  | LIS3DSH (SPI INT1) - accelerometer							| LIS3DSH_SPI_INT1_PIN
 * | PE1  | LIS3DSH (SPI INT2) - accelerometer							| LIS3DSH_SPI_INT2_PIN
 * | PE3  | LIS3DSH (SPI CS) - accelerometer							| LIS3DSH_SPI_CS_PIN
 * | PH4  | USB (NXT) - not used (#define USE_ULPI_PHY)					|
 * | PI11 | USB (DIR) - not used (#define USE_ULPI_PHY)					|
 * ================================================================================================
 * | TIM  |
 * ================================================================================================
 * | TIM2 | Time Delay (USB)
 * | TIM3 | PWM main clock
 * | TIM4 | PWM LED test
 * | TIM5 | Time Delay (utils) / HCSR04 Ultrasonic timer counter
 * | TIM10| Time delta counter for PID algorithm
 * ================================================================================================
 */

/* External variables --------------------------------------------------------*/
extern HCSR04_Status_t HCSR04_Status;
extern int32_t HCSR04_Distance;
extern uint8_t cmdResBuf[];

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint32_t cmdSendLis3DshData = 0; // defines pulling interval of the Lis3dsh via BT. 0 = disabled
uint32_t cmdSendMpu9250hData = 0; // defines pulling interval of the MPU-9250 via BT. 0 = disabled
uint32_t cmdSendHCSR04Data = 0; // defines pulling interval of the HCSR04 via BT. 0 = disabled

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program.
 * At this stage the microcontroller clock setting is already configured,
 * this is done through SystemInit() function which is called from startup
 * file (startup_stm32f4xx.s) before to branch to application main.
 * To reconfigure the default setting of SystemInit() function, refer to
 * system_stm32f4xx.c file
 * @param  None
 * @retval None
 */

int main(void) {
	uint16_t diody, diodyBt = 0;
	char c1 = 0, c2;
	uint32_t licznik = 6;
	uint32_t btLongPressed = 0;
	float PWM1proc = 0.0f;
	short x, y, z;
	uint32_t cmdHelperCounterL = 0;
	uint32_t cmdHelperCounterM = 0;
	uint32_t cmdHelperCounterH = 0;
	uint16_t cmdResLen;

	SysTick_Setup();

	// Setups the peripherals
	diag_log(dbg_level_log, "Setup peripherals\n");

	Delay_Setup();
	USB_BTC_host_Setup();

	PWM_TIM_Setup(SERVO_MIN);

	PWM_Led_test();
	Leds_Setup();
	Leds_Light(LED_GREEN | LED_RED | LED_BLUE | LED_ORANGE);
	Button_Setup();
	Accelerometer_LIS3DSH_Setup();
	Accelerometer_MPU_9250_Setup();
	Ultrasonic_HCSR04_Setup();
	initPid();
	diag_log(dbg_level_info, "System clock: %u Hz\r\n", SystemCoreClock);

	while (1) {
		// Perform USB tasks. Should be called as often as possible
		USB_HostTasksHandler();
		// Perform PID tasks. Should be called as often as possible
		PID_TaskHandler();
		//This is the menu used by the button
		diody = 0;
		c2 = Button_Read();
		if (c1 != c2) {
			c1 = c2;
			if (c1 == 1) {
				btLongPressed = 0;
				switch (licznik) {
				case 0:
					licznik++;
					diodyBt = LED_GREEN;
					PWM1proc += 0.1f;
					if (PWM1proc > 1.0f)
						PWM1proc = 0.0f;
					PWM_SetSpeed(Ch1, PWM1proc);
					break;
				case 1:
					diodyBt = LED_ORANGE;
					PWM1proc += 0.1f;
					if (PWM1proc > 1.0f)
						PWM1proc = 0.0f;
					PWM_SetSpeed(Ch1, PWM1proc);
					licznik++;
					break;
				case 2:
					diodyBt = LED_RED;
					PWM1proc += 0.1f;
					if (PWM1proc > 1.0f)
						PWM1proc = 0.0f;
					PWM_SetSpeed(Ch1, PWM1proc);
					licznik++;
					break;
				case 3:
					diodyBt = LED_BLUE;
					PWM1proc += 0.1f;
					if (PWM1proc > 1.0f)
						PWM1proc = 0.0f;
					PWM_SetSpeed(Ch1, PWM1proc);
					licznik++;
					break;
					// 4,5 is intentionally empty
				case 6:
					PWM_MotorControl(SERVO_MIN, SERVO_MIN, SERVO_MIN,
							SERVO_MIN);
					diodyBt = LED_GREEN | LED_RED | LED_BLUE | LED_ORANGE;
					licznik = 0;
					break;
				default:
					diodyBt = 0;
					licznik = 0;
				}
			}
		}
		if (c2 == 1) {
			btLongPressed++;
			if (btLongPressed > 100) {
				#ifdef ENABLE_TEMP_TRACE_DUMP
				BTC_TempTraceDump();
				#endif
				diodyBt = LED_GREEN | LED_RED | LED_BLUE | LED_ORANGE;
			}
		}
		// this is to read the accelerometer && set LEDs accordingly
		Accelerometer_LIS3DSH_read(&x, &y, &z, LIS3DSH_Normalized);
		if (x > LIS3DSH__ACCEPT_DATA) {
			diody |= LED_RED;
		}
		if (x < -LIS3DSH__ACCEPT_DATA) {
			diody |= LED_GREEN;
		}
		if (y > LIS3DSH__ACCEPT_DATA) {
			diody |= LED_ORANGE;
		}
		if (y < -LIS3DSH__ACCEPT_DATA) {
			diody |= LED_BLUE;
		}
		diody |= diodyBt;
		Leds_Light(diody);

		// read HCSR04 ultrasonic
		if (HCSR04_Status != HCSR04_NOT_INITIALIZED && (cmdHelperCounterH > cmdSendHCSR04Data)) {
			Ultrasonic_HCSR04_Read();
		}

		// check if MPU-9250 was initialized.
		if (isMPU9250_Initialised()<=0) {
			diag_log(dbg_level_error, "MPU9250 initialization failed (%i), retry.", (int)isMPU9250_Initialised());
			Accelerometer_MPU_9250_Setup();
		}
		// read MPU-9250 data. There is interrupt used, if no data function does nothing
		MPU_9250_ReadData();
		// read Lis3DSH data. There is interrupt used, if no data function does nothing
		Accelerometer_LIS3DSH_read(&x, &y, &z, LIS3DSH_NotNormalized);
		// This elements are to send the data notifications via Bluetooth
		// cmdHelperCounter is used to calculate the frequency of pulling the data
		if (cmdSendLis3DshData > 0) {
			if ((cmdHelperCounterL++) > cmdSendLis3DshData) {
				cmdResLen = snprintf((char*)cmdResBuf, CMD_RES_BUF_MAX_LENGTH,"%hi,%hi,%hi\r\n", x, y, z);
				CMD_RES_add(CMD_RES_LIS3DSH, cmdResLen, cmdResBuf, CMD_RES_queue_res_str);
				cmdHelperCounterL = 0;
			}
		}
		if (cmdSendMpu9250hData > 0) {
			if ((cmdHelperCounterM++) > cmdSendMpu9250hData) {
				MPU_9250_NotifyData();
				cmdHelperCounterM = 0;
				//cmdResLen = snprintf((char*)cmdResBuf, CMD_RES_BUF_MAX_LENGTH,"%hi,%hi,%hi,%hi\r\n", (int)gyroData[0], (int)gyroData[1], (int)gyroData[2],(int)gyroData[3]);
				//CMD_RES_add(CMD_RES_MPU9250, cmdResLen, cmdResBuf, CMD_RES_queue_res);
			}
		}
		if (cmdSendHCSR04Data > 0) {
			if ((cmdHelperCounterH++) > cmdSendHCSR04Data) {
				cmdResLen = snprintf((char*)cmdResBuf, CMD_RES_BUF_MAX_LENGTH,"%i\r\n", (int)HCSR04_Distance);
				CMD_RES_add(CMD_RES_HCSR04, cmdResLen, cmdResBuf, CMD_RES_queue_res_str);
				cmdHelperCounterH = 0;
			}
		}
	}
}
