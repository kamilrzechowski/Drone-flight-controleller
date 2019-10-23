/*
 * utils.h
 *
 *  Created on: 15 sty 2016
 *      Author: r.rzechowski
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef UTILS_H_
#define UTILS_H_
/* Includes ------------------------------------------------------------------*/
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "stm32f4xx_gpio.h"
#pragma GCC diagnostic warning "-Wunused-parameter"
#include "stm32f4_discovery_lis3dsh.h"
#include <btstack/bluetooth.h>
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define ULTRASONIC_USE_TIMER_AND_INT	1
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/* LEDs ----------------------------------------------------------------------*/
//#define DRIVERS_ARE_NOT_USING_LED // this is to switch off the driver functionality
#define LED_GREEN	GPIO_Pin_12
#define LED_ORANGE	GPIO_Pin_13
#define LED_RED		GPIO_Pin_14
#define LED_BLUE	GPIO_Pin_15
#define LED_PORT	GPIOD
typedef enum {
	Led_On,
	Led_Off,
	Led_Toggle,
} LED_OnOff_t;
void Leds_Setup();
void Leds_Light(uint16_t pin);
void Leds_OnOff(uint16_t pin, LED_OnOff_t state);

/* BUTTON --------------------------------------------------------------------*/
//#define BUTTON_GENERATES_INT
#define BUTTON_BLUE_PIN		GPIO_Pin_0
#define BUTTON_BLUE_PORT	GPIOA
void Button_Setup();
uint8_t Button_Read();

/* Accelerometer LIS3DSH -----------------------------------------------------*/
#define ACCELEROMETER_LIS3DSH_USES_INT
//#define ACCELEROMETER_LIS3DSH_USES_SYSTICK

#ifdef ACCELEROMETER_LIS3DSH_USES_SYSTICK
#define SYSTICK_DELAY
#endif
#if defined (ACCELEROMETER_LIS3DSH_USES_INT) && defined (ACCELEROMETER_LIS3DSH_USES_SYSTICK)
#error "ACCELEROMETER_LIS3DSH_USES_INT and ACCELEROMETER_LIS3DSH_USES_SYSTICK can not be defined together"
#endif
#define LIS3DSH_PASSCONDITION           200
#define LIS3DSH__ACCEPT_DATA			50
#define LISDSH_TEMP_ACCEPT				200
#define LIS3DSH_NUM_OF_PROBES			10
typedef enum {
	LIS3DSH_RowDataRead, LIS3DSH_ProcessedDataRead, LIS3DSH_Normalized, LIS3DSH_NotNormalized
} LIS3DSH_Direct_ReadMode_t;

void Accelerometer_LIS3DSH_Setup();
void Accelerometer_LIS3DSH_read(short *x, short *y, short *z,
		LIS3DSH_Direct_ReadMode_t mode);
void Accelerometer_LIS3DSH_direct_read(short *x, short *y, short *z,
		LIS3DSH_Direct_ReadMode_t mode);
uint8_t Accelerometer_LIS3DSH_selftest();

/* Accelerometer MPU9250 -----------------------------------------------------*/
#define MPU9250_PWR 	GPIO_Pin_1
#define MPU9250_PORT	GPIOC
int8_t isMPU9250_Initialised();
void Accelerometer_MPU_9250_Setup();

/* PWM TIM -------------------------------------------------------------------*/
//#define LEDS_TOGLE
typedef enum {
	Ch1, Ch2, Ch3, Ch4
} PWM_TIM_Channel_t;
void PWM_TIM_Setup(uint32_t initialSpeed);
void PWM_Led_test();
void PWM_MotorControl(uint32_t Motor1, uint32_t Motor2, uint32_t Motor3,
		uint32_t Motor4);
void PWM_SetSpeed(PWM_TIM_Channel_t TIM_Output, float percent);
void PWM_GetMotorControlVal(uint32_t *Motor1, uint32_t *Motor2, uint32_t *Motor3,
		uint32_t *Motor4);
#define SERVO_MAX		2050		// Max. pos. at 2050 us (2.00ms).
#define SERVO_MIN		950			// Min. pos. at 950  us (0.95ms).
#define SERVO_ZERO		0			// Zero (0ms).


/* Flash memory writing ------------------------------------------------------*/
void FlashMemory_WirteLinkKey(bd_addr_t bd_addr, link_key_t link_key, link_key_type_t link_key_type);
uint8_t FlashMemory_ReadLinkKey(bd_addr_t bd_addr, link_key_t link_key, link_key_type_t *link_key_type);

/* SysTick delay -------------------------------------------------------------*/
//#define SYSTICK_DELAY
void SysTick_Setup();
#ifdef SYSTICK_DELAY
void SysTick_Delay(__IO uint32_t nTime);
void SysTick_TimingDecrement(void);
#endif
void Delay_Setup();
void Delay_mSec(uint32_t nTime);
void Delay_uSec(uint32_t nTime);
void Deyaly_TimerIRQ (void);

/* USB host  -----------------------------------------------------------------*/
/* State Machine for the USBH_USR_ApplicationState */
typedef enum {
	USH_USR_CONNECTING,
	USH_USR_CONNECTED,
	USH_USR_APPLICATION,
	USH_USR_DISCONNECTED,
	USB_USR_ERROR
} USB_AppState_t;

void USB_BTC_host_Setup();
void USB_HostTasksHandler();
void CloseFileUnmount();

/* HC-SR04 Ultrasonic distance measurement sensor ----------------------------*/
typedef enum {
	HCSR04_NOT_INITIALIZED,
	HCSR04_OK,
	HCSR04_ERROR
} HCSR04_Status_t;
HCSR04_Status_t Ultrasonic_HCSR04_Setup();
#ifndef ULTRASONIC_USE_TIMER_AND_INT
	float Ultrasonic_HCSR04_Read();
	#define Ultrasonic_HCSR04_callback(...) ((void)0)
#else
	#define Ultrasonic_HCSR04_Read(...) ((void)0)
	void Ultrasonic_HCSR04_callback();
#endif
#if defined (ACCELEROMETER_LIS3DSH_USES_INT) && defined (BUTTON_GENERATES_INT)
#error "ACCELEROMETER_LIS3DSH_USES_INT and BUTTON_GENERATES_INT can not be defined together. They share same Line0."
#endif
/* PID reporting parameter ---------------------------------------------------*/
typedef enum {
	PID__REP_GYRO,
	PID_REP_EULER,
	PID_OUT_PITCH,
	PID_OUT_ROLL,
	PID_OUT_YOW,
	PID_OUT_LEVEL1,
	PID_OUT_LEVEL2,
	PID_REP_ACCEL,
	PID_REP_ACCEL_X,
	PID_REP_ACCEL_Y,
	PID_REP_ACCEL_Z,
	PID_REP_ERROR
} PID_report_param_t;
#endif /* UTILS_H_ */
