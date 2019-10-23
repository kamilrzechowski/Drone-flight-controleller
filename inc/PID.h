/*
 * PID.h
 *
 *  Created on: 21 wrz 2016
 *      Author: Kamil
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PID_H_
#define PID_H_

/* Includes ------------------------------------------------------------------*/
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "stm32f4xx.h"
#include "utils.h"
#pragma GCC diagnostic warning "-Wunused-parameter"

/* Exported types ------------------------------------------------------------*/
typedef struct{
	float P;
	float I;
	float D;
	float kp;
	float ki;
	float kd;
	float last_derivative;
	float last_error;
	float scaler;
	float imax;
	uint8_t derivative_valid;
	float output;
} pid_info_t;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void setExpectedHeight(uint32_t height);
void setRCParams(long pit, long roll, long yaw, uint32_t thr, uint32_t height);
void setPIDSettings(char PID, uint32_t ikp, uint32_t iki, uint32_t ikd, uint32_t iimax, uint32_t iscaler);
void getRCParams(long *pit, long *roll, long *yaw, uint32_t *thr, uint32_t *height, char *rParam);
void setPIDDebug(uint32_t delta);
float get_pid(float dt, float setPoint, float input, float scaler, pid_info_t *pid);
void reset_I(pid_info_t *pid);
float keepRange(float x,float a,float b);
void PID_TaskHandler();
void initPid();

#endif /* PID_H_ */
