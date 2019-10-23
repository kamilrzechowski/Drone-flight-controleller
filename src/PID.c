/*
 * PID.c

 *
 *  Created on: 20 wrz 2016
 *      Author: Kamil
 */

#include "PID.h"
#include "utils.h"
#include "math.h"
#include "stdlib.h"
#include "eMPL_outputs.h"
#include "Trace.h"
#include "utils_CMD_RES.h"

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

/* External variables --------------------------------------------------------*/
pid_info_t PID_Pitch_Rate = { 0, 0, 0,  0.7, 1, 1, 0, 0, 1, 50, 0, 0 },
		PID_Roll_Rate 	  = { 0, 0, 0,  0.7, 1, 1, 0, 0, 1, 50, 0, 0 },
		PID_Yaw_Rate      = { 0, 0, 0,  2.5, 1, 1, 0, 0, 1, 50, 0, 0 },
		PID_Pitch_Stab    = { 0, 0, 0,  4.5, 0, 0, 0, 0, 1,  0, 0, 0 },
		PID_Roll_Stab     = { 0, 0, 0,  4.5, 0, 0, 0, 0, 1,  0, 0, 0 },
		PID_Yaw_Stab      = { 0, 0, 0, 10.0, 0, 0, 0, 0, 1,  0, 0, 0 },
		PID_AccelX        = { 0, 0, 0,  1.0, 0, 0, 0, 0, 1,  0, 0, 0 },
		PID_AccelY        = { 0, 0, 0,  1.0, 0, 0, 0, 0, 1,  0, 0, 0 },
		PID_AccelZ        = { 0, 0, 0,  1.0, 0, 0, 0, 0, 1,  0, 0, 0 };

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const uint8_t _fCut = 20;
uint32_t expectedHeight;
long rcpit = 0, rcroll = 0, rcyaw = 0;
long accelData[3];
long prevEulerData[3], eulerData[3];
long  prevGyroData[3], gyroData[3], gyroAverage[3];
uint8_t firstReadLoop = 1;
uint32_t rcthr = 0;
static float yaw_target = 0.0f;
int8_t accuracyA, accuracyG, accuracyE;
unsigned long prevTimestampA, timestampA, prevTimestampG, timestampG, prevTimestampE, timestampE;
uint16_t prevDelta = 500;
uint32_t counter = 0;
uint8_t MPU_error_counter1 = 0;	//MPU error counter for gyro
uint8_t MPU_error_counter2 = 0; //MPU erroe counter for eulwr data
uint8_t gyroAverageCounter = 0;
binaryBTCbuffer_t binResBuf; // this is the buffer used to construct
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
PID_report_param_t PID_report_param = PID_REP_EULER;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  setExpectedHeight set expected height
 * @param height - the value to set
 * @retval none
 */
void setExpectedHeight(uint32_t height) {
	expectedHeight = height;
}

/**
 * @brief  setRCParams set pit, roll, yaw and throttle from the controller
 * @param pit - the value to set. values = (-45 * 2^16, 45 * 2^16)
 * @param roll - the value to set. values = (-45 * 2^16, 45 * 2^16)
 * @param yaw - the value to set. values = (-150 * 2^16, 150 * 2^16)
 * @param thr - the value to set. values = (SERVO_MIN, SERVO_MAX)
 * @param height - the value to set
 * @retval none
 */
void setRCParams(long pit, long roll, long yaw, uint32_t thr, uint32_t height) {
	rcpit = pit;
	rcroll = roll;
	rcyaw = yaw;
	rcthr = thr;
	expectedHeight = height;
}

void getRCParams(long *pit, long *roll, long *yaw, uint32_t *thr, uint32_t *height, char *rParam) {
	*pit = rcpit;
	*roll = rcroll;
	*yaw = rcyaw;
	*thr = rcthr;
	*height = expectedHeight;
	switch (PID_report_param) {
	case PID__REP_GYRO:
		*rParam = 'G';
		break;
	case PID_REP_EULER:
		*rParam = 'E';
		break;
	case PID_OUT_PITCH:
		*rParam = '1';
		break;
	case PID_OUT_ROLL:
		*rParam = '2';
		break;
	case PID_OUT_YOW:
		*rParam = '3';
		break;
	case PID_OUT_LEVEL1:
		*rParam = '4';
		break;
	case PID_OUT_LEVEL2:
		*rParam = '5';
		break;
	case PID_REP_ACCEL:
		*rParam = '6';
		break;
	case PID_REP_ACCEL_X:
		*rParam = '7';
		break;
	case PID_REP_ACCEL_Y:
		*rParam = '8';
		break;
	case PID_REP_ACCEL_Z:
		*rParam = '9';
		break;
	default:
		*rParam = '?';
	}
}

/**
 * @brief  setPIDSettings sets internal PID values
 * @param PID - which pid
 * @param kp - proportional value
 * @param ki - integral value
 * @param kd - derivative value
 * @param imax - max intagral
 * @param scaler - output value scaler (scaler of output motor value)
 * @retval none
 */
#define SCALE_FACTOR 1000.0f
void setPIDSettings(char PID, uint32_t ikp, uint32_t iki, uint32_t ikd,
		uint32_t iimax, uint32_t iscaler) {
	float kp = ((float) ikp) / SCALE_FACTOR;
	float ki = ((float) iki) / SCALE_FACTOR;
	float kd = ((float) ikd) / SCALE_FACTOR;
	float imax = ((float) iimax) / SCALE_FACTOR;
	float scaler = ((float) iscaler) / SCALE_FACTOR;
	switch (PID) {
	case '1':					//PID_Pitch_Rate
		PID_Pitch_Rate.kp = kp;
		PID_Pitch_Rate.ki = ki;
		PID_Pitch_Rate.kd = kd;
		PID_Pitch_Rate.imax = imax;
		PID_Pitch_Rate.scaler = scaler;
		break;
	case '2':					//PID_Roll_Rate
		PID_Roll_Rate.kp = kp;
		PID_Roll_Rate.ki = ki;
		PID_Roll_Rate.kd = kd;
		PID_Roll_Rate.imax = imax;
		PID_Roll_Rate.scaler = scaler;
		break;
	case '3':					//PID_Yaw_Rate
		PID_Yaw_Rate.kp = kp;
		PID_Yaw_Rate.ki = ki;
		PID_Yaw_Rate.kd = kd;
		PID_Yaw_Rate.imax = imax;
		PID_Yaw_Rate.scaler = scaler;
		break;
	case '4':					//PID_Pitch_Stab
		PID_Pitch_Stab.kp = kp;
		PID_Pitch_Stab.ki = ki;
		PID_Pitch_Stab.kd = kd;
		PID_Pitch_Stab.imax = imax;
		PID_Pitch_Stab.scaler = scaler;
		break;
	case '5':					//PID_Roll_Stab
		PID_Roll_Stab.kp = kp;
		PID_Roll_Stab.ki = ki;
		PID_Roll_Stab.kd = kd;
		PID_Roll_Stab.imax = imax;
		PID_Roll_Stab.scaler = scaler;
		break;
	case '6':					//PID_Yaw_Stab
		PID_Yaw_Stab.kp = kp;
		PID_Yaw_Stab.ki = ki;
		PID_Yaw_Stab.kd = kd;
		PID_Yaw_Stab.imax = imax;
		PID_Yaw_Stab.scaler = scaler;
		break;
	case '7':					//AccelX
		PID_AccelX.kp = kp;
		PID_AccelX.ki = ki;
		PID_AccelX.kd = kd;
		PID_AccelX.imax = imax;
		PID_AccelX.scaler = scaler;
		break;
	case '8':					//AccelY
		PID_AccelY.kp = kp;
		PID_AccelY.ki = ki;
		PID_AccelY.kd = kd;
		PID_AccelY.imax = imax;
		PID_AccelY.scaler = scaler;
		break;
	case '9':					//AccelZ
		PID_AccelZ.kp = kp;
		PID_AccelZ.ki = ki;
		PID_AccelZ.kd = kd;
		PID_AccelZ.imax = imax;
		PID_AccelZ.scaler = scaler;
		break;
	default:
		diag_log(dbg_level_warning, "Wrong PID setting %c\n", PID);
	}
}

/**
 * @brief setPIDDebug sets values for debuging PID alghorithm
 * @param delta - filter of last-current motor output
 * @retval none
 */
void setPIDDebug(uint32_t delta) {
	prevDelta = delta;
}

/**
 * @brief  initPid initialize time base for PID algorithm
 * @retval none
 */
void initPid() {
	memset(prevEulerData, 0, sizeof(prevEulerData));
	prevTimestampE = 0;
	//************************************************************************
	// Initialize PID send buffer
	// ************************************************************************
	binResBuf.size = 0;
	binResBuf.sequence = 0;
	//************************************************************************
	// Setup timer
	// ************************************************************************
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
	// configure timer
	// TIM10 works on 168 000 000 Hz
	TIM_TimeBaseStructure.TIM_Prescaler = 1680;	//every 0,01 ms
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);
	TIM_ARRPreloadConfig(TIM10, ENABLE);
	TIM_Cmd(TIM10, ENABLE);
}

/**
 * @brief  get_pid set output value to the motor
 * @param error - is a difference between desire and actual value
 * @param scaler - multiplier of output value
 * @param pid - struct of pid data
 * @retval output motor correction value
 */
float get_pid(float dt, float setPoint, float input, float scaler, pid_info_t *pid) {
	float output = 0.0f;
	float delta_time;
	float error = setPoint - input;

	if (dt > 1000.0f) {
		dt = 0.0f;
		// if this PID hasn't been used for a full second then zero
		// the intergator term. This prevents I buildup from a
		// previous fight mode from causing a massive return before
		// the integrator gets a chance to correct itself
		reset_I(pid);
	}
	//pid->last_t = tnow;

	delta_time = (float) dt / 1000.0f;

	// Compute proportional component
	pid->P = error * pid->kp;
	output += pid->P;

	// Compute derivative component if time has elapsed
	if ((fabsf(pid->kd) > 0) && (dt > 0)) {
		float derivative;

		if (!(pid->derivative_valid)) {		//(isnan(_last_derivative))
			// we've just done a reset, suppress the first derivative
			// term as we don't want a sudden change in input to cause
			// a large D output change
			pid->derivative_valid = 1;
			derivative = 0.0f;
			pid->last_derivative = 0.0f;
		} else {
			derivative = (error - pid->last_error) / delta_time;
		}

		// discrete low pass filter, cuts out the
		// high frequency noise that can drive the controller crazy
		float RC = 1.0f / (2.0f * M_PI * _fCut);
		derivative = pid->last_derivative + ((delta_time / (RC + delta_time)) * (derivative - pid->last_derivative));

		// update state
		pid->last_error = error;
		pid->last_derivative = derivative;

		// add in derivative component
		pid->D = pid->kd * derivative;
		output += pid->D;
	}

	// scale the P and D components
	output *= scaler;
	pid->D *= scaler;
	pid->P *= scaler;

	// Compute integral component if time has elapsed
	if ((fabsf(pid->ki) > 0) && (dt > 0)) {
		pid->I += (error * pid->ki) * scaler * delta_time;
		if (pid->I < -pid->imax) {
			pid->I = -pid->imax;
		} else if (pid->I > pid->imax) {
			pid->I = pid->imax;
		}
		output += pid->I;
	}

	pid->output = output;
	return output;
}

/**
 * @brief  reset_I reset integral and set last derivative as not valid
 * @param pid - the value to reset
 * @retval none
 */
void reset_I(pid_info_t *pid) {
	pid->I = 0;
	// we use NAN (Not A Number) to indicate that the last derivative value is not valid
	pid->derivative_valid = 0;	//NAN
}

/**
 * @brief  keepRange keeps require range
 * @param x - checked value
 * @param a - low limit
 * @param b - top limit
 * @retval value
 */
float keepRange(float x, float a, float b) {
	if (x < a) {
		return a;
	} else if (x > b) {
		return b;
	}
	return x;
}

int gyrox, gyroy, gyroz, eulerx, eulery, eulerz;
void PID_TaskHandler() {
	float dt;

	counter++;
	if (rcthr > SERVO_MIN + 100) {
		//=======================================================================
		// get and check gyro data
		if (!inv_get_sensor_type_gyro(gyroData, &accuracyG, (inv_time_t*) &timestampG)) {
			//TODO: MPU error
			MPU_error_counter1++;
			if (MPU_error_counter1 == 10) {
				diag_log(dbg_level_error, "MPU_PID_read_error. Initialization status (%i)", (int)isMPU9250_Initialised());
			}
			return;
		} else {
			MPU_error_counter1 = 0;
		}

		if ((memcmp(prevGyroData, gyroData, sizeof(prevGyroData)) == 0)
				&& (prevTimestampG == timestampG)) {
			return;
		}
		memcpy(prevGyroData, gyroData, sizeof(prevGyroData));
		prevTimestampG = timestampG;

		if (firstReadLoop) {
			firstReadLoop = 0;
			gyroAverage[0] = gyroData[0];
			gyroAverage[1] = gyroData[1];
			gyroAverage[2] = gyroData[2];
		} else {
			gyroAverage[0] = (gyroAverage[0] + gyroData[0]) / 2;
			gyroAverage[1] = (gyroAverage[1] + gyroData[1]) / 2;
			gyroAverage[2] = (gyroAverage[2] + gyroData[2]) / 2;
		}

		//=======================================================================
		// get and check accelerometer data
		float accelX_PID = 0.0f;
		float accelX_Range_PID = 0.0f;
		float accelY_PID = 0.0f;
		float accelY_Range_PID = 0.0f;
		float accelZ_PID = 0.0f;
		float accelZ_Range_PID = 0.0f;

		if (inv_get_sensor_type_accel(accelData, &accuracyA, (inv_time_t*) &timestampA)) {
			dt = timestampA - prevTimestampA;
			//TODO: this is the place to add moves left, right, up/down replacing 0.0f
			accelX_PID = get_pid(dt, 0.0f, ((float)(accelData[0])/4096.0f), PID_AccelX.scaler, &PID_AccelX);
			accelX_Range_PID = keepRange(accelX_PID, -5.0f, 5.0f);
			accelY_PID = get_pid(dt, 0.0f, ((float)(accelData[1])/4096.0f), PID_AccelY.scaler, &PID_AccelY);
			accelY_Range_PID = keepRange(accelY_PID, -5.0f, 5.0f);
			accelZ_PID = get_pid(dt, 0.0f, ((float)(accelData[2])/4096.0f), PID_AccelZ.scaler, &PID_AccelZ);
			accelZ_Range_PID = keepRange(accelZ_PID, -5.0f, 5.0f);
			//TODO: add the logic for Z axe manipulation, which should add to X & Y motor speed
			prevTimestampA = timestampA;
		}

		//=======================================================================
		// get and check euler data
		if (!inv_get_sensor_type_euler(eulerData, &accuracyE, (inv_time_t*) &timestampE)) {
			//TODO: MPU error
			MPU_error_counter2++;
			if (MPU_error_counter2 == 10) {
				diag_log(dbg_level_error, "MPU_PID_read_error");
			}
			return;
		} else {
			MPU_error_counter2 = 0;
		}

		if ((memcmp(prevEulerData, eulerData, sizeof(prevEulerData)) == 0)
				&& (prevTimestampE == timestampE)) {
			return;
		}

		memcpy(prevEulerData, eulerData, sizeof(prevEulerData));
		prevTimestampE = timestampE;
		firstReadLoop = 1;

		//=======================================================================
		// storing timer value
		dt = (float) (TIM10->CNT) / 100.0f;
		TIM10->CNT = (__IO uint32_t) 0;

		//=======================================================================
		// calculate stabilization - LEVEL1
		// *** MINIMUM THROTTLE TO DO CORRECTIONS MAKE THIS 20pts ABOVE YOUR MIN THR STICK ***/
		float pitch_stab_output_PID = get_pid(dt, (float) rcpit + accelY_Range_PID, ((float) (eulerData[0]) / 65536.0f), PID_Pitch_Stab.scaler, &PID_Pitch_Stab);
		float pitch_stab_output = keepRange(pitch_stab_output_PID, -250.0f, 250.0f);
		float roll_stab_output_PID = get_pid(dt, (float) rcroll + accelX_Range_PID, ((float) (eulerData[1]) / 65536.0f), PID_Roll_Stab.scaler,  &PID_Roll_Stab);
		float roll_stab_output = keepRange(roll_stab_output_PID, -250.0f, 250.0f);
		float yaw_stab_output_PID = get_pid(dt, yaw_target, ((float)(eulerData[2])/65536.0f), PID_Yaw_Stab.scaler, &PID_Yaw_Stab);
		float yaw_stab_output = keepRange(yaw_stab_output_PID, -360.0f, 360.0f);

	     //=======================================================================
		 // is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
		 //if(abs(rcyaw ) > 5) {
		 //yaw_stab_output = rcyaw;
		 //yaw_target = eulerData[2];   // remember this yaw for when pilot stops
		 //}

		//=======================================================================
		// calculate stabilization - rate PIDS - LEVEL2
		float pitch_output = get_pid(dt, pitch_stab_output, ((float) (gyroData[0]) / 65536.0f), PID_Pitch_Rate.scaler, &PID_Pitch_Rate);
		float roll_output = get_pid(dt, roll_stab_output,  ((float) (gyroData[1]) / 65536.0f), PID_Roll_Rate.scaler,  &PID_Roll_Rate);
		float yaw_output = get_pid(dt, yaw_stab_output, ((float) (gyroData[2])/65536.0f), PID_Yaw_Rate.scaler, &PID_Yaw_Rate);

		//=======================================================================
		// calculate motor. Motor mapping description:
		// MOTOR_FL = 3
		// MOTOR_BL = 2
		// MOTOR_FR = 4
		// MOTOR_BR = 1

		uint32_t BR = (uint32_t) ((float)rcthr + roll_output - pitch_output + yaw_output);
		uint32_t BL = (uint32_t) ((float)rcthr + roll_output + pitch_output - yaw_output);
		uint32_t FL = (uint32_t) ((float)rcthr - roll_output + pitch_output + yaw_output);
		uint32_t FR = (uint32_t) ((float)rcthr - roll_output - pitch_output - yaw_output);

		//=======================================================================
		// set motor PWM - MOTOR ON
		PWM_MotorControl(BR, BL, FL, FR);

		//=======================================================================
		// do reporting
		if (counter % 1 == 0) { //TODO: intentionally always true - the factor to be found later
			switch (PID_report_param) {
			case PID__REP_GYRO:
				// this is to put int32_t into uint8_t buffer without data conversion
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) gyroData[0];
				binResBuf.size += sizeof(int32_t);
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) gyroData[1];
				binResBuf.size += sizeof(int32_t);
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) gyroData[2];
				break;
			case PID_OUT_PITCH:
				// this is to put int32_t into uint8_t buffer without data conversion
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) eulerData[0];
				binResBuf.size += sizeof(int32_t);
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) pitch_stab_output_PID;
				binResBuf.size += sizeof(int32_t);
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) pitch_output;
				break;
			case PID_OUT_ROLL:
				// this is to put int32_t into uint8_t buffer without data conversion
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) eulerData[1];
				binResBuf.size += sizeof(int32_t);
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) roll_stab_output_PID;
				binResBuf.size += sizeof(int32_t);
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) roll_output;
				break;
			case PID_OUT_YOW:
				// this is to put int32_t into uint8_t buffer without data conversion
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) eulerData[2];
				binResBuf.size += sizeof(int32_t);
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) yaw_stab_output_PID;
				binResBuf.size += sizeof(int32_t);
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) yaw_output;
				break;
			case PID_OUT_LEVEL1:
				// this is to put int32_t into uint8_t buffer without data conversion
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) pitch_stab_output_PID;
				binResBuf.size += sizeof(int32_t);
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) roll_stab_output_PID;
				binResBuf.size += sizeof(int32_t);
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) yaw_stab_output_PID;
				break;
			case PID_OUT_LEVEL2:
				// this is to put int32_t into uint8_t buffer without data conversion
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) pitch_output;
				binResBuf.size += sizeof(int32_t);
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) roll_output;
				binResBuf.size += sizeof(int32_t);
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) yaw_output;
				break;
			case PID_REP_ACCEL:
				// this is to put int32_t into uint8_t buffer without data conversion
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) accelData[0];
				binResBuf.size += sizeof(int32_t);
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) accelData[1];
				binResBuf.size += sizeof(int32_t);
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) accelData[2];
				break;
			case PID_REP_ACCEL_X:
				// this is to put int32_t into uint8_t buffer without data conversion
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) accelData[0];
				binResBuf.size += sizeof(int32_t);
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) (accelX_PID * 100.0f);
				binResBuf.size += sizeof(int32_t);
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) (accelX_Range_PID * 100.0f);;
				break;
			case PID_REP_ACCEL_Y:
				// this is to put int32_t into uint8_t buffer without data conversion
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) accelData[1];
				binResBuf.size += sizeof(int32_t);
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) (accelY_PID * 100.0f);
				binResBuf.size += sizeof(int32_t);
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) (accelY_Range_PID * 100.0f);;
				break;
			case PID_REP_ACCEL_Z:
				// this is to put int32_t into uint8_t buffer without data conversion
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) accelData[2];
				binResBuf.size += sizeof(int32_t);
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) (accelZ_PID * 100.0f);
				binResBuf.size += sizeof(int32_t);
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) (accelZ_Range_PID * 100.0f);;
				break;
			default: //PID_REP_ERROR
				diag_log(dbg_level_warning, "Wrong PID_report_param setting %02x\n", (int)PID_report_param);
				// no break - explicit fallthrough to PID_REP_EULER
			case PID_REP_EULER:
				// this is to put int32_t into uint8_t buffer without data conversion
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) eulerData[0];
				binResBuf.size += sizeof(int32_t);
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) eulerData[1];
				binResBuf.size += sizeof(int32_t);
				*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (int32_t) eulerData[2];
				break;
			};
			// this is to put int32_t into uint8_t buffer without data conversion
			binResBuf.size += sizeof(int32_t);
			*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (uint32_t) dt;
			binResBuf.size += sizeof(int32_t);
			*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (uint32_t) BR;
			binResBuf.size += sizeof(int32_t);
			*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (uint32_t) BL;
			binResBuf.size += sizeof(int32_t);
			*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (uint32_t) FL;
			binResBuf.size += sizeof(int32_t);
			*(int32_t*) (&(binResBuf.dataBuf[binResBuf.size])) = (uint32_t) FR;
			binResBuf.size += sizeof(int32_t);
			if (binResBuf.size >= BIN_RES_BUF_MAX_LENGTH - 8 * sizeof(uint32_t)) {
				// binResBuf.size + 2*sizeof(uint16_t) = number of bytes in teh buffer + buffer size & sequence number = entire structure size
				CMD_RES_add(CMD_RES_ANALYSIS, binResBuf.size + 2 * sizeof(uint16_t), (uint8_t*) (&binResBuf), CMD_RES_queue_res_bin);
				binResBuf.size = 0;
				binResBuf.sequence++;
			}
		}
	} else {
		//=======================================================================
		// set motor PWM - MOTORS OFF
		if (rcthr > SERVO_MIN) {
			PWM_SetSpeed(Ch1, 0);
			PWM_SetSpeed(Ch2, 0);
			PWM_SetSpeed(Ch3, 0);
			PWM_SetSpeed(Ch4, 0);
		}

		//=======================================================================
		// reset yaw target so we maintain this on take-off
		yaw_target = eulerData[2];

		//=======================================================================
		// reset PID integrals whilst on the ground
		reset_I(&PID_Pitch_Rate);
		reset_I(&PID_Roll_Rate);
		reset_I(&PID_Yaw_Rate);
		reset_I(&PID_Pitch_Stab);
		reset_I(&PID_Roll_Stab);
		reset_I(&PID_Yaw_Stab);
	}
}
