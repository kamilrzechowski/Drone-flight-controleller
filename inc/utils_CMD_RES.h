/*
 * utils_CMD_RES.h
 *
 *  Created on: 15 maj 2016
 *      Author: r.rzechowski
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef UTILS_CMD_H_
#define UTILS_CMD_H_
/* Includes ------------------------------------------------------------------*/
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic warning "-Wunused-parameter"
#include <btstack/hci.h>

/* Global defines ------------------------------------------------------------*/
#define CMD_RES_BUF_MAX_LENGTH (HCI_ACL_PAYLOAD_SIZE - L2CAP_HEADER_SIZE - 5 - 2)
#define BIN_RES_BUF_MAX_LENGTH (CMD_RES_BUF_MAX_LENGTH - 3*sizeof(uint16_t))	// size, sequence & header

/* Exported types ------------------------------------------------------------*/
typedef enum {
	CMD_RES_queue_cmd,
	CMD_RES_queue_res_str,
	CMD_RES_queue_res_bin
} CMD_RES_queue_t;

typedef struct _CMD_RES_struct_t {
	uint8_t cmd;
	uint16_t param_size;
	CMD_RES_queue_t queueType;
	uint8_t *param;
	struct _CMD_RES_struct_t *next;
} CMD_RES_struct_t;

typedef struct {
	uint16_t size; 			// size of the message to be send = number of bytes in dataBuf
	uint16_t sequence;		// the sequence is for monitoring only. It allows receiving client to track how many messages were lost due to performance of sending
	uint8_t dataBuf[BIN_RES_BUF_MAX_LENGTH]; // this is the real buffer with the data to be send
} binaryBTCbuffer_t;
/* Exported constants --------------------------------------------------------*/
// CMD_RES_BUF_MAX_LENGTH is calculated in following way
// HCI_ACL_PAYLOAD_SIZE - L2CAP_HEADER_SIZE = l2cap_max_mtu()
// 5 = rfcomm_max_frame_size_for_l2cap_mtu() ;; Assume RFCOMM header without credits and 2 byte (14 bit) length field
// 2 = for end of line marks

// Incoming command
#define CMD_RES_setParam		'0' // general purpose command to set device parameter
#define CMD_RES_pulling			'1' // received pulling for MPU-9250 or LIS3DSH
#define CMD_RES_settingsReq		'2' // requests settings from the device
#define CMD_RES_motorSpeed		'3' // requests to set motor speed
#define CMD_RES_setMPUreport	'4' // sets reporting scope of MPU9250 data
#define CMD_RES_setExpParam		'5' // sets expected parameters values for PID controller
#define CMD_RES_setPIDSettings	'6' // sets expected parameters values for PID controller internal settings
#define CMD_RES_pingEcho		'7' // Ping the device to receive echo back
#define CMD_RES_VoidCmd			13	// this is '/r' character coming wrongly from the interface and should be skied

// Outgoing events
#define CMD_RES_respERR			'0' // ID= 48 response notifying command was rejected as an result of an error
#define CMD_RES_respOK			'1' // ID= 49 response confirming that command was processed successfully
#define CMD_RES_logDEBUG		'2' // ID= 50 logs debug messages
#define CMD_RES_respSettings	'3'	// ID= 51 response to notify the values of current settings
#define CMD_RES_LIS3DSH			'4' // ID= 52 response with LIS3DSH data
#define CMD_RES_MPU9250			'5' // ID= 53 response with MPU9250 data
#define CMD_RES_HCSR04			'6' // ID= 54 response with HCSR04 data
#define CMD_RES_ANALYSIS		'7' // ID= 55 response with data to plot in Analysis tab
#define CMD_RES_ECHO			'8' // ID= 56 echo response

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
CMD_RES_struct_t* CMD_RES_add(uint8_t cmd, uint16_t param_size, uint8_t *param, CMD_RES_queue_t queue_type);
CMD_RES_struct_t* CMD_RES_get_first(CMD_RES_queue_t queue_type);
CMD_RES_struct_t* CMD_RES_free_firest(CMD_RES_queue_t queue_type);
void CMD_RES_free_all();
uint16_t CMD_RES_printf(CMD_RES_struct_t *cmdElem, const char* format, ...);

#endif /* UTILS_CMD_H_ */
