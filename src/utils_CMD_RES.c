/*
 * utils_CMD_RES.c
 *
 *  Created on: 15 maj 2016
 *      Author: r.rzechowski
 */


/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <btstack_debug.h>
#include "utils_CMD_RES.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MAX_ELEMS_IN_QUEUE 5
/* Private variables ---------------------------------------------------------*/
static CMD_RES_struct_t *cmd_list_root = NULL;
static CMD_RES_struct_t *res_list_root = NULL;
/* Exported variables --------------------------------------------------------*/
extern uint16_t rfcomm_channel_id; // this variable indicates, if RFcomm connection is established, and data can be send
uint8_t cmdResBuf[CMD_RES_BUF_MAX_LENGTH]; // this is the buffer used to construct text massages send via bluetooth

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  CMD_add adds element to command list
  * @param cmd - command ID
  * @param param_size - command parameters' size
  * @param param - pointer to parameter buffer
  * @retval element added
  */
CMD_RES_struct_t* CMD_RES_add(uint8_t cmd, uint16_t param_size, uint8_t *param, CMD_RES_queue_t queue_type) {
	CMD_RES_struct_t *elem = (queue_type == CMD_RES_queue_cmd) ? cmd_list_root : res_list_root;

	uint16_t elemsInQueue = 0;

	if (rfcomm_channel_id == 0) { // no connection
		return NULL;
	}

	if(elem == NULL) {
		elem = malloc(sizeof(CMD_RES_struct_t));
	} else {
		while (elem->next != NULL) {
			elem = elem->next;
			elemsInQueue ++;
		}
		if (elemsInQueue > MAX_ELEMS_IN_QUEUE) {
			log_warning("Message warning: to many elements in the queue - %04x. Message skipped.\n", elemsInQueue);
			return NULL;
		}
		elem->next = malloc(sizeof(CMD_RES_struct_t));
		elem = elem->next;
	}
	if (elem == NULL) {
		log_warning("Message warning: can not allocate memory [%04x]. Message skipped.\n", elemsInQueue);
		return NULL;
	}

	elem->cmd = cmd;
	elem->param_size = param_size + 1; //zero will be added at the end
	elem->queueType = queue_type;
	elem->next = NULL;

	if (param_size > 0) {
		elem->param = malloc(param_size);
		if (elem->param == NULL) {
			free(elem);
			elem = NULL;
		} else {
			memcpy(elem->param, param, param_size);
			elem->param[param_size] = 0; // add 0 at the end
		}
	} else {
		elem->param = NULL;
	}

	if (queue_type == CMD_RES_queue_cmd) {
		if (cmd_list_root == NULL) {
			cmd_list_root = elem;
		}
	} else {
		if (res_list_root == NULL) {
			res_list_root = elem;
		}
	}
	return elem;
}

/**
  * @brief  CMD_get_first gets pointer to the first element on the list
  * @param none
  * @retval pointer to the first element on the list
  */
CMD_RES_struct_t* CMD_RES_get_first(CMD_RES_queue_t queue_type) {
	return (queue_type == CMD_RES_queue_cmd) ? cmd_list_root : res_list_root;
}

/**
  * @brief  CMD_get_first gets pointer to the first element on the list
  * @param none
  * @retval none
  */
CMD_RES_struct_t* CMD_RES_free_first(CMD_RES_queue_t queue_type) {
	CMD_RES_struct_t *elem = NULL;

	if (queue_type == CMD_RES_queue_cmd) {
		if (cmd_list_root == NULL) return NULL;
		elem = cmd_list_root->next;
		if (cmd_list_root->param) {
			free(cmd_list_root->param);
		}
		free(cmd_list_root);
		cmd_list_root = elem;
	} else {
		if (res_list_root == NULL) return NULL;
		elem = res_list_root->next;
		if (res_list_root->param) {
			free(res_list_root->param);
		}
		free(res_list_root);
		res_list_root = elem;
	}
	return elem;
}

void CMD_RES_free_all() {
	while (CMD_RES_free_first(CMD_RES_queue_cmd) != NULL) {;}
	while (CMD_RES_free_first(CMD_RES_queue_res_str) != NULL) {;}
	while (CMD_RES_free_first(CMD_RES_queue_res_bin) != NULL) {;}
	cmd_list_root = NULL;
	res_list_root = NULL;
}

uint16_t CMD_RES_printf(CMD_RES_struct_t *cmdElem, const char* format, ...) {
	uint16_t cmdResLen, cmdBytesToCopy;
	va_list ap;

	va_start (ap, format);
	cmdResLen = vsnprintf((char*)cmdResBuf, CMD_RES_BUF_MAX_LENGTH, format, ap);
	va_end(ap);

	if (cmdResLen > CMD_RES_BUF_MAX_LENGTH) {
		cmdBytesToCopy = 0;
		cmdResLen = CMD_RES_BUF_MAX_LENGTH;
		cmdElem->param_size = CMD_RES_BUF_MAX_LENGTH;
	} else if (cmdElem->param_size + cmdResLen > CMD_RES_BUF_MAX_LENGTH) {
		cmdBytesToCopy = CMD_RES_BUF_MAX_LENGTH - cmdResLen;
		cmdElem->param_size = CMD_RES_BUF_MAX_LENGTH;
	} else {
		cmdBytesToCopy = cmdElem->param_size;
		cmdElem->param_size = cmdElem->param_size + cmdResLen;
	}
	if (cmdBytesToCopy > 0 && cmdElem->param_size > 0 && cmdElem->param!=NULL) {
		memcpy(cmdResBuf+cmdResLen, cmdElem->param, cmdBytesToCopy);
	}
	return cmdElem->param_size;
}
