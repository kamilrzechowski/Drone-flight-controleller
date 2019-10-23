/*
 * utils_LEDs.c
 *
 *  Created on: 15 sty 2016
 *      Author: r.rzechowski
 */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "utils_list.h"
#include "trace.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  buff_add adds element to list
  * @param param - additional parameter
  * @param legth - buffer size
  * @param buff  - pointer to parameter buffer
  * @retval element added
  */
buffList_Typedef* buff_List_add(uint8_t param, uint16_t length, uint8_t *buff, buffList_Typedef** root) {
	buffList_Typedef *elem = *root;

	if(elem == NULL) {
		elem = malloc(sizeof(buffList_Typedef));
	} else {
		while (elem->lNext != NULL) {
			elem = elem->lNext;
		}
		elem->lNext = malloc(sizeof(buffList_Typedef));
		elem = elem->lNext;
	}
	if (elem == NULL) {
		diag_log(dbg_level_warning, "Message warning: can not allocate memory on the list %02x\n", param);
		return NULL;
	}

	elem->lParam = param;
	elem->lLength = length;
	elem->lNext = NULL;

	if (length > 0) {
		elem->lBuff = malloc(length);
		if (elem->lBuff != NULL) {
			memcpy(elem->lBuff, buff, length);
		}
	} else {
		elem->lBuff = NULL;
	}

	if (*root == NULL) {
		*root = elem;
	}
	return elem;
}

/**
 * @brief  buff_List_freeFirst
 *         The function frees the memory allocated for buffer.
 * @param  root : list root pointer
 * @retval  new elemen of list root
 */
buffList_Typedef* buff_List_freeFirst(buffList_Typedef** root) {

	if (*root != NULL) {

		buffList_Typedef *next = (*root)->lNext;

		if ((*root)->lBuff != NULL) {
			free((*root)->lBuff);
		}
		free(*root);
		*root = next;
	}

	return *root;
}

/**
 * @brief  buff_List_freeAll
 *         The function free all memory alocated to the list.
 * @param  none
 * @retval  0 = empty; 1 = not empty
 */
void buff_List_freeAll(buffList_Typedef** root) {
	buffList_Typedef *next;
	while (*root) {
		next = (*root)->lNext;
		if ((*root)->lBuff != NULL) {
			free((*root)->lBuff);
		}
		free(*root);
		*root = next;
	}
}


/**
 * @brief  BTC_bufIn_size
 *         The function adds element to the bufIn as last element.
 * @param  BTC_buff: buffer
 * @retval  none
 */
uint16_t buff_List_count(buffList_Typedef** root) {
	uint16_t count = 0;
	buffList_Typedef *elem = *root;

	while (elem) {
		elem = elem->lNext;
		count++;
	}
	return count;
}
