/*
 * utils_List.h
 *
 *  Created on: 12.11.2016
 *      Author: Rafal
 */

#ifndef UTILS_LIST_H_
#define UTILS_LIST_H_

/* Includes ------------------------------------------------------------------*/
typedef struct _buffList {
	uint8_t lParam;
	uint16_t lLength;
	uint8_t *lBuff;
	struct _BuffList *lNext;
} buffList_Typedef;

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
buffList_Typedef* buff_List_add(uint8_t param, uint16_t length, uint8_t *buff, buffList_Typedef** root);
buffList_Typedef* buff_List_freeFirst(buffList_Typedef** root);
void buff_List_freeAll(buffList_Typedef** root);
uint16_t buff_List_count(buffList_Typedef** root);

#endif /* UTILS_LIST_H_ */
