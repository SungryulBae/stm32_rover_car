/*
 * callcmd.h
 *
 *  Created on: Nov 3, 2021
 *      Author: bae
 */

#ifndef INC_CALLCMD_H_
#define INC_CALLCMD_H_

#include <stdio.h>
#include "string.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>

#define byte unsigned char
#define uint8 unsigned char
#define word unsigned short
#define lword unsigned int
#define int8u unsigned char
#define u8 unsigned char
#define u16 unsigned short
#define u32 unsigned int
#define DD_SUCCESS YES
#define FALSE -1
#define YES 1
#define NO 0

/*The Rx task will block on the Rx queue for a long period*/

#define COM1_DEBUG 0x00
#define printf SMprintf
uint16_t Uart3_DeQueue(void);
bool Uart3_Is_Empty(void);
extern UART_HandleTypeDef huart3; // access huart3 instance

extern UART_HandleTypeDef huart2;
bool Uart2_Is_Empty(void);
uint16_t Uart2_DeQueue(void);
void put_uart2(unsigned char data);



#endif /* INC_CALLCMD_H_ */
