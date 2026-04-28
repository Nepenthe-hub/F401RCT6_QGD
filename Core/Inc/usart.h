/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */
#define RX_BUF_SIZE     256
/* USER CODE END Private defines */

void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
extern uint8_t  rx_buf[RX_BUF_SIZE];
extern uint16_t rx_len;
extern uint8_t  rx_flag;

void USART6_SendByte(uint8_t data);
void USART6_SendString(const char *str);
void USART6_SendBuffer(uint8_t *buf, uint16_t len);
void USART6_SendHex(uint8_t *buf, uint16_t len);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

