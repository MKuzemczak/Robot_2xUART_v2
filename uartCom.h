#pragma once

#include "stm32f4xx_hal.h"

#include "UartPort.h"
#include "comFlags.h"

#define ARDUINO_PORT USART1
#define PC_PORT USART2

extern UartPort pcPort;
extern UartPort arduinoPort;

extern Flags flags;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;



// init HAL uart handles and uartPort objects collectively
bool portInit();
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);


void MX_USART2_UART_Init(void);
void MX_USART1_UART_Init(void);
