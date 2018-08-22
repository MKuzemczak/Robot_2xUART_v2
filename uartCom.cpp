#include "uartCom.h"


UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

UartPort pcPort;
UartPort arduinoPort;

Flags flags;

bool portInit()
{
	MX_USART2_UART_Init();
	MX_USART1_UART_Init();

	pcPort.setHandlePtr(&huart2);
	arduinoPort.setHandlePtr(&huart1);
	
	pcPort.setSendingComplete();
	arduinoPort.setSendingComplete();
	
	HAL_UART_Receive_IT(pcPort.getHandlePtr(), pcPort.getRcvByte(), 1);
	HAL_UART_Receive_IT(arduinoPort.getHandlePtr(), arduinoPort.getRcvByte(), 1);
	
	return true;
}

/*
 *	interrupt called at end of transmitting
 *
 *	Nie wysylac nic z tej funkcji!!!!
 **/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	
	if(huart->Instance == PC_PORT) {
		pcPort.setSendingComplete();
	} else if(huart->Instance == ARDUINO_PORT) {
		arduinoPort.setSendingComplete();
	}
}

/*
 *	interrupt called at end of receiving
 *	
 *	Nie wysylac nic z tej funkcji!!!!
 **/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	
	if(huart->Instance == PC_PORT) {
		
		char rcvByte = *(pcPort.getRcvByte());  // byte received from the PC UART port
		
		if ((rcvByte >= FLAG_OFFSET && rcvByte < NUM_OF_FLAGS + FLAG_OFFSET) &&
			rcvByte != PC_LOOP && rcvByte != PC_SIM_INPUTS)
		{
			flags.set(rcvByte);
		}
		else if (rcvByte == PC_LOOP || rcvByte == PC_SIM_INPUTS)
		{
			flags.toggle(rcvByte);
		}
		else if (rcvByte == '\n')
		{
			flags.set(PC_SAVE_RCV_VAL);
		}
		else if ((rcvByte >= '0' && rcvByte <= '9') || rcvByte == '.')
		{
			pcPort.rcvCharToBuffer();
		}
		
		HAL_UART_Receive_IT(pcPort.getHandlePtr(), pcPort.getRcvByte(), 1);	// nasluchuj dalej
	} else if(huart->Instance == ARDUINO_PORT) {
		
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		char rcvByte = *(arduinoPort.getRcvByte());
		if (rcvByte >= FLAG_OFFSET && rcvByte < NUM_OF_FLAGS + FLAG_OFFSET)
		{
			flags.set(rcvByte);
		}
		else if (rcvByte == '\n')
		{
			pcPort << *(arduinoPort.getBuffer());
		}
		else if ((rcvByte >= '0' && rcvByte <= '9') || rcvByte == '.')
		{
			arduinoPort.rcvCharToBuffer();
		}
		
		HAL_UART_Receive_IT(arduinoPort.getHandlePtr(), arduinoPort.getRcvByte(), 1);
	}
}


/* USART1 init function */
void MX_USART1_UART_Init(void)
{

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}