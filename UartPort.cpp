#include "UartPort.h"


UartPort & UartPort::operator <<(uint8_t c)
{
	clearSendingComplete();
	
	HAL_UART_Transmit_IT(handlePtr, &c, 1);
	
	while (!txCplt) ;
	
	return *this;
}

UartPort & UartPort::operator <<(const std::string s)
{
	clearSendingComplete();
	
	int size = s.length();
	
	uint8_t buffer[size];
	
	for (int i = 0; i < size; i++)
		buffer[i] = s[i];
	
	
	HAL_UART_Transmit_IT(handlePtr, buffer, size);
	
	while (!txCplt) ;
	
	return *this;
}

UartPort & UartPort::operator <<(uint8_t * c)
{
	clearSendingComplete();
	
	int size = 0;
	
	while (c[size] != '\0')
		size++;
	
	HAL_UART_Transmit_IT(handlePtr, c, size);
	
	while (!txCplt) ;
	
	return *this;	
}

UartPort & UartPort::operator <<(const char c)
{
	clearSendingComplete();
	
	uint8_t ch = c;
	
	HAL_UART_Transmit_IT(handlePtr, &ch, 1);
	
	while (!txCplt) ;
	
	return *this;
}

UartPort & UartPort::operator <<(const int i)
{
	clearSendingComplete();
	
	char buf0[20];
	
	int size = sprintf(buf0, "%d", i);
	
	uint8_t buf1[size];
	
	for (int j = 0; j < size; j++)
		buf1[j] = buf0[j];
	
	HAL_UART_Transmit_IT(handlePtr, buf1, size);
	
	while (!txCplt) ;
	
	return *this;	
}
UartPort & UartPort::operator <<(const double d)
{
	clearSendingComplete();
	
	char buf0[20];
	
	int size = sprintf(buf0, "%.2f", d);
	
	uint8_t buf1[size];
	
	for (int j = 0; j < size; j++)
		buf1[j] = buf0[j];
	
	HAL_UART_Transmit_IT(handlePtr, buf1, size);
	
	while (!txCplt) ;
	
	return *this;	
}

UartPort & UartPort::operator <<(const float f)
{
	clearSendingComplete();
	
	char buf0[20];
	
	int size = sprintf(buf0, "%f", f);
	
	uint8_t buf1[size];
	
	for (int j = 0; j < size; j++)
		buf1[j] = buf0[j];
	
	HAL_UART_Transmit_IT(handlePtr, buf1, size);
	
	while (!txCplt) ;
	
	return *this;	
}

