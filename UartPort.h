#pragma once

#include <string>

#include "stm32f4xx_hal.h"

#include "PortBuffer.hpp"

class UartPort
{
	UART_HandleTypeDef * handlePtr;
	bool txCplt;
	
	uint8_t rcvChar;
	PortBuffer<20> buffer; 
	
public:
	void setSendingComplete()
	{
		txCplt = true;
	}
	
	void clearSendingComplete()
	{
		txCplt = false;
	}
	
	void setHandlePtr(UART_HandleTypeDef * h)
	{ 
		handlePtr = h;
	}
	
	bool sendingComplete()
	{
		return txCplt;
	}
	
	UART_HandleTypeDef * getHandlePtr()
	{
		return handlePtr;
	}
	
	void bufAppend(const char c)
	{
		buffer.add(c);
	}
	
	void rcvCharToBuffer()
	{
		char c = rcvChar;
		buffer.add(c);
	}
	
	PortBuffer<20> * getBuffer()
	{
		return &buffer;
	}
	
	uint8_t * getRcvByte()
	{
		return &rcvChar;
	}
	
	UartPort & operator <<(uint8_t c);
	UartPort & operator <<(const std::string s);
	UartPort & operator <<(uint8_t * c);
	UartPort & operator <<(const char c);
	UartPort & operator <<(const int i);
	UartPort & operator <<(const double d);
	UartPort & operator <<(const float f);
	
	template<int Buf_Size>
		UartPort & operator <<(const PortBuffer<Buf_Size> p);
};

template<int Buf_Size>
	UartPort & UartPort::operator <<(PortBuffer<Buf_Size> p)
	{
		*this << p.toString();
	}