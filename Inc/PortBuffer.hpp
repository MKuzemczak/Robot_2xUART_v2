#pragma once

#include <string>

typedef enum 
{	
	INDEX_OUT_OF_BORDER,
	BUFFER_FULL
} BufferExceptions;

template <int Buf_Size>
class PortBuffer
{
	char tab[Buf_Size];
	int currentLength;
	
public:
	PortBuffer();
	~PortBuffer();
	
	int size()
	{
		return Buf_Size;
	}
	
	int getCurrentLength()
	{
		return currentLength;
	}
	
	void clear();
	
	char & operator [](const int index) ;
	char & at(const int index);
	
	void add(const char c);
	
	int toInt();
	double toDouble();
	void toCharTable(char * table);
	std::string toString();
};

template<int Buf_Size>
	PortBuffer<Buf_Size>::PortBuffer()
	{
		currentLength = 0;
	
		for (char c : tab)
			c = 0;
	}

template<int Buf_Size>
	PortBuffer<Buf_Size>::~PortBuffer()
	{
	}

template<int Buf_Size>
	void PortBuffer<Buf_Size>::clear()
	{
		currentLength = 0;
		
		for (char c : tab)
			c = '\0';
	}

template<int Buf_Size>
	char & PortBuffer<Buf_Size>::operator [](const int index) 
	{
		return tab[index];	
	}

template<int Buf_Size>
	char & PortBuffer<Buf_Size>::at(const int index)
	{
		if (index >= Buf_Size || index < 0)
			throw INDEX_OUT_OF_BORDER;
		
		return tab[index];
	}

template<int Buf_Size>
	void PortBuffer<Buf_Size>::add(const char c)
	{
		tab[currentLength++] = c;
		tab[currentLength] = '\0';
	}

template<int Buf_Size>
	int PortBuffer<Buf_Size>::toInt()
	{
		return atoi(tab);
	}

template<int Buf_Size>
	double PortBuffer<Buf_Size>::toDouble()
	{
		return atof(tab);
	}

template<int Buf_Size>
	void PortBuffer<Buf_Size>::toCharTable(char * table)
	{
		for (int i = 0; i < currentLength + 1; i++)
			table[i] = tab[i];
	}
	
template<int Buf_Size>
	std::string PortBuffer<Buf_Size>::toString()
	{
		std::string s;
		
		for (char c : tab)
			s.push_back(c);
		
		return s;
	}