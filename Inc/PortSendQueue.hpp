#pragma once

/**************************************************
 *	Prawdopodobnie niepotrzebne
 ******************************************************/

#include <vector>

class PortSendQueue : public std::vector < std::vector <char> >
{
	
public:
	PortSendQueue();
	~PortSendQueue();
	
};

