#pragma once

#define NUM_OF_FLAGS 14

#define FLAG_OFFSET 'A'

#define PC_CONNECTED FLAG_OFFSET+0
#define PC_RCV_POINTS FLAG_OFFSET+1
#define PC_RCV_GRIP FLAG_OFFSET+2
#define PC_RCV_LINK_LENGTHS FLAG_OFFSET+3
#define PC_RCV_DOF FLAG_OFFSET+4
#define PC_RCV_PTS_AMT FLAG_OFFSET+5
#define PC_RCV_SINGLE_PT FLAG_OFFSET+6
#define PC_STOP FLAG_OFFSET+7
#define PC_LOOP FLAG_OFFSET+8
#define PC_SIM_INPUTS FLAG_OFFSET+9
#define PC_RCV_SIM_INPUTS FLAG_OFFSET+0x0A
#define PC_SAVE_RCV_VAL FLAG_OFFSET+0x0B
#define ARDUINO_CONNECTED FLAG_OFFSET+0x0C
#define ARDUINO_MOV_FIN FLAG_OFFSET+0x0D



class Flags
{
	char tab[NUM_OF_FLAGS];
	
	int currentlySetFlag;
	
public:
	Flags();
	
	void set(int i)
	{
		tab[i-FLAG_OFFSET] = 1;
		
		if(i != PC_CONNECTED && i != PC_LOOP && i != PC_SIM_INPUTS && i != PC_STOP)
			currentlySetFlag = i - FLAG_OFFSET;
	}
	
	void reset(int i)
	{
		tab[i - FLAG_OFFSET] = 0;
		
		if (i != PC_CONNECTED && i != PC_LOOP && i != PC_SIM_INPUTS && i != PC_STOP)
			currentlySetFlag = -1;
	}
	
	void toggle(int i)
	{
		if (tab[i - FLAG_OFFSET] == 1)
		{
			tab[i - FLAG_OFFSET] = 0;
			
			if (i != PC_CONNECTED && i != PC_LOOP && i != PC_SIM_INPUTS && i != PC_STOP)
				currentlySetFlag = -1;
		}
		else
		{
			tab[i - FLAG_OFFSET] = 1;
			
			if (i != PC_CONNECTED && i != PC_LOOP && i != PC_SIM_INPUTS && i != PC_STOP)
				currentlySetFlag = i - FLAG_OFFSET;
		}
	}
	
	int getCurrentRcvFlag()
	{
		return currentlySetFlag;
	}
	
	void resetCurrentRcvFlag()
	{
		reset(currentlySetFlag);
	}
	
	bool isSet(int i)
	{
		if (tab[i - FLAG_OFFSET] == 1)
			return true;
		
		return false;
			
	}
	
	
};

