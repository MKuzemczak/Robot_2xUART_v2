#include "comFlags.h"

Flags::Flags()
{
	for (int i = 0; i < NUM_OF_FLAGS; i++)
		tab[i] = 0;
	
	tab[ARDUINO_MOV_FIN] = 1;
}