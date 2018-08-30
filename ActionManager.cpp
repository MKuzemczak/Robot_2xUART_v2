#include "ActionManager.h"



ActionManager::ActionManager()
{
	stepInProgress = false;
	checkCalculations = false;
	started = false;
	actionCntr = 0;
	robotPtr = NULL;
}


ActionManager::~ActionManager()
{
}

bool ActionManager::start()
{
	if (actions.size() == 0)
	{
		pcPort << "Error: ActionManager::start() : actions.size() == 0\n";
		return false;
	}
	if (robotPtr == NULL)
	{
		pcPort << "Error: ActionManager::start() : robotPtr == NULL\n";
		return false;
	}
	
	actions[0]->calculate(*robotPtr);
	started = true;
	checkCalculations = true;
	
	return true;
}

/*
 * Ka¿de wywo³anie metody BaseAction::execute() sprawia ¿e akcja wykonuje czêœæ swojej roboty.
 *	Jeœli akcja uzna, ¿e skoñczy³a, ActionManager::nextStep() przechodzi do nastepnej akcji.
 **/
void ActionManager::nextStep()
{
	if (!stepInProgress)
	{
	
		stepInProgress = true;

		#ifdef DEBUG_ACTION_MANAGER
		//pcPort << "ActionManager::nextStep() : poczatek\nactions.size() == " << (int)actions.size() << ", actionCntr == " << actionCntr << '\n';
		#endif // DEBUG_ACTION_MANAGER

	
		if(actions.size() > 0 && actions[actionCntr]->size() > 0)
		{
			actions[actionCntr]->execute();

			pcPort << "action no. " << actionCntr << '\n';
#ifdef DEBUG_ACTION_MANAGER
			pcPort << "ActionManager::nextStep() : execute(), actionCntr == " << actionCntr << '\n';
#endif // DEBUG_ACTION_MANAGER

		}
			
	
		if (actions[actionCntr]->isDone() && actions[((actionCntr < actions.size() - 1) ? (actionCntr + 1) : 0)]->isCalculated())
		{
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			actionCntr++;
			checkCalculations = true;
		}
	
		if (flags.isSet(PC_LOOP) && actionCntr == actions.size())
			actionCntr = 0;

		#ifdef DEBUG_ACTION_MANAGER
		//pcPort << "ActionManager::execute() : koniec\n";
		#endif // DEBUG_ACTION_MANAGER

		#ifdef DEBUG_ACTION_MANAGER
		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);			  
		#endif // DEBUG_ACTION_MANAGER

			stepInProgress = false;
	}
}

void ActionManager::calculations()
{
#ifdef DEBUG_ACTION_MANAGER
	pcPort << "ActionManager::calculations() : actionCntr + 1 == " << actionCntr + 1 << '\n';				  
#endif // DEBUG_ACTION_MANAGER
	
	if (actionCntr < actions.size() - 1)
	{
		if (!actions[actionCntr + 1]->isCalculated())
		{
			actions[actionCntr + 1]->calculate(*robotPtr);
			
#ifdef DEBUG_ACTION_MANAGER
			pcPort << "ActionManager::calculations() : calc + 1\n";				  
#endif // DEBUG_ACTION_MANAGER

		}
			
	}
	else if (flags.isSet(PC_LOOP))
	{
		if (!actions[0]->isCalculated())
		{
			actions[0]->calculate(*robotPtr);
			
#ifdef DEBUG_ACTION_MANAGER
			pcPort << "ActionManager::calculations() : calc 0\n";				  
#endif // DEBUG_ACTION_MANAGER
			
		}
	}
	
	checkCalculations = false;
	
}