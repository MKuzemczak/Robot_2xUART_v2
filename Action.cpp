#include "Action.h"

BaseAction::BaseAction()
{
	calculated = false;
	done = false;
}

void BaseAction::setDone()
{
	done = true;
}
void BaseAction::resetDone()
{
	done = false;
}
bool BaseAction::isDone()
{
	return done;
}

void BaseAction::setType(ActionType t)
{
	type = t;
}
	
void BaseAction::setCalculated()
{
	calculated = true;
}
	
bool BaseAction::isCalculated()
{
	return calculated;
}
	
void BaseAction::resetCalculated()
{
	calculated = false;
}