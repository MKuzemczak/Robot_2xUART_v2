#include "Action.h"

SetSingleJointAction::SetSingleJointAction()
{
	setType(SINGLE);
}

SetSingleJointAction::~SetSingleJointAction()
{
	
}
	
void SetSingleJointAction::calculate(Robot & robot)
{
	robot.setThetaDeg(joint, angleDeg);
	
	robot.mapThetasToServos(servoSignals);
	
	setCalculated();
	resetDone();
}

void SetSingleJointAction::execute()
{
	std::string s;
	
	s = "B";
	for (int j = 0; j < servoSignals.size(); j++)
	{
		s += std::to_string(servoSignals[j]) + "\n";

	}
	s += "C";
	
	while (!flags.isSet(ARDUINO_MOV_FIN)) ;
	
	arduinoPort << s;
	
	flags.reset(ARDUINO_MOV_FIN);
	
	servoSignals.erase(0);
	
	setDone();
	resetCalculated();
}