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
	
	Lista<int> s;
	
	robot.mapThetasToServos(s);
	
	servoSignal = s[joint];
	
	setCalculated();
	resetDone();
}

void SetSingleJointAction::execute()
{
	std::string s;
	
	s = "D";
	s += std::to_string(joint);
	s += "\n";
	s += "E";
	s += std::to_string(servoSignal);
	s += "\n";
	
	arduinoPort << s;
	
	flags.reset(ARDUINO_MOV_FIN);
	
	setDone();
	resetCalculated();
}