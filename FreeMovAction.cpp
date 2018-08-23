#include "Action.h"

FreeMovAction::FreeMovAction(Eigen::Vector3d dest)
{
	destination = dest;
	setType(FREE);
}

FreeMovAction::~FreeMovAction()
{
	
}

void FreeMovAction::calculate(Robot & robot)
{
	robot.setRegional(destination);
	
	robot.mapThetasToServos(destInServoDegs);
}

void FreeMovAction::execute()
{
	pcPort << 'P';
	
	
	pcPort << 'P';
}
	

