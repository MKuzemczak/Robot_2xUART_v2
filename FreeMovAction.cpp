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
	
	for (int j = 0; j < robot.getDOF(); j++)
	{
		destInServoDegs.push_back(map(robot.getJointThetaRad(j), 0, PI, (double)robot.getServoMin(j), (double)robot.getServoMax(j)));
	}
}

void FreeMovAction::execute()
{
	pcPort << 'P';
	
	
	pcPort << 'P';
}
	

