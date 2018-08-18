#include "Action.h"

Action::Action()
{
}


Action::~Action()
{
}


StraightLineMovAction::StraightLineMovAction(Eigen::Vector3d start, Eigen::Vector3d dest)
{
	setType(STRAIGHT_LINE);
	
	starting = start;
	destination = dest;
}

void StraightLineMovAction::calculate(Robot & robot)
{
	Lista<Eigen::Vector3d> path;
	
	lerp(path);
	
	for (int i = 0; i < path.size(); i++)
	{
		robot.setRegional(path[i]);
		
		Lista<int> s;
			
		for (int j = 0; j < robot.getDOF(); j++)
		{
			s.push_back(map(robot.getJointThetaRad(j), 0, PI, (double)robot.getServoMin(j), (double)robot.getServoMax(j)));
		}
		
		pathInServoDegs.push_back(s);
	}
}

void StraightLineMovAction::execute()
{
	pcPort << 'P';
	
	for (int i = 0; i < pathInServoDegs.size(); i++)
	{
		for (int j = 0; j < pathInServoDegs[i].size(); j++)
			pcPort << pathInServoDegs[i][j] << " ";
		
		pcPort << '\n';
	}
	
	pcPort << 'P';
}
	
void StraightLineMovAction::lerp(Lista<Eigen::Vector3d> & path)
{
	Eigen::Vector3d v = destination - starting;
	
	double vmod = sqrt(pow(v[0], 2) + pow(v[1], 2) + pow(v[2], 2));
	
	double number = vmod / 10;
	
	int loops = (int)number;
	
	v.normalize();
	
	v *= 10;
	
	
	for (int i = 1; i < loops; i++)
	{
		path.push_back(starting + i*v);
	}
	
	path.push_back(destination);
}

double map(double x, double in_min, double in_max, double out_min, double out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}