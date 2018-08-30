#include "Action.h"

ConstTCPOrientAction::ConstTCPOrientAction(Eigen::Vector3d start, Eigen::Vector3d dest)
{
	setType(CONST_STRAIGHT);
	
	starting = start;
	destination = dest;
}

void ConstTCPOrientAction::calculate(Robot & robot)
{
	Lista<Eigen::Vector3d> path;
	Eigen::Vector3d orient;
	
	orient = robot.getJointLocation(robot.getDOF() - 1) - robot.getTCPlocation();
	
	
	lerp(path);
	

	
	for (int i = 0; i < path.size(); i++)
	{
		Eigen::Vector3d jointLoc;
		
		jointLoc = path[i] + orient;
		
		robot.set(0, robot.getRegJointsAmount() - 1, robot.getDOF() - 1, jointLoc);

		robot.set(robot.getRegJointsAmount(), robot.getDOF() - 1, robot.getDOF(), path[i]);

		Lista<int> s;
		
		robot.mapThetasToServos(s);
		
		pathInServoDegs.push_back(s);

	}
	
	setCalculated();
	resetDone();
}

void ConstTCPOrientAction::execute()
{
	std::string s;
	
	
	s = "B";
		
	for (int j = 0; j < pathInServoDegs[0].size(); j++)
	{
		s += std::to_string(pathInServoDegs[0][j]) + "\n";


	}
		
	s += 'C';	
		
	while (!flags.isSet(ARDUINO_MOV_FIN)) ;
		
	arduinoPort << s;
	pcPort << s << '\n';
	
	flags.reset(ARDUINO_MOV_FIN);
	pathInServoDegs.erase(0);
	
	if (pathInServoDegs.size() == 0)
	{
		setDone();
		resetCalculated();
	}
		
}


void ConstTCPOrientAction::lerp(Lista<Eigen::Vector3d> & path)
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

int ConstTCPOrientAction::size()
{
	return pathInServoDegs.size();
}
