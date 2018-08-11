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

void StraightLineMovAction::calculate()
{
	Lista<Eigen::Vector3d> path;
	
	lerp(path);
	
	
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