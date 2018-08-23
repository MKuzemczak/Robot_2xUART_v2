#include "Action.h"

//#define DEBUG_STRAIGHT_LINE_ACTION

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
	
#ifdef DEBUG_STRAIGHT_LINE_ACTION
	pcPort << "Straight Line Action, calculate(), wykonano lerp\n";	  
#endif // DEBUG

	
	for (int i = 0; i < path.size(); i++)
	{
#ifdef DEBUG_STRAIGHT_LINE_ACTION
		pcPort << "Straight Line Action, calculate(), poczatek petli\n";	  
#endif // DEBUG

		robot.setRegional(path[i]);

#ifdef DEBUG_STRAIGHT_LINE_ACTION
		pcPort << "Straight Line Action, calculate(), ustawiono czesc regionalna robota\n";	  
#endif // DEBUG

		Lista<int> s;
		
		robot.mapThetasToServos(s);
		
#ifdef DEBUG_STRAIGHT_LINE_ACTION
		pcPort << "straightLineMovAction::calculate() : zmapowano katy na serwa:\n";
		
		for (int i = 0; i < s.size(); i++)
		{
			pcPort << s[i] << ' ';
		}
		
		pcPort << "\n\n";
#endif // DEBUG
		
		pathInServoDegs.push_back(s);
		
#ifdef DEBUG_STRAIGHT_LINE_ACTION
		pcPort << "Straight Line Action, calculate(), koniec petli\n";	  
#endif // DEBUG

	}
}

void StraightLineMovAction::execute()
{
	std::string s;
	
	for (int i = 0; i < pathInServoDegs.size(); i++)
	{
		s = "B";
		
		for (int j = 0; j < pathInServoDegs[i].size(); j++)
		{
			s += std::to_string(pathInServoDegs[i][j]) + "\n";
			
			pcPort << pathInServoDegs[i][j] << " ";
		}
		
		s += 'C';	
		
		pcPort << '\n';
		
		//while (!flags.isSet(ARDUINO_MOV_FIN)) ;
		
		//arduinoPort << s;
		flags.reset(ARDUINO_MOV_FIN);
			
	}
	
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

