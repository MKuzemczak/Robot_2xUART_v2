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
			
		for (int j = 0; j < robot.getDOF(); j++)
		{
			pcPort << "void StraightLineMovAction::calculate(Robot & robot): reprezentacja serw w Joint do zrobienia, ten kod nie dzia³a poprawnie!\n";
			s.push_back(map(robot.getJointThetaRad(j),
				(double)robot.getConversionMin(j)*DEG_TO_RAD,
				(double)robot.getConversionMax(j)*DEG_TO_RAD,
				(double)robot.getServoMin(j),
				(double)robot.getServoMax(j)));
			
			
#ifdef DEBUG_STRAIGHT_LINE_ACTION
			pcPort << "Straight Line Action, calculate(), koniec petli wewnetrznej\n";	  
#endif // DEBUG

		}
		
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

double map(double x, double in_min, double in_max, double out_min, double out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}