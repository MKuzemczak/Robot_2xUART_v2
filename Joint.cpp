#include "Joint.h"



Joint::Joint()
{
	alpha = 0;
	Theta = 0;
	aLength = 0;
	dLength = 0;
	
	DHmatrix = Eigen::Matrix4d::Identity();
}

Joint::Joint(double a, double al, double dl)
{
	alpha = a;
	Theta = 0;
	aLength = al;
	dLength = dl;
	
	DHmatrix = Eigen::Matrix4d::Identity();
}


Joint::~Joint()
{
}


/*
 * This method maps joint's current Theta angle to all its servos' signals
 **/
void Joint::mapThetaToServo(Lista<int> & lista)
{
	for (int i = 0; i < servosMinMax.size(); i++)
	{
		int a = (int)map(Theta, 
			(double)angleConversionMinMaxDeg[0]*DEG_TO_RAD, 
			(double)angleConversionMinMaxDeg[1]*DEG_TO_RAD, 
			(double)servosMinMax[i][0], 
			(double)servosMinMax[i][1]);
		
#ifdef DEBUG_JOINT
		pcPort << "Joint::Theta == " << Theta << "\n";
		pcPort << "Joint::mapThetaToServo() : a == " << a << "\n\n";
#endif // DEBUG_JOINT

		
		
		lista.push_back(a);
	}
}

///////////////////////////////////////////////////////////////////////////////////// setters & getters

int Joint::getConstructionMinDeg()
{
	return angleConstructionMinMaxDeg[0];
}

int Joint::getConstructionMaxDeg()
{
	return angleConstructionMinMaxDeg[1];
}

int Joint::getConversionMinDeg()
{
	return angleConversionMinMaxDeg[0];
}

int Joint::getConversionMaxDeg()
{
	return angleConversionMinMaxDeg[1];
}

int Joint::getServoAmount()
{
	return servosMinMax.size();
}

void Joint::setAlpha(double a)
{
	alpha = a;
			
}
	
double Joint::getAlpha()
{
	return alpha;
}
	
// aktualizuj kat Theta
void Joint::setTheta(double T)
{
	Theta = T;
}
	
double Joint::getTheta()
{
	return Theta;
}
	
void Joint::setaLength(double l)
{
	aLength = l;
}
	
double Joint::getaLength()
{
	return aLength;
}
	
void Joint::setdLength(double l)
{
	dLength = l;
}
	
double Joint::getdLength()
{
	return dLength;
}
	
void Joint::setLocation(Eigen::Vector3d & v)
{
	locationInGlobal = v;
}
	
Eigen::Vector3d & Joint::getLocation()
{
	return locationInGlobal;
}
	
void Joint::setZinGlobal(Eigen::Vector3d & v)
{
	ZaxisInGlobal = v;
}
	
Eigen::Vector3d & Joint::getZinGlobal()
{
	return ZaxisInGlobal;
}
	
Eigen::Matrix4d & Joint::getDHmatrix()
{
	return DHmatrix;
}
	
void Joint::addServoMinMax(int min, int max)
{
	Eigen::Vector2i v;
		
	v << min, max;
		
		
#ifdef DEBUG_JOINT
	pcPort << "Joint::addServoMinMax(" << min << ", " << max << ");\n\n";				  
#endif // DEBUG_JOINT
		
	servosMinMax.push_back(v);
}
	
void Joint::setServoMinMax(int servo, int min, int max)
{
	if (servo < 0 || servo >= servosMinMax.size())
		pcPort << "Joint::setServoMinMax(...): Error: Servo index out of bounds\n";
	else
	{
		servosMinMax[servo][0] = min;
		servosMinMax[servo][1] = max;
	}
		

}
	
void Joint::setConversionMinMaxDeg(int min, int max)
{
	angleConversionMinMaxDeg << min, max;
		
#ifdef DEBUG_JOINT
	pcPort << "Joint::setConversionMinMaxDeg(" << min << ", " << max << ");\n\n";				  
#endif // DEBUG_JOINT
}
	
void Joint::setConstructionMinMaxDeg(int min, int max)
{
	angleConstructionMinMaxDeg << min, max;
		
#ifdef DEBUG_JOINT
	pcPort << "Joint::setConstructionMinMaxDeg(" << min << ", " << max << ");\n\n";				  
#endif // DEBUG_JOINT
}
	
Eigen::Vector2i & Joint::getConstructionMinMaxDeg()
{
	return angleConstructionMinMaxDeg;
}

//////////////////////////////////////////////////////////////////////////////// !setters & getters & adders


/////////////////////////////////////////////// others
double map(double x, double in_min, double in_max, double out_min, double out_max) {
	
#ifdef DEBUG_JOINT
	pcPort << "map() : x == " << x << ", in_min == " << in_min	
<< ", in_max == " << in_max << ", out_min == " << out_min << ", out_max == " << out_max << "\n\n";	
#endif // DEBUG

	
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}