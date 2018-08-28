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

double map(double x, double in_min, double in_max, double out_min, double out_max) {
	
#ifdef DEBUG_JOINT
	pcPort << "map() : x == " << x << ", in_min == " << in_min	
<< ", in_max == " << in_max << ", out_min == " << out_min << ", out_max == " << out_max << "\n\n";	
#endif // DEBUG

	
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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