#include "Joint.h"



Joint::Joint(double a, double T, double l)
{
	alpha = a;
	Theta = T;
	length = l;
	
	DHmatrix = Eigen::Matrix4d::Identity();
	updateDHmatrix();
}


Joint::~Joint()
{
}

void Joint::updateDHmatrix()
{
	double cT = cos(Theta),
		ca = cos(alpha),
		sT = sin(Theta),
		sa = sin(alpha);
	
	
	DHmatrix(0, 0) = cT;
	DHmatrix(0, 1) = -sT * ca;
	DHmatrix(0, 2) = sT * sa;
	DHmatrix(0, 3) = length * cT;
	DHmatrix(1, 0) = sT;
	DHmatrix(1, 1) = cT * ca;
	DHmatrix(1, 2) = -cT * sa;
	DHmatrix(1, 3) = length * sT;
	DHmatrix(2, 1) = sa;
	DHmatrix(2, 2) = ca;
}
