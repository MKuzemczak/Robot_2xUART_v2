#include "Joint.h"



Joint::Joint()
{
	alpha = 0;
	Theta = 0;
	aLength = 0;
	dLength = 0;
	
	DHmatrix = Eigen::Matrix4d::Identity();
	//updateDHmatrix();
}

Joint::Joint(double a, double al, double dl)
{
	alpha = a;
	Theta = 0;
	aLength = al;
	dLength = dl;
	
	DHmatrix = Eigen::Matrix4d::Identity();
	//updateDHmatrix();
}


Joint::~Joint()
{
}
