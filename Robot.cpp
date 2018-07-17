#include "Robot.h"

using namespace Eigen;

Robot::Robot()
{
	DOF = 0;
}


Robot::~Robot()
{
}

void Robot::addRegJoint(double a, double T, double l)
{
	regJoints.push_back(Joint(a, T, l));
	
	updateDHvectors();
}
void Robot::addLocJoint(double a, double T, double l)
{
	locJoints.push_back(Joint(a, T, l));
	
	updateDHvectors();
}
	
void Robot::updateDHvectors()
{
	Matrix4d m;
	
	m = Matrix4d::Identity();
	
	for (int i = 0; i < regJoints.size(); i++)
	{
		Vector3d v;
		v << m(0, 3), m(1, 3), m(2, 3);
			
		regJoints[i].setLocation(v);
		
		m *= regJoints[i].getDHmatrix();
		
		Vector4d Z;
		Z << 0, 0, 1, 1;
		
		Z = (m * Z).eval();
		
		v << Z(0), Z(1), Z(2); 
		regJoints[i].setZinGlobal(v);
	}
	
	for (int i = 0; i < locJoints.size(); i++)
	{
		Vector3d v;
		v << m(0, 3), m(1, 3), m(2, 3);
			
		locJoints[i].setLocation(v);
		
		m *= locJoints[i].getDHmatrix();
		
		Vector4d Z;
		Z << 0, 0, 1, 1;
		
		Z = (m * Z).eval();
		locJoints[i].setZinGlobal(v);
	}
}
	