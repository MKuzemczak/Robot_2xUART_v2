#include "Robot.h"

using namespace Eigen;

Robot::Robot()
{
	DOF = 0;
	
	joints.push_back(&TCP);
}


Robot::~Robot()
{
}

void Robot::addRegJoint(double a, double al, double dl)
{
	regJoints.push_back(Joint(a * DEG_TO_RAD, al, dl));
	
	// dodanie do listy wszystkich przegubow wskaznik na teraz dodawany
	joints.insert(joints.iteratorAt(regJoints.size() - 1), &(regJoints[regJoints.size() - 1]));
	
	updateDHmatrices();	
	updateDHvectors();
}
void Robot::addLocJoint(double a, double al, double dl)
{
	locJoints.push_back(Joint(a * DEG_TO_RAD, al, dl));
	
	joints.insert(joints.iteratorAt(joints.size() - 1), &(locJoints[locJoints.size() - 1]));
	
	updateDHmatrices();
	updateDHvectors();
}
	
void Robot::updateDHmatrices()
{
	double sT, cT, sa, ca, aLength, dLength;	
	
	for (int i = 0; i < joints.size(); i++)
	{
		Matrix4d & DHmatrix = joints[i]->getDHmatrix();
		
		if (i == 0)
		{
			cT = 1;
			sT = 0;
		}
		else
		{
			cT = cos(joints[i - 1]->getTheta());
			sT = sin(joints[i - 1]->getTheta());
		}
		
		ca = cos(joints[i]->getAlpha());
		sa = sin(joints[i]->getAlpha());
		
		aLength = joints[i]->getaLength();
		dLength = joints[i]->getdLength();

		
		DHmatrix(0, 0) = cT;
		DHmatrix(0, 1) = -sT * ca;
		DHmatrix(0, 2) = sT * sa;
		DHmatrix(0, 3) = aLength * cT;
		DHmatrix(1, 0) = sT;
		DHmatrix(1, 1) = cT * ca;
		DHmatrix(1, 2) = -cT * sa;
		DHmatrix(1, 3) = aLength * sT;
		DHmatrix(2, 1) = sa;
		DHmatrix(2, 2) = ca;
		DHmatrix(2, 3) = dLength;
		
	}
	
	
}

void Robot::updateDHvectors()
{
	Vector3d v;
	Matrix4d m;
	
	m = Matrix4d::Identity();
	
	for (int i = 0; i < joints.size(); i++)
	{
		m *= joints[i]->getDHmatrix();
		
		//pcPort << m << '\n'; // debug
		
		v << m(0, 3), m(1, 3), m(2, 3);
			
		joints[i]->setLocation(v);
		
		Vector4d Z;
		Z << 0, 0, 1, 1;
		
		Z = (m * Z).eval();
		
		v << Z(0), Z(1), Z(2); 
		joints[i]->setZinGlobal(v);
	}
}
	

void Robot::setThetaDeg(int joint, double theta)
{
	joints[joint]->setTheta(theta*DEG_TO_RAD);
	
	updateDHmatrices();
	updateDHvectors();
}
void Robot::setThetaRad(int joint, double theta)
{
	joints[joint]->setTheta(theta*DEG_TO_RAD);
	
	updateDHmatrices();
	updateDHvectors();
}

Eigen::Vector3d & Robot::getJointLocation(int joint)
{
	return joints[joint]->getLocation();
}	
	
Eigen::Matrix4d & Robot::getJointDHmatrix(int joint)
{
	return joints[joint]->getDHmatrix();
}

Eigen::Vector3d & Robot::getJointZinGlobal(int joint)
{
	return joints[joint]->getZinGlobal();
}