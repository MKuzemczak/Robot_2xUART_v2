#pragma once	

#include <vector>

#include "Lista.h"

#include "Joint.h"

//#define DEBUG_ROBOT

class Robot
{
	Lista<Joint> regJoints,
		locJoints;// lista przegubów
	// pomys³: rozdzelone na regionalne i lokalne
	
	Lista<Joint*> joints;
	
	Joint TCP;// polozenie TCP
	int DOF;// liczba DOF
	
	
	// aktualne rozwarcie chwytaka
	// lista punktow	
public:
	Robot();
	~Robot();

	
	void addRegJoint(double a, double al, double dl);
	void addLocJoint(double a, double l, double dl);
	
	
	
	
	void setThetaDeg(int joint, double theta);
	void setThetaRad(int joint, double theta);
	
	void setTCPaLength(double l)
	{
		TCP.setaLength(l); 
		
		updateDHmatrices();
		updateDHvectors();
	}
	
	Eigen::Vector3d & getJointLocation(int joint);
	Eigen::Vector3d & getTCPlocation()
	{
		return TCP.getLocation();
	}
	Eigen::Matrix4d & getJointDHmatrix(int joint);
	Eigen::Vector3d & getJointZinGlobal(int joint);
	
	int getDOF()
	{
		return DOF;
	}
	
	double getJointThetaRad(int joint)
	{
		return joints[joint]->getTheta();
	}
	
	void addJointServoMinMax(int joint, int min, int max)
	{
		joints[joint]->addServoMinMax(min, max);
	}
	
	void setJointConversionMinMax(int joint, int min, int max)
	{
		joints[joint]->setConversionMinMaxDeg(min, max);
	}
	
	void setJointConstructionMinMax(int joint, int min, int max)
	{
		joints[joint]->setConstructionMinMaxDeg(min, max);
	}
	
	void mapThetasToServos(Lista<int> & lista);
	
	void updateDHmatrices();
	void updateDHvectors();
	bool jacobian(Eigen::MatrixXd & jacobM, int indexOfSetJoint/*Lista<Joint> & joints, Eigen::Vector3d & setPoint*/);	
	bool jacobAlgStep(double param, int indexOfSetJoint, Eigen::Vector3d & target);
	bool set(Eigen::Vector3d & point); // using jacobian algorithm
	bool setRegional(Eigen::Vector3d & point); 
};

void constrain(double & x, double min, double max);