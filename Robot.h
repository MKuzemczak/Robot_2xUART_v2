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
	
	Lista<Eigen::Vector2i> servoDegLimits; // each element of the list is a vector containing min & max values of a servo, measured in units accepted by servo controller
	
	Lista<Eigen::Vector2i> jointDegConstraints;	// each element of the list is a vector of min & max degrees of a joint (constructional restrictions)
	
	Lista<Eigen::Vector2i> jointDegLimitsForConversion; // needed to convert from radians to servo degs
	
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
	
	int getServoMin(int index)
	{
		return servoDegLimits[index][0];
	}
	
	int getServoMax(int index)
	{
		return servoDegLimits[index][1];
	}
	
	int getConversionMin(int index)
	{
		return jointDegLimitsForConversion[index][0];
	}
	
	int getConversionMax(int index)
	{
		return jointDegLimitsForConversion[index][1];
	}
	
	void updateDHmatrices();
	void updateDHvectors();
	bool jacobian(Eigen::MatrixXd & jacobM, int indexOfSetJoint/*Lista<Joint> & joints, Eigen::Vector3d & setPoint*/);	
	bool jacobAlgStep(double param, int indexOfSetJoint, Eigen::Vector3d & target);
	bool set(Eigen::Vector3d & point); // using jacobian algorithm
	bool setRegional(Eigen::Vector3d & point); 
};

void constrain(double & x, double min, double max);