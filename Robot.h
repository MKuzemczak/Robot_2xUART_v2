#pragma once	

#include <vector>

#include "Lista.h"

#include "Joint.h"

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
	
	
	Eigen::Vector3d & getTCPlocation()
	{
		return TCP.getLocation();
	}
	
	void setThetaDeg(int joint, double theta);
	void setThetaRad(int joint, double theta);
	
	void setTCPaLength(double l)
	{
		TCP.setaLength(l); 
		
		updateDHmatrices();
		updateDHvectors();
	}
	
	Eigen::Vector3d & getJointLocation(int joint);
	Eigen::Matrix4d & getJointDHmatrix(int joint);
	Eigen::Vector3d & getJointZinGlobal(int joint);
	
	void updateDHmatrices();
	void updateDHvectors();
	bool jacobian(Eigen::MatrixXd & jacobM, Lista<Joint> & joints, Eigen::Vector3d & setPoint);	
	bool jacobAlgStep();
	
};

