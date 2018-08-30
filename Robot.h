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

	void mapThetasToServos(Lista<int> & lista);
	
	void updateDHmatrices();
	void updateDHvectors();
	bool jacobian(Eigen::MatrixXd & jacobM, int indexOfSetJoint/*Lista<Joint> & joints, Eigen::Vector3d & setPoint*/);	
	bool jacobAlgStep(double param, int indexOfSetJoint, Eigen::Vector3d & target);
	bool set(Eigen::Vector3d & point); // using jacobian algorithm
	bool setRegional(Eigen::Vector3d & point); 
	
	bool jacobAlgStep(double param, int startJoint, int endJoint, int setJoint, Eigen::Vector3d & target);
	bool jacobian(Eigen::MatrixXd & jacobM, int startJoint, int endJoint, int setJoint);	
	
	//////////////////////////////////////////////////////// setters & getters & adders
	void addRegJoint(double a, double al, double dl);
	void addLocJoint(double a, double l, double dl);
	
	void setThetaDeg(int joint, double theta);
	void setThetaRad(int joint, double theta);
	
	void setTCPaLength(double l);
	
	Eigen::Vector3d & getJointLocation(int joint);
	Eigen::Vector3d & getTCPlocation();
	Eigen::Matrix4d & getJointDHmatrix(int joint);
	Eigen::Vector3d & getJointZinGlobal(int joint);
	
	int getDOF();
	
	double getJointThetaRad(int joint);
	
	void addJointServoMinMax(int joint, int min, int max);
	
	void setJointConversionMinMax(int joint, int min, int max);
	
	int getJointConversionMin(int joint);
	int getJointConversionMax(int joint);
	
	void setJointConstructionMinMax(int joint, int min, int max);
	
	int getJointConstructionMin(int joint);
	int getJointConstructionMax(int joint);
	
	int getJointServoAmount(int joint);
	/////////////////////////////////////////////////////////////// !setter & getters & adders
	
};

void constrain(double & x, double min, double max);