#pragma once

#include <vector>

#include "Joint.h"

class Robot
{
	std::vector<Joint> regJoints,
		locJoints;// lista przegubów
	// pomys³: rozdzelone na regionalne i lokalne
	Eigen::Vector3d TCPloc;// polozenie TCP
	int DOF;// liczba DOF
public:
	Robot();
	
	void addRegJoint(double a, double T, double l);
	void addLocJoint(double a, double T, double l);
	
	void updateDHvectors();
	
	~Robot();
};

