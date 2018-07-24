#pragma once

#include "EigenAddons.h"

typedef enum 
{
	STRAIGHT_LINE,
	FREE,
	ARCH,
	LOCAL_PART_CHANGE
} PointType;

class Action
{
	// klasa bazowa, niewiele tu jest, wszystko w klasach pochodnych
	
	PointType type;
	
public:
	Action();
	~Action();
};

class StraightLineMovAction : public Action
{
	Eigen::Vector3d destination;
	
public:
};

