#pragma once


#include "Action.h"
#include "Lista.h"

#define DEBUG_ACTION_MANAGER

class ActionManager
{
	Lista<BaseAction*> actions;
	bool stepInProgress,
		checkCalculations,
		started;
	int actionCntr;
	
	Robot* robotPtr;
public:
	ActionManager();
	~ActionManager();
	
	bool start();
	void nextStep();
	void calculations();
	
	
	int size()
	{
		return actions.size();
	}
	
	void addStraightLineMovAction(Eigen::Vector3d & start, Eigen::Vector3d & dest)
	{
		actions.push_back(new StraightLineMovAction(start, dest));
	}
	
	void addFreeMovAction(Eigen::Vector3d & dest)
	{
		actions.push_back(new FreeMovAction(dest));
	}
	
	void addArchMovAction(Eigen::Vector3d start,
						Eigen::Vector3d inter,
						Eigen::Vector3d dest)
	{
		actions.push_back(new ArchMovAction(start, inter, dest));
	}
	
	bool isCheckCalculations()
	{
		return checkCalculations;
	}
	
	void setRobotPtr(Robot * ptr)
	{
		robotPtr = ptr;
	}
	
	bool isStarted()
	{
		return started;
	}
	
	void stop()
	{
		started = false;
	}
};

