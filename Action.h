#pragma once

#include "EigenAddons.h"
#include "Lista.h"
#include "Robot.h"

typedef enum 
{
	STRAIGHT_LINE,
	FREE,
	ARCH,
	LOCAL_CHANGE,
	DELAY,
	SINGLE
} ActionType;


/* Klasa Action, po której dziedzicz¹ klasy pochodne,
 * pozwala na stworzenie jednolitej sekwencji, listy czynnoœci, które robot ma wykonaæ.
 * 
 * Ka¿da klasa pochodna dla Action przechowuje dane potrzebne
 *  do wykonania danej czynnoœci (np. punkty poœrednie lub czas postoju), 
 *  oraz metodê, która wykorzystuje te dane (np. interpoluje tor lub obs³uguje czasomierz). 
 *
 **/
class BaseAction
{
	// klasa bazowa, niewiele tu jest, wszystko w klasach pochodnych
	
	ActionType type;
	bool calculated,
		done;
	
public:
	BaseAction();
	~BaseAction()
	{
	}
	
	void setType(ActionType t);
	void setCalculated();
	bool isCalculated();
	void resetCalculated();
	void setDone();
	void resetDone();
	bool isDone();
	
	virtual void calculate(Robot & robot)
	{
	}
	virtual void execute()
	{		  
	}
	virtual int size()
	{
	}
};

class StraightLineMovAction : public BaseAction
{
	Eigen::Vector3d starting,
					destination;
	
	Lista<Lista<int>> pathInServoDegs;
	
	
public:
	StraightLineMovAction(Eigen::Vector3d start, Eigen::Vector3d dest);
	~StraightLineMovAction()
	{
	}
	
	virtual void calculate(Robot & robot);
	virtual void execute();
	
	void lerp(Lista<Eigen::Vector3d> & path);
	virtual int size();
};

class FreeMovAction : public BaseAction
{
	Eigen::Vector3d destination;
	
	Lista<int> destInServoDegs;
	
public:
	FreeMovAction(Eigen::Vector3d dest);
	~FreeMovAction();
	
	virtual void calculate(Robot & robot);
	virtual void execute();
};

class ArchMovAction : public BaseAction
{
	Eigen::Vector3d starting, 
					intermediate,
					destination;
	
public:
	ArchMovAction(Eigen::Vector3d start, Eigen::Vector3d inter,
		Eigen::Vector3d dest);
	
	~ArchMovAction();
	
	virtual void calculate(Robot & robot);
	
	virtual void execute();
	
};

class DelayAction : public BaseAction
{
	double millis;
	
public:
	DelayAction(double mill);
	~DelayAction();
	
	virtual void calculate(Robot & robot)
	{
	}
	virtual void execute()
	{
	}
};

class SetSingleJointAction : public BaseAction
{
	int joint,
		angleDeg,
		servoSignal;
	
public:
	SetSingleJointAction();
	~SetSingleJointAction();
	
	virtual void calculate(Robot & robot);
	virtual void execute();
};