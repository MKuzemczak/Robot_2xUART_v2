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
	DELAY
} ActionType;


/* Klasa Action, po której dziedzicz¹ klasy pochodne,
 * pozwala na stworzenie jednolitej sekwencji, listy czynnoœci, które robot ma wykonaæ.
 * 
 * Ka¿da klasa pochodna dla Action przechowuje dane potrzebne
 *  do wykonania danej czynnoœci (np. punkty poœrednie lub czas postoju), 
 *  oraz metodê, która wykorzystuje te dane (np. interpoluje tor lub obs³uguje czasomierz). 
 *
 **/
class Action
{
	// klasa bazowa, niewiele tu jest, wszystko w klasach pochodnych
	
	ActionType type;
	
public:
	Action()
	{
	}
	~Action()
	{
	}
	
	void setType(ActionType t)
	{
		type = t;
	}
	
	virtual void calculate(Robot & robot)
	{
	}
	virtual void execute()
	{
	}
};

class StraightLineMovAction : public Action
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
};

class FreeMovAction : public Action
{
	Eigen::Vector3d destination;
	
	Lista<int> destInServoDegs;
	
public:
	FreeMovAction(Eigen::Vector3d dest);
	~FreeMovAction();
	
	virtual void calculate(Robot & robot);
	virtual void execute();
};

class ArchMovAction : public Action
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

class DelayAction : public Action
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
