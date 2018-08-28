#pragma once

#include "EigenAddons.h"
#include "uartCom.h"
#include "Lista.h"

//#define DEBUG_JOINT

#define ERR_NO_LENGTH 1

class Joint
{
	double alpha; // kat alfa - wokol tego X, miedzy poprzednim Z a tym tutaj.
	double Theta; // kat Theta - wokol tego Z
	double aLength; // dlugosc a - wzdluz tego x
	double dLength; // d³ugoœæ d - wzdluz poprzedniego z
	Eigen::Matrix4d DHmatrix; // macierz DH transformacji punktu w tym ukladzie do poprzedniego ukladu
	Eigen::Vector3d locationInGlobal,  // wektor polozenia przegubu w globalnym ukladzie
		ZaxisInGlobal;
	
	Lista<Eigen::Vector2i> servosMinMax;					// list of minimum & maximum signals of servos that this joint represents
	Eigen::Vector2i angleConversionMinMaxDeg,		// list if min&max angles needed for conversion. In degrees (because Integer)
							angleConstructionMinMaxDeg;		// list of min&max angles imposed by construction of the robot. In degrees
	
public:
	Joint();
	Joint(double a, double al, double dl);
	
	void mapThetaToServo(Lista<int> & lista);
	
	void setAlpha(double a)
	{
		alpha = a;
			
	}
	
	double getAlpha()
	{
		return alpha;
	}
	
	// aktualizuj kat Theta
	void setTheta(double T)
	{
		Theta = T;
	}
	
	double getTheta()
	{
		return Theta;
	}
	
	void setaLength(double l)
	{
		aLength = l;
	}
	
	double getaLength()
	{
		return aLength;
	}
	
	void setdLength(double l)
	{
		dLength = l;
	}
	
	double getdLength()
	{
		return dLength;
	}
	
	void setLocation(Eigen::Vector3d & v)
	{
		locationInGlobal = v;
	}
	
	Eigen::Vector3d & getLocation()
	{
		return locationInGlobal;
	}
	
	void setZinGlobal(Eigen::Vector3d & v)
	{
		ZaxisInGlobal = v;
	}
	
	Eigen::Vector3d & getZinGlobal()
	{
		return ZaxisInGlobal;
	}
	
	Eigen::Matrix4d & getDHmatrix()
	{
		return DHmatrix;
	}
	
	void addServoMinMax(int min, int max)
	{
		Eigen::Vector2i v;
		
		v << min, max;
		
		
#ifdef DEBUG_JOINT
		pcPort << "Joint::addServoMinMax(" << min << ", " << max << ");\n\n";				  
#endif // DEBUG_JOINT
		
		servosMinMax.push_back(v);
	}
	
	void setServoMinMax(int servo, int min, int max)
	{
		if (servo < 0 || servo >= servosMinMax.size())
			pcPort << "Joint::setServoMinMax(...): Error: Servo index out of bounds\n";
		else
		{
			servosMinMax[servo][0] = min;
			servosMinMax[servo][1] = max;
		}
		

	}
	
	void setConversionMinMaxDeg(int min, int max)
	{
		angleConversionMinMaxDeg << min, max;
		
#ifdef DEBUG_JOINT
		pcPort << "Joint::setConversionMinMaxDeg(" << min << ", " << max << ");\n\n";				  
#endif // DEBUG_JOINT
	}
	
	void setConstructionMinMaxDeg(int min, int max)
	{
		angleConstructionMinMaxDeg << min, max;
		
#ifdef DEBUG_JOINT
		pcPort << "Joint::setConstructionMinMaxDeg(" << min << ", " << max << ");\n\n";				  
#endif // DEBUG_JOINT
	}
	
	Eigen::Vector2i & getConstructionMinMaxDeg()
	{
		return angleConstructionMinMaxDeg;
	}
	
	int getServoAmount()
	{
		return servosMinMax.size();
	}
	
	int getConstructionMinDeg();
	int getConstructionMaxDeg();
	int getConversionMinDeg();
	int getConversionMaxDeg();
	
	~Joint();
};

double map(double x, double in_min, double in_max, double out_min, double out_max);