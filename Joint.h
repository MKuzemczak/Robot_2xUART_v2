#pragma once

#include "EigenAddons.h"
#include "uartCom.h"

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
	
public:
	Joint();
	Joint(double a, double al, double dl);
	
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
	
	
	~Joint();
};

