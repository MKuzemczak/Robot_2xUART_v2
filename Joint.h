#pragma once
#include "EigenAddons.h"

class Joint
{
	double alpha; // kat alfa - miedzy poprzednim Z a tym tutaj.
	double Theta; // kat Theta - miedzy poprzednim X a tym tutaj.
	double length; // dlugosc l - miedzy poprzednim Z a tym tutaj
	Eigen::Matrix4d DHmatrix; // macierz DH transformacji punktu w tym ukladzie do poprzedniego ukladu
	Eigen::Vector3d locationInGlobal,  // wektor polozenia przegubu w globalnym ukladzie
		ZaxisInGlobal;
	
public:
	Joint(double a, double T, double l);
	
	void setAlpha(double a)
	{
		alpha = a;
		updateDHmatrix();
	}
	
	double getAlpha()
	{
		return alpha;
	}
	
	// aktualizuj kat Theta
	void setTheta(double T)
	{
		Theta = T;
		updateDHmatrix();
	}
	
	double getTheta()
	{
		return Theta;
	}
	
	void setLength(double l)
	{
		length = l;
		updateDHmatrix();
	}
	
	double getLength()
	{
		return length;
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
	
	// aktualizuj macierz DH
	void updateDHmatrix();
	
	~Joint();
};

