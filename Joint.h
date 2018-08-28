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
	
	~Joint();
	
	
	void mapThetaToServo(Lista<int> & lista);
	
	
	/////////////////////////////////////// setters & getters & adders
	void setAlpha(double a);
	
	double getAlpha();
	
	// aktualizuj kat Theta
	void setTheta(double T);
	
	double getTheta();
	
	void setaLength(double l);
	
	double getaLength();
	
	void setdLength(double l);
	
	double getdLength();
	
	void setLocation(Eigen::Vector3d & v);
	
	Eigen::Vector3d & getLocation();
	
	void setZinGlobal(Eigen::Vector3d & v);
	
	Eigen::Vector3d & getZinGlobal();
	
	Eigen::Matrix4d & getDHmatrix();
	
	void addServoMinMax(int min, int max);
	
	void setServoMinMax(int servo, int min, int max);
	
	void setConversionMinMaxDeg(int min, int max);
	
	void setConstructionMinMaxDeg(int min, int max);
	
	Eigen::Vector2i & getConstructionMinMaxDeg();
	
	int getServoAmount();
	
	int getConstructionMinDeg();
	int getConstructionMaxDeg();
	int getConversionMinDeg();
	int getConversionMaxDeg();
	///////////////////////////////////////////////////////// !setters & getters & adders
	
};

double map(double x, double in_min, double in_max, double out_min, double out_max);