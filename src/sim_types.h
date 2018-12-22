#pragma once

struct Vehicle_T{
	double a; 
	double b;
	double d; 
	double rW; 
	double m; 
	double Cf; 
	double Cr;
	double Iz; 
	double muF; 
	double muR;
	double g;
	double L;
	double FzF;
	double FzR;
	double h; 
	double brakeTimeDelay; 
	double rollResistance;
	double powerLimit;
	double dragCoeff;
	double deltaLim;
	double beta;
	double brakeFactor; 
};

void loadVehicleShelley(Vehicle_T& vehicle);