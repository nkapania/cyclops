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

struct LocalState_T{
	double Ux;
	double Uy;
	double r; 
	double s; 
	double e; 
	double dPsi;
};

struct AuxVars_T{
	double K;
	double UxDes;
	double AxDes;
	double alphaFdes;
	double alphaRdes;
	double deltaFFW;
	double deltaFB;
	double betaFFW;
	};

struct ControlInput{
	double delta;
	double Fx; 
};



void loadVehicleShelley(Vehicle_T& vehicle);