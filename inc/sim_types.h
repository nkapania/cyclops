#pragma once
#include <vector>

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

struct GlobalState_T{
	double X; 
	double Y;
	double Psi;
};

struct MapMatchOutput_T{
	double e;
	double s;
	double K;
	double psiDes;
	bool converged;
	int iterations;
	double smallestNorm;
};

struct SimOutput_T{
	std::vector<double> AxDes;
	std::vector<double> UxDes;
	std::vector<double> t;
	std::vector<double> K;
	std::vector<double> alphaFdes;
	std::vector<double> alphaRdes;
	std::vector<double> deltaFFW;
	std::vector<double> deltaFB;		
	std::vector<double> betaFFW;
	std::vector<double> Ux;
	std::vector<double> Ax;		
	std::vector<double> s;
	std::vector<double> e;
	std::vector<double> lapNumber;
	std::vector<double> deltaPsi;
	std::vector<double> posE;		
	std::vector<double> posN;
	std::vector<double> psi;
	std::vector<double> r;
	std::vector<double> Uy;
	std::vector<double> deltaCmd;
	std::vector<double> FxCmd;
	std::vector<double> alphaF;
	std::vector<double> alphaR;	
	std::vector<double> FxF;
	std::vector<double> FxR;
	std::vector<double> FyF;
	std::vector<double> FyR;	
	std::vector<double> zetaF;
	std::vector<double> zetaR;
	std::vector<double> FzF;
	std::vector<double> FzR;
};


void loadVehicleShelley(Vehicle_T& vehicle);