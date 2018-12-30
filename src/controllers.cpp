#include "controllers.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include "speed_profiles.h"
#include "tiremodels.h"
#include <iostream>
#include "utils.h"
#include "sim_types.h"
#include <algorithm>

//Constructor::

LanekeepingController::LanekeepingController(World& world, Vehicle_T& vehicle, BasicProfile& profile){
	this-> xLA = 14.2; //meters
	this-> kLK = 0.0538; //rad / meter
	this-> kSpeed = 3000.0; //speed proportional gain
	this-> alphaFlim = 7.0 * M_PI / 180;
	this-> alphaRlim = 5.0 * M_PI / 180;
	this-> numTableValues = 250;

	this->world = world;
	this->vehicle = vehicle;
	this->profile = profile;

	//values where car is sliding:

	this-> alphaFslide = std::abs( atan(3 * vehicle.muF * vehicle.m * vehicle.b/ vehicle.L*vehicle.g/vehicle.Cf));
	this-> alphaRslide = std::abs( atan(3 * vehicle.muR * vehicle.m * vehicle.a/ vehicle.L*vehicle.g/vehicle.Cr));

    this-> alphaFtable = linspace(-this->alphaFslide, this->alphaFslide, this->numTableValues);
    this-> alphaRtable = linspace(-this->alphaRslide, this->alphaRslide, this->numTableValues);

    this-> FyFtable = fiala(vehicle.Cf, vehicle.muF, vehicle.muF, alphaFtable, vehicle.FzF);
    this-> FyRtable = fiala(vehicle.Cr, vehicle.muR, vehicle.muR, alphaRtable, vehicle.FzR);

    //reverse order of arrays to keep consistency with Python version of code. 

    std::reverse(this->alphaFtable.begin(), this->alphaFtable.end()); 
    std::reverse(this->alphaRtable.begin(), this->alphaRtable.end()); 
    std::reverse(this->FyFtable.begin(),    this->FyFtable.end()); 
    std::reverse(this->FyRtable.begin(),    this->FyRtable.end()); 

}

LanekeepingController::LanekeepingController(){
	}


double LanekeepingController::getDeltaFB(const LocalState_T& localState,const double betaFFW){
	double kLK = this->kLK;
	double xLA = this->xLA;
	double e = localState.e;
	double deltaPsi = localState.dPsi;

	double deltaFB = -kLK * (e + xLA * sin(deltaPsi + betaFFW));
	return deltaFB;
}

double LanekeepingController::speedTracking(const LocalState_T& localState, double& UxDes, double& AxDes, double& FxFFW, double& FxFB){
	//Unpack vectors for readability

	std::vector<double> AxTable = this-> profile.Ax;
	std::vector<double> UxTable = this-> profile.Ux;
	std::vector<double> sTable  = this-> profile.s;

	double m = this->vehicle.m;
	double fdrag = this->vehicle.dragCoeff;
	double frr = this->vehicle.rollResistance;

	double s = localState.s;
	double Ux = localState.Ux;

	interpolate1D(sTable, AxTable, sTable.size(), s, AxDes);
	interpolate1D(sTable, UxTable, sTable.size(), s, UxDes);

	FxFFW = m*AxDes + sgn(Ux)*fdrag*pow(Ux, 2) + frr*sgn(Ux);
	FxFB  = -this->kSpeed * (Ux - UxDes); //feedback
	double FxCommand = FxFFW + FxFB;

	return FxCommand;

}

double LanekeepingController::getDeltaFFW(const LocalState_T& localState,double& betaFFW,const double K, double& FyFdes, double& FyRdes, double& alphaFdes, double& alphaRdes){
	double a = this->vehicle.a;
	double b = this->vehicle.b;
	double L = this->vehicle.L;
	double m = this->vehicle.m;

	double Ux = localState.Ux;

	FyFdes = b / L * m * pow(Ux, 2) * K ;
	FyRdes = a / b * FyFdes;

	alphaFdes = force2alpha(this->FyFtable, this->alphaFtable, FyFdes);
	alphaRdes = force2alpha(this->FyRtable, this->alphaRtable, FyRdes);

	betaFFW = alphaRdes + b*K;
	double deltaFFW = K * L + alphaRdes - alphaFdes;

	return deltaFFW;

}

double LanekeepingController::lanekeeping(const LocalState_T& localState, double& deltaFFW, double& deltaFB, double& K, double& alphaFdes, double& alphaRdes, double& betaFFW){
	std::vector<double> sTable = this->world.s;
	std::vector<double> kTable = this->world.curvature;
	double s = localState.s;


	double FyFdes; double FyRdes; 
	interpolate1D(sTable, kTable, kTable.size(), s, K);
	deltaFFW = this-> getDeltaFFW(localState, betaFFW,  K, FyFdes, FyRdes, alphaFdes, alphaRdes);
	deltaFB  = this-> getDeltaFB (localState, betaFFW);
	double delta = deltaFFW + deltaFB;

	return delta; 
	}


AuxVars_T LanekeepingController::updateInput(const LocalState_T& localState, ControlInput& controlInput){
	
	double deltaFFW, deltaFB, K, alphaFdes, alphaRdes;
	double betaFFW, UxDes, AxDes, FxFFW, FxFB;

	double delta = this->lanekeeping(localState, deltaFFW, deltaFB, K, alphaFdes, alphaRdes, betaFFW);
	double Fx = this-> speedTracking(localState, UxDes, AxDes, FxFFW, FxFB);

	controlInput.delta = delta;
	controlInput.Fx = Fx;  

	
	AuxVars_T out = {};
	out.K = K;
	out.UxDes = UxDes;
	out.AxDes = AxDes;
	out.alphaFdes = alphaFdes;
	out.alphaRdes = alphaRdes;
	out.deltaFFW = deltaFFW;
	out.deltaFB = deltaFB;
	out.betaFFW = betaFFW;

	return out;

	}

//Destructor
LanekeepingController::~LanekeepingController(){
};


