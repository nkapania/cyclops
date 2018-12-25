#include "controllers.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "speed_profiles.h"
#include "tiremodels.h"
#include "matplotlibcpp.h"
#include <iostream>
#include "utils.h"

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

}


double LanekeepingController::getDeltaFB(LocalState_T& localState,const double betaFFW){
	double kLK = this->kLK;
	double xLA = this->xLA;
	double e = localState.e;
	double deltaPsi = localState.dPsi;

	double deltaFB = -kLK * (e + xLA * sin(deltaPsi + betaFFW));
	return deltaFB;
}

double LanekeepingController::speedTracking(LocalState_T& localState){
	//Unpack vectors for readability

	std::vector<double> AxTable = this-> profile.Ax;
	std::vector<double> UxTable = this-> profile.Ux;
	std::vector<double> sTable  = this-> profile.s;

	double m = this->vehicle.m;
	double fdrag = this->vehicle.dragCoeff;
	double frr = this->vehicle.rollResistance;

	double s = localState.s;
	double Ux = localState.Ux;

	//TODO: Get help for why we can't find interpolate1D function
	double AxDes = 0;
	double UxDes = 0;

	//interpolate1D(sTable, AxTable, sTable.size(), s, AxDes);
	//InterpData_T out2;// = interpolate1D(sTable, UxTable, sTable.size(), s, UxDes);


	double FxFFW = m*AxDes + sgn(Ux)*fdrag*pow(Ux, 2) + frr*sgn(Ux);
	double FxFB  = -this->kSpeed * (Ux - UxDes); //feedback
	double FxCommand = FxFFW + FxFB;

	return FxCommand;

}

double LanekeepingController::getDeltaFFW(LocalState_T& localState,double& betaFFW, double K){
	double a = this->vehicle.a;
	double b = this->vehicle.b;
	double L = this->vehicle.L;
	double m = this->vehicle.m;

	double Ux = localState.Ux;

	double FyFdes = b / L * m * pow(Ux, 2) * K ;
	double FyRdes = a / b * FyFdes;

	double alphaFdes = force2alpha(this->FyFtable, this->alphaFtable, FyFdes);
	double alphaRdes = force2alpha(this->FyRtable, this->alphaRtable, FyRdes);

	betaFFW = alphaRdes + b*K;
	double deltaFFW = K * L + alphaRdes - alphaFdes;

	return deltaFFW;

}

//Destructor
LanekeepingController::~LanekeepingController(){
};


