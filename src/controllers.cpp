#include "controllers.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "speed_profiles.h"
#include "tiremodels.h"
#include "matplotlibcpp.h"
#include <iostream>


//Note - this is a poor man's linspace function, not 
// robustly tested - C++ does not appear to have a 
// standard version of this. Do not use outside of this file. 
std::vector<double> linspace(double a, double b, int n) {
    std::vector<double> array;
    double step = (b-a) / (n-1);

    while(a <= b) {
        array.push_back(a);
        a += step;           // could recode to better handle rounding errors
    }
    return array;
}

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

    namespace plt = matplotlibcpp;
    plt::plot(alphaFtable, FyFtable);
    plt::plot(alphaRtable, FyRtable);

    plt::show();


}


//Destructor
LanekeepingController::~LanekeepingController(){
};
