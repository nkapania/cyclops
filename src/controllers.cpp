#include "controllers.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "speed_profiles.h"


//Constructor::

LanekeepingController::LanekeepingController(World& world, Vehicle_T& vehicle){
	this-> xLA = 14.2; //meters
	this-> kLK = 0.0538; //rad / meter
	this-> kSpeed = 3000.0; //speed proportional gain
	this-> alphaFlim = 7.0 * M_PI / 180;
	this-> alphaRlim = 5.0 * M_PI / 180;
	this-> numTableValues = 250;

	this->world = world;
	this->vehicle = vehicle;
	//this->profile = profile;

	//values where car is sliding:

	this-> alphaFslide = abs( atan(3 * vehicle.muF * vehicle.m * vehicle.b/ vehicle.L*vehicle.g/vehicle.Cf));
	this-> alphaRslide = abs( atan(3 * vehicle.muR * vehicle.m * vehicle.a/ vehicle.L*vehicle.g/vehicle.Cr));

    // this-> FyFtable = fiala(vehicle.Cf, vehicle.muF, vehicle.muF, alphaFtable, vehicle.FzF);
    // this-> FyRtable = fiala(vehicle.Cr, vehicle.muR, vehicle.muR, alphaRtable, vehicle.FzR);

    // this-> alphaFtable = 
    // this-> alphaRtable = 


}


//Destructor
LanekeepingController::~LanekeepingController(){
};
