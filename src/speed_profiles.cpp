#include "speed_profiles.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <math.h>
#include <iostream>
#include "matplotlibcpp.h"

//Constructor
BasicProfile::BasicProfile(Vehicle_T& vehicleIn, World& worldIn, double frictionIn, double vMaxIn, double AxMaxIn){
	this->vehicle = vehicleIn;
	this->world = worldIn;
	this->mu = frictionIn;
	this->vMax = vMaxIn;
	this->axMax = AxMaxIn;

	this->s=worldIn.s; //note that this creates a copy
	

	if (world.isOpen){
		this->generateBasicProfileOpen();
	}
	else{
		this->generateBasicProfileClosed();
	}

}

BasicProfile::~BasicProfile(){
}

void BasicProfile::generateBasicProfileOpen(){
	//to be implemented
	}


void BasicProfile::generateBasicProfileClosed(){
	//unpack constants and vectors for ease of readability
	double g = this->vehicle.g;
	std::vector<double> K = this->world.curvature;
	std::vector<double> s = this->world.s;
	double AyMax = this->mu * g;
	double AxMax = std::min(this->mu * g, std::abs(AxMax));

	//calculate lowest velocity point
	int N = K.size();

	std::vector<double> UxSS(N);
	for (int i = 0; i < N; i++){
		UxSS.at(i) = std::sqrt(AyMax / std::abs(K.at(i) + 0.000000001));
	}

	double minUx = *std::min_element(UxSS.begin(), UxSS.end());
	double maxUx = this -> vMax;
	int idx   = UxSS.begin() - std::min_element(UxSS.begin(), UxSS.end());

	// std::vector<double> kShifted (K);
	// std::rotate(kShifted.begin(), kShifted.begin() + idx, kShifted.end());

	// std::vector<double> UxShift;
	// std::vector<double> AxShift;

	//this->generateSpeed(UxShift&, AxShift&, kShifted, minUx, maxUxShifted, AxMax, AyMaxShifted)

	// std::rotate(UxShift.begin(), UxShift - idx, UxShift.end())
	// std::rotate(AxShift.begin(), AxShift - idx, AxShift.end())

	// this -> Ux = UxShift;
	// this -> Ax = AxShift:




	// std::cout << idx << std::endl;
	// std::cout << minUx << std::endl;





}