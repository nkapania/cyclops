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
	double AxMax = std::min(this->mu * g, std::abs(this->axMax));

	//calculate lowest velocity point
	int N = K.size();

	std::vector<double> UxSS(N);
	for (int i = 0; i < N; i++){
		UxSS.at(i) = std::sqrt(AyMax / std::abs(K.at(i) + 0.000000001));
	}

	double minUx = *std::min_element(UxSS.begin(), UxSS.end());
	double maxUx = this -> vMax;
	int idx   = std::min_element(UxSS.begin(), UxSS.end()) - UxSS.begin();

	//std::cout << idx << std::endl;

	std::vector<double> kShifted (K);
	std::rotate(kShifted.begin(), kShifted.begin() + idx, kShifted.end());

	std::vector<double> UxShift;
	std::vector<double> AxShift;

	this->genSpeed(UxShift, AxShift, kShifted, minUx, maxUx, AxMax, AyMax);

	//unshift back to original. 

	std::rotate(UxShift.begin(), UxShift.end() - idx, UxShift.end());
	std::rotate(AxShift.begin(), AxShift.end() - idx, AxShift.end());

	this->Ux = UxShift;
	this->Ax = AxShift;


}

void BasicProfile::genSpeed(std::vector<double>& UxShift, std::vector<double>& AxShift, std::vector<double> kShifted, double minUx, double maxUx, double AxMax, double AyMax){
	const double g = 9.81; 
	std::vector<double> s = this->s;
	double numSteps = s.size();

	//pre allocate three velocity profiles (stead state, braking, decel)

	std::vector<double> UxInit1 (numSteps, 0);
	std::vector<double> UxInit2 (numSteps, 0);
	std::vector<double> UxInit3 (numSteps, 0);

	UxInit2[0] = minUx;
	UxInit3[numSteps-1] = minUx;

	//pre-allocate Ax and Ay

	std::vector<double> ax(numSteps, 0);
	std::vector<double> ay(numSteps, 0);


	//Desired velocity should meet lateral acceleration requirement
	for (int i = 0; i < numSteps; i++){
		UxInit1[i] = std::sqrt(AyMax / std::abs(kShifted.at(i) + 0.000000001));
	}

	//integrate forward to find acceleration limit

	for (int i = 0; i < UxInit2.size() - 1; i++){
		double temp = std::sqrt( pow( UxInit2[i] , 2 ) + 2*AxMax*(s[i+1] - s[i]));
		//std::cout << AxMax << std::endl;

		if (temp > maxUx) temp = maxUx;
		if (temp > UxInit1[i+1]) temp = UxInit1[i+1];

		UxInit2[i+1] = temp;
	}

	// namespace plt = matplotlibcpp;
	// plt::plot(UxInit2);
	// plt::show();

	//moving rearward,  integrate backwards
	for (int i = UxInit3.size()-1; i >= 0; i--){
		double temp = std::sqrt( pow( UxInit3[i], 2) + 2 * AxMax * (s[i] - s[i-1]));

		if (temp > UxInit2[i-1]) temp = UxInit2[i-1];
		UxInit3[i-1] = temp;
	}

	//calculate acceleration profile from physics
	for (int i = 0; i < UxInit3.size()-1; i++){
		ax[i] = (pow(UxInit3[i+1], 2) - pow( UxInit3[i], 2)) / (2* ( s[i+1] - s[i]));
	}

	ax[numSteps-1] = ax[numSteps - 2]; // assign last value of ax

	UxShift = UxInit3;
	AxShift = ax;

}


