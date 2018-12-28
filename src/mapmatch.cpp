#include "mapmatch.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <complex> //complex
#include "utils.h"
#include "world.h"

MapMatch::MapMatch(World& world){
	this->world = world;
	this-> seed = 0; //index of map to take a guess
	this-> firstSearch = true;
	this-> REQUIRED_DISTANCE = 10.0;
	this-> MAX_BACKWARD_ITERS = 75;
	this-> MAX_FORWARD_ITERS  = 225;
}

MapMatch:: ~MapMatch(){

}

void MapMatch::localize(LocalState_T& localState, const GlobalState_T globalState){
	
	MapMatchOutput_T out = this->convertToLocal(globalState.X, globalState.Y);
	double e = out.e;
	double s = out.s;
	double psiDes = out.psiDes;
	bool initStatus = out.converged;

	if (!initStatus){
		localState.e = 0.0;
		localState.dPsi = 0.0;
		localState.s = 0.0;
	}

	else{

		//sanity check - s must be within map boundaries
		double sEnd = *(this -> world.s.end()- 1);
		if (s < 0){
			s = sEnd + s; //cycle to end of path
		}
		else if (s > sEnd){ 
			s = s - sEnd;
		}

		double dPsi = globalState.Psi - psiDes;
		while (dPsi > M_PI){
			dPsi = dPsi - 2 * M_PI;
		}

		while (dPsi < -M_PI){
			dPsi = dPsi + 2 * M_PI;
		}

		localState.e = e;
		localState.s = s; 
		localState.dPsi = dPsi; 
	}

}

MapMatchOutput_T MapMatch::convertToLocal(const double posE, const double posN){
	//very crude map matching - works on small maps
	World world = this->world; 
	size_t m = world.s.size();
	std::complex<double> EN (posE, posN); //use complex type here to
	//take advantage of built-in norm function. 

	//go forward

	double lastPair = 9999999;
	int forwardInd = this->seed;
	bool stillDecreasing = true;
	int numForwardIterations = 0;
	double currentPair;

	while (stillDecreasing and (numForwardIterations < this->MAX_FORWARD_ITERS)){
		numForwardIterations++;
		if (forwardInd <= m-2){
			std::complex<double> EN0 (world.posE[forwardInd], world.posN[forwardInd]);
			std::complex<double> EN1 (world.posN[forwardInd+1], world.posN[forwardInd+1]);
			currentPair = std::norm(EN - EN0) + std::norm(EN-EN1);
		}
		else{
			//allow searching at the beginning of the map if world is closed
			if (world.isOpen){
				currentPair = 9999999;
			}
			else{
				std::complex<double> EN1 (world.posE[forwardInd], world.posN[forwardInd]);
				std::complex<double> EN0 (world.posE[0], world.posN[0]);
				currentPair = std::norm(EN - EN1) + std::norm(EN - EN0);
			}

		}

		bool stillDecreasing = currentPair < lastPair;

		if (stillDecreasing){
			lastPair = currentPair;
		}

		//allow searching at beginning of map if map is closed
		if (forwardInd == m-1 && !world.isOpen){
			forwardInd = 0;
		}
		else{
			forwardInd ++;
		}
	}

	double smallestF = lastPair; 
	lastPair = 9999999; //inf
	int backwardInd = this->seed;
	stillDecreasing = true;
	int numBackwardIterations = 0;
	double smallestB;
	int lowSind;
	int highSind;

	while (stillDecreasing & (numBackwardIterations < this->MAX_BACKWARD_ITERS)){
		numBackwardIterations += 1;

		if (backwardInd >=1){
			std::complex<double> EN0 (world.posE[backwardInd], world.posN[backwardInd]);
			std::complex<double> EN1 (world.posE[backwardInd-1], world.posN[backwardInd-1]);
			currentPair = std::norm(EN - EN0) + std::norm(EN-EN1);
		}

		else{
			//allow searching at end of map if map is closed
			if (world.isOpen){
				currentPair = 9999999;
			}
			else{
				std::complex<double> EN0 (world.posE[backwardInd], world.posN[backwardInd]);
				std::complex<double> EN1 (world.posE[m-1]        , world.posN[m-1]); 
				currentPair = currentPair = std::norm(EN - EN0) + std::norm(EN-EN1);

			}

		}

		bool stillDecreasing = currentPair < lastPair; 
		if (stillDecreasing){
			lastPair = currentPair; 

			//allow searching from end of map if map is closed
			if (backwardInd == 0 && !world.isOpen){
				backwardInd = m-1;
			}
			else{
				backwardInd = backwardInd - 1;
			}

		}

		smallestB = lastPair; 

		if (smallestB < smallestF){
			if (backwardInd > 0){
				lowSind = backwardInd - 1;
			}

			else{
				lowSind = m - 2;
				//This should be m-1, but paths are defined so that
				//the last point overlaps with the first point. This will
				//mess up the cross product below, so we just go back one index
				// when we cross to the next lap. 
			}

			highSind  = backwardInd;
		}

		else{
			int lowSInd = forwardInd;
			if (forwardInd < m-1){
				highSind = forwardInd + 1;
			}
			else{
				highSind = 1;
				//This should be 0, but paths are defined so that the last
				//point overlaps with the first point. This messes up the
				//cross product, so just go up one index when we cross to the
				//next laps
			}

		}

	}

	//Need to track this for initialization testing
	double smallestNorm = std::fmin(smallestB, smallestF);

	std::complex<double> ENlow (world.posE[lowSind], world.posN[lowSind]);
	std::complex<double> ENhigh(world.posE[highSind], world.posN[highSind]);

	double a = std::norm(EN - ENlow);
	double b = std::norm(EN - ENhigh);
	double c = std::norm(ENlow - ENhigh);

	double deltaS = (pow(a,2) + pow(c,2) - pow(b,2)) / (2*c);
	double abs_e = std::sqrt(std::abs(pow(a,2) - pow(deltaS,2)));

	double s = world.s[lowSind] + deltaS;

	double headingVector[3] = {-sin(world.roadPsi[lowSind]), cos(world.roadPsi[lowSind]), 0};
	double pENaugmented[3] = {EN.real(), EN.imag(), 0};
	double pathVector[3] = {world.posE[lowSind], world.posN[lowSind], 0}; 

	double positionVector[3] = 	{pENaugmented[0] - pathVector[0], pENaugmented[1] - pathVector[1], pENaugmented[2] - pathVector[2]};
	double crssSgn = crossSign(headingVector[0],headingVector[1], positionVector[0], positionVector[1]);

	double e = abs_e * crssSgn;

	//compute K and psi desired via interpolation
	double psiDes = world.roadPsi[lowSind] + (world.roadPsi[highSind] - world.roadPsi[lowSind])/(world.s[highSind] - world.s[lowSind])*deltaS;
	double K =      world.curvature[lowSind]   + (world.curvature[highSind] - world.curvature[lowSind])/(world.s[highSind] - world.s[lowSind])*deltaS;


	bool converged;
	if (smallestNorm < this->REQUIRED_DISTANCE){
		converged = true;
		this->seed = lowSind;
	}

	else{
		converged = false;
		this->seed = this->seed + this->MAX_BACKWARD_ITERS + this->MAX_FORWARD_ITERS;

		//wrap around if necessary
		if (this->seed > m - 1){
			this->seed = 0;
		}
	}

	int iterations = numForwardIterations + numBackwardIterations;

	MapMatchOutput_T out;
	out.e = e; 
	out.s = s; 
	out.K = K; 
	out.psiDes = psiDes;
	out.converged = converged; 
	out.iterations = iterations;
	out.smallestNorm = smallestNorm; 

	return out;

}

	






























