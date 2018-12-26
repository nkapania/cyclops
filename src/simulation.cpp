#include "simulation.h"
#include <iostream>
#include <cmath>
#include "tiremodels.h"

//Constructor::
Simulation::Simulation(World& world, Vehicle_T& vehicle,
 LanekeepingController& controller, BasicProfile& profile){

 	this-> world = world;
 	this-> vehicle = vehicle;
 	this-> profile = profile;
 	this-> controller = controller;
 	this-> isRunning = true;
 	this-> physics = BICYCLE;
 	this-> ts = 0.01; //simulation time step in seconds
 	this-> maxTime = 99999999.0; 
 	this-> vStart = 10.0;
 	this-> wtType = NONE;
 	this-> matchType = EULER;
 	this-> tires = FIALA;
 	this-> desiredLaps = 1;
 	this-> lapNumber = 0; 
 	this-> last_s = 0.0;  
 	this-> buffer = 0.5; //stop simulation a little before the end of the last lap. 
	this-> maxE = 5.0;
}



//Destructor:

Simulation::~Simulation(){
	}


void Simulation::simulate(){
//initialize states and instantiate objects

	double Ux0 = this->profile.Ux[0];
	LocalState_T localState;
	localState.Ux = Ux0;

	GlobalState_T globalState;
	globalState.X = this->world.posE[0];
	globalState.Y = this->world.posN[0];
	globalState.Psi = this->world.roadPsi[0];

	ControlInput controlInput;

	//start the counter
	int counter = 0;

	//Run the simulation!
	while (this->isRunning){
		this->checkForTermination(localState, counter);
		AuxVars_T auxVars = this->controller.updateInput(localState, controlInput);
		this->updateState(controlInput, localState, globalState, auxVars);

		counter++;

		this->printStatus(localState, counter);

	}


}

void Simulation::checkForTermination(const LocalState_T& localState,const int counter){
	//check if we have ended the simulation
	double t = counter * this-> ts;
	this->incrementLapNumber(localState.s);

	double endS = *this->world.s.end()-this->buffer;

	if (localState.s > endS && this->lapNumber == this->desiredLaps - 1) //Stop simulation a little before end of path
	{
		this->isRunning = false;
		double runTime = counter * this->ts;
		std::cout << "Simulation complete - total time " << runTime << "seconds" << std::endl;

	}

	//check if we have gone off the track
	if (std::abs(localState.e) > this->maxE){
		std::cout << "Car has left the track - terminating ..." << std::endl;
		this->isRunning = false;
	}




}


void Simulation::updateState(const ControlInput& controlInput, LocalState_T& localState, GlobalState_T& globalState, AuxVars_T& auxVars){
	double K = auxVars.K;
	double UxDes = auxVars.UxDes;

	if (this->physics == BICYCLE){
		bicycleModel(controlInput, localState, globalState, K);
	}

	// to do implement other types of physics (e..g 4 wheeled)

}


void Simulation::printStatus(const LocalState_T& localState, const int counter){
	double sEnd = *this->world.s.end();
	double pctComplete = ceil( 100 * (localState.s + sEnd * this->lapNumber) / (sEnd * this->desiredLaps));

	if (counter % 100 == 0){
		std::cout << "Simulation is " << pctComplete << "percent done " << std::endl;
	}

}


void Simulation::incrementLapNumber(const double s){
	//method to check if we have elapsed a lap

	if (this->last_s > s) this->lapNumber ++;
	this->last_s = s;

}

void Simulation::bicycleModel(const ControlInput& controlInput, LocalState_T& localState, GlobalState_T& globalState, const double K){
	
	//unpack variables for brevity
	double FxDes = controlInput.Fx;
	double delta = controlInput.delta;

	double Ux = localState.Ux;
	double r = localState.r; 
	double Uy = localState.Uy;
	double e = localState.e; 
	double deltaPsi = localState.dPsi; 
	double s = localState.s; 

	double X = globalState.X;
	double Y = globalState.Y;
	double psi = globalState.Psi; 

	double m = this->vehicle.m;
	double a = this->vehicle.a;
	double b = this->vehicle.b;
	double Iz = this->vehicle.Iz;

	//calculate forces and tire slips
	double FxF, FxR, FzF, FzR;

	getFx(FxF, FxR, FxDes, Ux); 
	getNormalForces(FzF, FzR, FxF+FxR);

	double alphaF, alphaR;

	getSlips(alphaF, alphaR, localState, controlInput);

	double FyF, FyR, zetaF, zetaR; 

	if (this->tires == COUPLED){
		coupledTireForces(FyF, FyR, zetaF, zetaR, alphaF, alphaR, FxF, FxR, FzF, FzR, vehicle); 
	}
	else if (this->tires == FIALA){
		//just set FxF and FxR to 0
		coupledTireForces(FyF, FyR, zetaF, zetaR, alphaF, alphaR, 0, 0, FzF, FzR, vehicle);
	}

	else if (this->tires == LINEAR){
		FyF = -vehicle.Cf * alphaF; 
		FyR = -vehicle.Cr * alphaR;
		zetaF = 1.0;
		zetaR = 1.0;

	}

	else{
		std::cerr<<"improper tire type specified"<<std::endl;
	}

	//Calculate state derivatives and update
	double dUy = (FyF + FyR) / m - r*Ux;
	double dr  = (a*FyF - b*FyR) / Iz;
	double dUx = Uy * r + (FxF + FxR - FyF * delta) / m;

	double de; double ds; double dDeltaPsi;

	if (matchType == EULER){
		de = Uy * cos(deltaPsi) + Ux * sin(deltaPsi);
		ds = Ux * cos(deltaPsi) - Uy * sin(deltaPsi);
		dDeltaPsi = r - K  * Ux;
	}

	double dX = - Uy * cos(psi) - Ux * sin(psi);
	double dY =   Ux * cos(psi) - Uy * sin(psi);
	double dotPsi = r; 

	//Update states with Euler integration
	Uy = Uy + ts * dUy;
	r  = r + ts * dr;
	Ux = Ux + ts * dUx;
	X = X + ts*dX;
	Y = Y + ts*dY;
	psi = psi + ts*dotPsi;

	//For Euler integration, update states with ODEs
	if (matchType == EULER){
		e = e + ts*de;
		s = s + ts*ds;
		deltaPsi = deltaPsi + ts * dDeltaPsi; 
	}

	//update local state and global state
	localState.e = e; localState.s = s; localState.dPsi = deltaPsi; 
	localState.Ux = Ux; localState.Uy = Uy; localState.r = r; 
	globalState.X = X; globalState.Y = Y; globalState.Psi = psi; 

}

void Simulation::getFx(double& FxF,double& FxR,const double FxDes,const double Ux){
	//implement engine and brake limits
	double Fx; 
	if (FxDes > 0){ 
		if (Ux == 0){
			Fx = FxDes;
		}
		else{
			Fx = std::min(this->vehicle.powerLimit / Ux - 0.7 * pow(Ux, 2) - 300, FxDes);
		}
	}

	else{
		Fx = FxDes;
	}

	//Distribute according to weight
	FxF = Fx * this->vehicle.b / this->vehicle.L; 
	FxR = Fx * this->vehicle.a / this->vehicle.L; 
}

void Simulation::getNormalForces(double& FzF,double& FzR, const double Fx){
	if (this->wtType == NONE){
		//return the static normal forces
		FzF = this->vehicle.FzF;
		FzR = this->vehicle.FzR; 

	}
	else if (wtType == STEADYSTATE){
		double L = vehicle.a + vehicle.b; 
		double m = this->vehicle.m; 
		double a = this->vehicle.a;
		double b = this->vehicle.b; 
		double g = this->vehicle.g; 
		double h = this->vehicle.h; 

		FzF = 1 / L * (m*b*g - h * Fx); 
		FzR = 1 / L * (m*a*g + h * Fx);
	}

	else{
		std::cerr<<"invalid wt transfer type specified"<<std::endl;
	}
}

void Simulation::getSlips(double& alphaF,double& alphaR,const LocalState_T localState,const ControlInput controlInput){
	double Ux = localState.Ux;
	double Uy = localState.Uy; 
	double r = localState.r;
	double delta = controlInput.delta;

	if (Ux < 2.0){
		alphaF = 0; //speed too low to get slip estimate
		alphaR = 0;
	}
	else{
		alphaF = atan( (Uy + this->vehicle.a * r) / Ux ) - delta;
		alphaR = atan( (Uy - this->vehicle.b * r) / Ux ); 
	}
}
