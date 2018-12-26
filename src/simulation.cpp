#include "simulation.h"
#include <iostream>
#include <cmath>

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
 	this-> matchType = NO;
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




