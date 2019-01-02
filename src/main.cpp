#include <iostream>
#include "sim_types.h"
#include "world.h"
#include "speed_profiles.h"
#include <string>
#include "controllers.h"
#include "simulation.h"
#include "matplotlibcpp.h"

int main()
{
	namespace plt = matplotlibcpp;
	Vehicle_T vehicle;
	World world;
	//std::string fileName="/home/nkapania/cyclops/maps/simpleRace.csv";
	std::string fileName = "/home/nkapania/cyclops/maps/thunderhill_race_flatCSV.csv";

	loadVehicleShelley(vehicle);
	world.loadFromCSV(fileName);

	double friction = 0.6;
	double vMax = 99.0;

	BasicProfile speedProfile(vehicle, world, friction, vMax);

	LanekeepingController controller(world, vehicle, speedProfile);
	Simulation simulation(world, vehicle, controller, speedProfile);
	SimOutput_T simOut = simulation.simulate();


	//plot results
	plotResults(simOut, world);

	return 0;
}


