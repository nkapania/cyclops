#include <iostream>
#include "sim_types.h"
#include "world.h"
#include "speed_profiles.h"
#include <string>
#include "controllers.h"
#include "simulation.h"

int main()
{
	Vehicle_T vehicle;
	World world;
	std::string fileName="/home/nkapania/cyclops/maps/simpleRace.csv";

	loadVehicleShelley(vehicle);
	world.loadFromCSV(fileName);

	double friction = 0.90;
	double vMax = 99.0;

	BasicProfile speedProfile(vehicle, world, friction, vMax);
	LanekeepingController controller(world, vehicle, speedProfile);
	Simulation simulation(world, vehicle, controller, speedProfile);
	simulation.simulate();

	return 0;
}


