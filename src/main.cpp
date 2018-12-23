#include <iostream>
#include "sim_types.h"
#include "world.h"
#include "speed_profiles.h"
#include <string>

int main()
{

	Vehicle_T vehicle;
	World world;
	std::string fileName="../maps/simpleRace.csv";

	loadVehicleShelley(vehicle);
	world.loadFromCSV(fileName);

	double friction = 0.90;
	double vMax = 99.0;

	//BasicProfile speedProfile(vehicle, world, friction, vMax);

	std::cout << world.s[100] << std::endl;

	return 0;
}


