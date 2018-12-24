#include <iostream>
#include "sim_types.h"
#include "world.h"
#include "speed_profiles.h"
#include <string>
#include "matplotlibcpp.h"
#include "controllers.h"

int main()
{

	namespace plt = matplotlibcpp;

	Vehicle_T vehicle;
	World world;
	std::string fileName="../maps/simpleRace.csv";

	loadVehicleShelley(vehicle);
	world.loadFromCSV(fileName);

	double friction = 0.90;
	double vMax = 99.0;

	BasicProfile speedProfile(vehicle, world, friction, vMax);
	LanekeepingController controller(world, vehicle); // speedProfile);

	std::cout << "done" << std::endl;

	return 0;
}


