#include <iostream>
#include "sim_types.h"
#include "world.h"
#include "speed_profiles.h"
#include <string>
#include "matplotlibcpp.h"

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

	plt::plot(speedProfile.s, speedProfile.Ux);
	plt::show();

	plt::plot(speedProfile.s, speedProfile.Ax);
	plt::show();

	//std::cout << speedProfile << std::endl;

	return 0;
}


