#include <iostream>
#include "sim_types.h"
#include "world.h"

int main()
{

	Vehicle_T vehicle;
	World world;

	loadVehicleShelley(vehicle);
	world.loadFromCSV();

	std::cout << world.s.size() << std::endl;

	return 0;
}


