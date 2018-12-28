#pragma once
#include "sim_types.h"
#include "world.h"

class MapMatch{
private:
	World world;
	int seed; //index of map to take a guess
	bool firstSearch;
	double REQUIRED_DISTANCE;
	int MAX_BACKWARD_ITERS;
	int MAX_FORWARD_ITERS;

	MapMatchOutput_T convertToLocal(const double posE,const double posN);

public:
	//constructor and desctructor
	MapMatch(World& world);
	~MapMatch();

	void localize(LocalState_T& localState, const GlobalState_T globalState);

};

