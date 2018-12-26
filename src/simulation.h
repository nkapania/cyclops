#pragma once
#include "world.h"
#include "sim_types.h"
#include "speed_profiles.h"
#include "controllers.h"

// Defines a class that implements the main simulation functionality

class Simulation{
private:
    

    enum Physics
    {
        BICYCLE //TO DO: add support for more physics
    };

    enum WeightTransferType
    {
        NONE,
        STEADYSTATE
    };

    enum Tires
    {
        FIALA,
        COUPLED,
        LINEAR
    };

    enum MapMatchType
    {
        NO,
        EMBED,
        CLOSEST
    };


    World world;
    Vehicle_T vehicle;
    BasicProfile profile;
    LanekeepingController controller;
    bool isRunning;
    Physics physics;
    double ts;
    MapMatchType matchType;
    double maxTime;
    double vStart;
    WeightTransferType wtType;
    Tires tires;
    double desiredLaps;
    double lapNumber;
    double last_s; 
    double maxE; 
    double buffer;

    void incrementLapNumber(const double s);
    void checkForTermination(const LocalState_T& localState,const int counter);
    void printStatus(const LocalState_T& localState, const int counter);
    void updateState(const ControlInput& controlInput, LocalState_T& localState, GlobalState_T& globalState, AuxVars_T& auxVars);
    void bicycleModel(const ControlInput& controlInput, LocalState_T& localState, GlobalState_T& globalState, const double K);



public:

    //Constructor
    Simulation(World& world, Vehicle_T& vehicle, LanekeepingController& controller, BasicProfile& profile); 
    void simulate();


    //Destructor
    ~Simulation();

    };