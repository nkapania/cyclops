#pragma once
#include "world.h"
//#include "speed_profiles.h"
#include "sim_types.h"

// Defines a class that implements the lanekeeping controller from
// Nitin Kapania's thesis

class LanekeepingController{
private:
    
    World world;
    Vehicle_T vehicle;
    //BasicProfile profile;
    double xLA; //lookahead distance, meters
    double kLK; //proportional gain, rad / meter
    double kSpeed; //speed proportional gaiin - N / (m/s)
    double alphaFlim; //steering limits for feedforward
    double alphaRlim; //steering limits for feedworward
    int numTableValues;

    //values where car is sliding
    double alphaFslide; 
    double alphaRslide; 

    std::vector<double> alphaFtable; 
    std::vector<double> alphaRtable;
    std::vector<double> FyFtable;
    std::vector<double> FyRtable; 

    double getDeltaFB(LocalState_T& localState,const double betaFFW);
    double speedTracking(LocalState_T& localState);
    double getDeltaFFW(LocalState_T& localState,double& betaFFW, double K);

public:

    //Constructor
    LanekeepingController(World& world, Vehicle_T& vehicle); // BasicProfile& profile); 
    AuxVars_T updateInput(LocalState_T& localState, ControlInput& controlInput);

    //Destructor
    ~LanekeepingController();

 

    };