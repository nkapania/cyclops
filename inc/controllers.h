#pragma once
#include "world.h"
#include "speed_profiles.h"
#include "sim_types.h"


// Defines a class that implements the lanekeeping controller from
// Nitin Kapania's thesis

class LanekeepingController{
private:
    
    World world;
    Vehicle_T vehicle;
    BasicProfile profile;
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

    double getDeltaFB(const LocalState_T& localState,const double betaFFW);
    double speedTracking(const LocalState_T& localState, double& UxDes, double& AxDes, double& FxFFW, double& FxFB);
    double getDeltaFFW(const LocalState_T& localState,double& betaFFW,const double K, double& FyFdes, double& FyRdes, double& alphaFdes, double& alphaRdes);
    double lanekeeping(const LocalState_T& localState, double& deltaFFW, double& deltaFB, double& K, double& alphaFdes, double& alphaRdes, double& betaFFW); 

public:

    //Constructors
    LanekeepingController(World& world, Vehicle_T& vehicle, BasicProfile& profile); 
    LanekeepingController();

    AuxVars_T updateInput(const LocalState_T& localState, ControlInput& controlInput);

    //Destructor
    ~LanekeepingController();

 

    };