#pragma once
#include "world.h"
#include "sim_types.h"
#include <vector>

// Defines a velocity profile class


class BasicProfile{
private:
    Vehicle_T vehicle;
    World world;
    std::vector<double> s;
    std::vector<double> Ux ;
    std::vector<double> Ax;
    double mu;
    double vMax;
    double axMax;

    void generateBasicProfileOpen();
    void generateBasicProfileClosed();


public:

    // Constructor
    BasicProfile(Vehicle_T& vehicle, World& world, double friction = 0.3, double vMax = 10.0, double AxMax = 9.91);

    // Destructor
    ~BasicProfile();
};