#include "sim_types.h"
#define _USE_MATH_DEFINES
#include <math.h>

void loadVehicleShelley(Vehicle_T& vehicle){
    vehicle.a = 1.0441; //CG to front wheelbase [m]
    vehicle.b = 1.4248; //CG to rear wheelbase [m] 
    vehicle.d = 1.50;   //vehicle width, meters (used for plotting only)
    vehicle.rW = 0.34; //wheel radius, meters (used for plotting only)
    vehicle.m = 1512.4; //vehicle mass (kg)
    vehicle.Cf = 160000.0; //vehicle cornering stiffness (N)
    vehicle.Cr = 180000.0; //vehicle cornering stiffness (N)
    vehicle.Iz  = 2.25E3;  //vehicle inertia (kg  m^2)
    vehicle.muF = 0.97;     //front friction coeff
    vehicle.muR = 1.02;    //rear friction coeff
    vehicle.g = 9.81;      //m/s^2, accel due to gravity
    vehicle.L = vehicle.a + vehicle.b; //total vehicle length, m
    vehicle.FzF = vehicle.m*vehicle.b*vehicle.g/vehicle.L;   //Maximum force on front vehicles
    vehicle.FzR = vehicle.m*vehicle.a*vehicle.g/vehicle.L;   //Maximium force on rear vehicles
    vehicle.h = 0.55;   //Distance from the ground
    vehicle.brakeTimeDelay = 0.25; //Seconds
    vehicle.rollResistance = 255.0; //Newtons
    vehicle.powerLimit = 160000.0; //Watts
    vehicle.dragCoeff = 0.3638; //N / (m/s)^2
    vehicle.deltaLim = 27. * M_PI / 180;  //Steering limit, radians
    vehicle.beta = 2.0; //ratio of front to wheel rear torque - used currently for vp gen
    vehicle.brakeFactor = 0.95; //parameter needed for velocity profile generation
}

