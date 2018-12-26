# cyclops
Implementation of path following simulation framework in C++

#build instructions
g++ -o main main.cpp load_vehicle.cpp world.cpp speed_profiles.cpp controllers.cpp tiremodels.cpp utils.cpp -std=c++0x -I/usr/include/python2.7 -lpython2.7
