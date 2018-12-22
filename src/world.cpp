#include "world.h"
#include "utils.h"
#include <string>
#include <vector>

void World::loadFromCSV(){


    std::ifstream       file("../maps/simpleRace.csv");
    CSVRow              row;

    std::vector<std::string> s;

    while(file >> row)
    {
    	   this->s.push_back(row[0]);
    	   this->curvature.push_back(row[1]);
           this->posE.push_back(row[2]);
           this->posN.push_back(row[3]);
           this->roadPsi.push_back(row[4]);
    }


}

