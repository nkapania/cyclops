#include "world.h"
#include "utils.h"
#include <string>
#include <vector>

void World::loadFromCSV(){


    std::ifstream       file("../maps/simpleRace.csv");
    CSVRow              row;

    while(file >> row)
    {
    	   this->s.push_back(stod(row[0], NULL));
    	   this->curvature.push_back(stod(row[1], NULL));
         this->posE.push_back(stod(row[2], NULL));
         this->posN.push_back(stod(row[3], NULL));
         this->roadPsi.push_back(stod(row[4], NULL));
    }

    //convert to doubles

    this->isOpen = false; //hardcoded for now


}

