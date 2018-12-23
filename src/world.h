#pragma once
#include <vector>
#include <string>

class World{
public:
	std::vector<double> roadIC, posE, posN, roadPsi, curvature, s;
	bool isOpen;

	void loadFromCSV();
	void loadFromMAT();
	void genFromEN();
	void genFromSK();
	void resample();

};
