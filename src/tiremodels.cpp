#include <cmath>
#include <vector>
#include "utils.h"


//To do. Put this in utils. 
template <typename T> int sgn(T val) 
{
    return (T(0) < val) - (val < T(0));
}


std::vector<double> fiala(double C,double muP,double muS,std::vector<double> alpha, double Fz){
	double alphaSlide = std::abs( atan( 3 * muP * Fz / C));

	int N = alpha.size();
	std::vector<double> Fy(N, 0);

	// Use 3rd order polynomial equations when below the tire range
	for (int i = 0; i < N; i++){
		if (std::abs(alpha[i]) < alphaSlide){
			Fy[i] = -C * tan(alpha[i]) + pow(C, 2) / (3 * muP * Fz) * (2 - muS / muP) * tan(alpha[i]) * std::abs(tan(alpha[i])) - pow(C, 3) / (9*pow(muP,2)*pow(Fz, 2))*pow(tan(alpha[i]), 3) * (1-(2*muS)/(3*muP)); 
		}

		else{
			//Use sliding force otherwise
			Fy[i] = -muS * Fz * sgn(alpha[i]);
		}
	}

	return Fy;

}
