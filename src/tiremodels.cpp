#include <cmath>
#include <vector>
#include "utils.h"
#include "sim_types.h"

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

void coupledTire(double& Fy, double& zeta, const double alpha, const double Fx,
	const double Fz, const double muS, const double muP,const double C){

	if (pow(muP * Fz, 2) > pow(Fx, 2)){
		double arg = std::fmax( pow((muP*Fz), 2) - pow(Fx, 2), 0); 
		zeta = std::sqrt(arg) / (muP*Fz);

		}
	else zeta = 0;

	double alphaSlide = std::abs( atan( 3 * zeta * muP * Fz / C) );

	//use fiala model if not sliding

	if (std::abs(alpha) < alphaSlide){
		double linearTerm = - C *tan(alpha); 
		double quadTerm =  pow(C, 2) * (2 - muS/muP) * std::abs(tan(alpha))*tan(alpha) / (3 * zeta * muP * Fz);
		double cubicTerm = - pow(C, 3) *pow(tan(alpha),3) * (1 - 2*muS / (3*muP) ) /( 9* pow(muP , 2) * pow(zeta, 2) * pow(Fz, 2) );
		Fy = linearTerm + quadTerm + cubicTerm; 
	}

	else{
		Fy = -zeta*muS*Fz*sgn(alpha);
	}
}


void coupledTireForces(double& FyF,double& FyR,double& zetaF,double& zetaR,
 const double alphaF,const double alphaR, const double FxF,const double FxR,
  const double FzF,const double FzR,const Vehicle_T& vehicle){

  	double muF = vehicle.muF; 
  	double muR = vehicle.muR; 

  	double Cf = vehicle.Cf; 
  	double Cr = vehicle.Cr; 

  	coupledTire(FyF, zetaF, alphaF, FxF, FzF, muF, muF, Cf); 
  	coupledTire(FyR, zetaR, alphaR, FxR, FzR, muR, muR, Cr); 
}