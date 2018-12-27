#pragma once
#include <vector>


std::vector<double> fiala(double C,double muP,double muS, std::vector<double> alpha, double Fz);
void coupledTireForces(double& FyF,double& FyR,double& zetaF,double& zetaR, const double alphaF,const double alphaR, const double FxF,const double FxR, const double FzF,const double FzR,const Vehicle_T& vehicle);
