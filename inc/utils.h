#pragma once
#include <math.h> //floor
#include <iostream>
#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>


/* code below from stack overflow for reading from csv */
class CSVRow
{
    public:
        std::string const& operator[](std::size_t index) const;
        std::size_t size() const;
        void readNextRow(std::istream& str);
    private:
        std::vector<std::string>    m_data;
};

std::istream& operator>>(std::istream& str, CSVRow& data);



/* Performs a binary search and returns the index of the element closest but not exeeding xq in vector x. Assumes xq is
 * within bounds of the vector and that x is monotonically increasing.
 * Templated to be called with any Container type that supports operator[], size is the number of elements in container.
 */
template<typename Container>
size_t binarySearch(const Container& x, size_t size, double xq);

/* interpolation helper struct for constant time interpolation */
struct InterpData_T {
    size_t low_idx;
    double frac;
    InterpData_T(size_t low, double frac_): low_idx(low), frac(frac_) {}
    InterpData_T():                         low_idx(0),   frac(0)     {}
};

template<typename Container1, typename Container2>
InterpData_T interpolate1D(const Container1& x, const Container2& y, size_t size, const double xq, double& yq);


/* calculate constant time interpolation */
template<typename Container>
void constTimeInterp(const InterpData_T& interp, const Container& y, double& yq);

std::vector<double> linspace(double a, double b, int n);

template <typename T> int sgn(T val);

double force2alpha(std::vector<double>& forceTable, std::vector<double>&alphaTable, double Fdes);

double crossSign(double u1, double u2, double v1, double v2);
