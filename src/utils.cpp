#include "utils.h"
#include <algorithm>

/* code below from stack overflow for reading from csv */

std::string const& CSVRow::operator[](std::size_t index) const
        {
            return m_data[index];
        }

std::size_t CSVRow::size() const
        {
            return m_data.size();
        }

void CSVRow::readNextRow(std::istream& str)
        {
            std::string         line;
            std::getline(str, line);

            std::stringstream   lineStream(line);
            std::string         cell;

            m_data.clear();
            while(std::getline(lineStream, cell, ','))
            {
                m_data.push_back(cell);
            }
            // This checks for a trailing comma with no data after it.
            if (!lineStream && cell.empty())
            {
                // If there was a trailing comma then add an empty element.
                m_data.push_back("");
            }
        }



std::istream& operator>>(std::istream& str, CSVRow& data)
{
    data.readNextRow(str);
    return str;
}   


/* Converts desired tire force to slip angle using interp function */
double force2alpha(std::vector<double>& forceTable, std::vector<double>&alphaTable, double Fdes){
    double Fmax =  *std::max_element(forceTable.begin(), forceTable.end());
    double Fmin =  *std::min_element(forceTable.begin(), forceTable.end());

    if (Fdes > Fmax){
        Fdes = Fmax - 1;
    }
    else if(Fdes < Fmin){
        Fdes = Fmin + 1;
    }

    double alphaDes; 
    interpolate1D(forceTable, alphaTable, alphaTable.size(), Fdes, alphaDes);

    return alphaDes;
}



/* Performs a binary search and returns the index of the element closest but not exeeding xq in vector x. Assumes xq is
 * within bounds of the vector and that x is monotonically increasing.
 * Templated to be called with any Container type that supports operator[], size is the number of elements in container.
 */
template<typename Container>
size_t binarySearch(const Container& x, size_t size, double xq) {
    size_t low_ind = 0;
    size_t high_ind = size-1;
    size_t mid_ind;

    while (true) {
        mid_ind = floor((low_ind+high_ind)/2);

        if (low_ind > high_ind) {
            std::cerr << "Search Error" << std::endl;
            return -1;
        }
        // check if we are done
        if ( (high_ind-low_ind) == 1 ) {
            break;
        }
        if (x[mid_ind] < xq) {
            low_ind = mid_ind;
        } else {
            high_ind = mid_ind;
        }
    }
    return low_ind;
}

/* 1D interpolation
 * x is of type Container, which supports operator[]
 * y is the array of values which correspond to those in x
 * size is the number of elements in each of x and y. x[size-1] is the last element in x
 * xq is the x query value
 * yq is the y query value
 * InterpData_T is returned, allowing constant time interpolation calls to replace:
 *      "interpolate1D(x, other_y, size, xq, other_yq)"
 * with:
 *      "constTimeInterp(InterpData, other_y, other_yq)"
 */
template<typename Container1, typename Container2>
InterpData_T interpolate1D(const Container1& x, const Container2& y, size_t size, const double xq, double& yq) {
    // first check if query point exceeds bounds of vector, saturate if so
    if (xq <= x[0]) {
        yq = y[0];
        return InterpData_T(0, 0);
    } else if (xq >= x[size-1]) {
        yq = y[size-1];
        return InterpData_T(size-2, 1);
    }

    // find lower index
    size_t low_ind = binarySearch(x, size,  xq);
    double frac = (xq-x[low_ind])/(x[low_ind+1]-x[low_ind]);
    InterpData_T interp(low_ind, frac);

    constTimeInterp(interp, y, yq);
    return interp;
}

//Note - this is a poor man's linspace function, not 
// robustly tested - C++ does not appear to have a 
// standard version of this. Do not use outside of this library. 

std::vector<double> linspace(double a, double b, int n) {
    std::vector<double> array;
    if (a > b){
        std::cerr<<"linspace error - a should be less than b"<<std::endl;
    }

    double step = (b-a) / (n-1);

    while(a <= b) {
        array.push_back(a);
        a += step;           // could recode to better handle rounding errors
    }
    return array;
}

// Sign function
template <typename T> int sgn(T val) 
{
    return (T(0) < val) - (val < T(0));
}


/* calculate constant time interpolation */
template<typename Container>
void constTimeInterp(const InterpData_T& interp, const Container& y, double& yq) {
    yq = y[interp.low_idx] + interp.frac*(y[interp.low_idx+1]-y[interp.low_idx]);
}