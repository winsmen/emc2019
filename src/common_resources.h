#ifndef COMMON_RESOURCES_H
#define COMMON_RESOURCES_H

#include <sstream>
#include <string.h>

using namespace std;


template<typename T>
string to_string(const T &n);

struct LRFpoint
{
    double d;   //Distance
//    double a;   //Angle
    int i;      //Index
    LRFpoint(double d, int i)
    {
        LRFpoint::d = d;
        LRFpoint::i = i;
//        a = (i-center)*ang_inc;
    }
    LRFpoint()
    {
        d = -1;
//        a = -1;
        i = -1;
    }
    void assignPoint(double d, int i)
    {
        LRFpoint::d = d;
        LRFpoint::i = i;
//        a = (i-center)*ang_inc;
    }
};

// define to_string function
template<typename T>
string to_string(const T& n)
{
    ostringstream ss;
    ss << n;
    return ss.str();
}

void polar2cart(double r,double theta, double &x,double &y, double x_off = 0, double y_off = 0)
{
    x = x_off + r*sin(theta)*80;
    y = y_off - r*cos(theta)*80;
}

#endif // COMMON_RESOURCES_H

