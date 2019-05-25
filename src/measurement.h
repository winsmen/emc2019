#ifndef MEASUREMENT
#define MEASUREMENT

#include <emc/io.h>

class Measurement
{
    emc::IO io;
    emc::LaserData scan;
    emc::OdometryData odom;
    struct LRFpoint;
    double ang_inc;
    int center_index;
    int right_index;
    int left_index;
    LRFpoint center,right,left;
public:
    Measurement(emc::IO &i, emc::LaserData &s, emc::OdometryData &o);
    int measure();
};

struct Measurement::LRFpoint
{
    double d;   //Distance
    double a;   //Angle
    int i;      //Index
    LRFpoint(double d, int i)
    {
        LRFpoint::d = d;
        LRFpoint::i = i;
        a = (i-center_index)*ang_inc;
    }
    LRFpoint()
    {
        d = -1;
        a = -1;
        i = -1;
    }
};

Measurement::Measurement(emc::IO &i, emc::LaserData &s, emc::OdometryData &o)
{
    io = i;
    scan = s;
    odom = o;
}

int Measurement::measure()
{
    /* Return values:
     * 1  - Success
     * 0  - LRF read failed, Odometer read success
     * -1 - LRF read success, Odometer read failed
     * -2 - LRF and Odometer read failed*/
    int ret = 1;
    if (io.readLaserData(scan))
    {
        dist_center = scan.ranges[center];
        dist_right = scan.ranges[right];
        dist_left = scan.ranges[left];
        getMaxMinDist();
    }
    else
        ret = 0;
    if (io.readOdometryData(odom))
        angle = odom.a;
    else
        ret -= 2;
    return ret;
}

#endif // MEASUREMENT

