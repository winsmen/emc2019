#ifndef MEASUREMENT
#define MEASUREMENT

#include <emc/io.h>
#include <fstream>
#include "common_resources.h"


class Measurement
{
    // Operational variables
    emc::IO io;
    emc::LaserData scan;
    emc::OdometryData odom;
    double ang_inc;
    int scan_span;
    int side_range;
    int padding;
    int av_range;
    int min_range;
    float min_permit_dist;
    ofstream measure_log;

    //Measured variables
    LRFpoint center, right, left;
    double angle;
    LRFpoint farthest, nearest;
    bool front_clear,right_clear,left_clear;

    // Private Member Functions
    void getMaxMinDist();

public:
    Measurement(emc::IO &io, emc::LaserData &scan, emc::OdometryData &odom,
                int side_range, int padding, int av_range, int min_range, float min_permit_dist);
    ~Measurement();
    int measure();
};


Measurement::Measurement(emc::IO &io, emc::LaserData &scan, emc::OdometryData &odom,
                         int side_range, int padding, int av_range, int min_range, float min_permit_dist)
    : io(io), scan(scan), odom(odom),
      side_range(side_range), padding(padding), av_range(av_range), min_range(min_range), min_permit_dist(min_permit_dist)
{
    int max_iter = 20;
    while(!io.readLaserData(scan) || !io.readOdometryData(odom) && max_iter > 0)
        --max_iter;
    ang_inc = scan.angle_increment;
    scan_span = scan.ranges.size();
    int center_index = (scan_span-1)/2;
    int right_index = center_index - (M_PI/2)/ang_inc;
    int left_index = center_index + (M_PI/2)/ang_inc;
    center.assignPoint(scan.ranges[center_index],center_index);
    right.assignPoint(scan.ranges[right_index],right_index);
    left.assignPoint(scan.ranges[left_index],left_index);
    measure_log.open("measure_log.txt", ios::out | ios::trunc);
    measure_log << "Measurment Log: <date> <time>" << endl;
}

Measurement::~Measurement()
{
    measure_log << "End of Log" << endl;
    measure_log.close();
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
        center.d = scan.ranges[center.i];
        right.d = scan.ranges[right.i];
        left.d = scan.ranges[left.i];
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


void Measurement::getMaxMinDist()
{
    farthest.assignPoint(scan.ranges[0],scan.range_min);
    nearest.assignPoint(scan.ranges[0],scan.range_max);
    front_clear = true;
    right_clear = true;
    left_clear = true;
    for (int i = padding; i < scan_span-padding; ++i)
    {
        if (scan.ranges[i] > farthest.d)
        {
            farthest.assignPoint(scan.ranges[i],i);
        }
        else if (scan.ranges[i] < nearest.d && scan.ranges[i] > min_range)
        {
            nearest.assignPoint(scan.ranges[i],i);
        }
        if (abs(i - right.i) < side_range)
        {
            if (scan.ranges[i] < min_permit_dist && scan.ranges[i] > min_range)
                right_clear = false;
        }
        else if (abs(i - left.i) < side_range)
        {
            if (scan.ranges[i] < min_permit_dist && scan.ranges[i] > min_range)
            {
                left_clear = false;
            }
        }
        else
        {
            if (scan.ranges[i] < min_permit_dist && scan.ranges[i] > min_range)
            {
                front_clear = false;
            }
        }
    }
}

#endif // MEASUREMENT
