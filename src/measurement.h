#ifndef MEASUREMENT
#define MEASUREMENT

#include <emc/io.h>
#include <fstream>
#include <iostream>
#include <ctime>
#include "common_resources.h"

#ifndef LOG_FLAG
#define LOG_FLAG    3
#endif

using namespace std;

class Measurement
{
    // Operational variables
    emc::IO io;
    emc::LaserData scan;
    emc::OdometryData odom;
    double ang_inc;
    int scan_span;
    const int &side_range;
    const int &padding;
    const int &av_range;
    const int &min_range;
    const float &min_permit_dist;
    ofstream measure_log;

    //Measured variables
    World world;

    // Private Member Functions
    void getMaxMinDist();
    void log(string text);

public:
    Measurement(emc::IO &io, emc::LaserData &scan, emc::OdometryData &odom, World &world,
                const Performance &specs);
    ~Measurement();
    int measure();
};


Measurement::Measurement(emc::IO &io, emc::LaserData &scan, emc::OdometryData &odom, World &world,
                         const Performance &specs)
    : io(io), scan(scan), odom(odom), world(world),
      side_range(specs.side_range), padding(specs.padding), av_range(specs.av_range),
      min_range(specs.min_range), min_permit_dist(specs.min_permit_dist)
{
    int max_iter = 20;
    while(!io.readLaserData(scan) || !io.readOdometryData(odom) && max_iter > 0)
        --max_iter;
    ang_inc = scan.angle_increment;
    scan_span = scan.ranges.size();
    int center_index = (scan_span-1)/2;
    int right_index = center_index - (M_PI/2)/ang_inc;
    int left_index = center_index + (M_PI/2)/ang_inc;
    world.center.assignPoint(scan.ranges[center_index],center_index);
    world.right.assignPoint(scan.ranges[right_index],right_index);
    world.left.assignPoint(scan.ranges[left_index],left_index);
    measure_log.open("../measure_log.txt", ios::out | ios::trunc);
    time_t now = time(0);
    log("Measurment Log: " + string(ctime(&now)));
    log("center, right, left, farthest, nearest, front_clear, right_clear, left_clear");
}

Measurement::~Measurement()
{
    log("End of Log");
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
        world.center.d = scan.ranges[world.center.i];
        world.right.d = scan.ranges[world.right.i];
        world.left.d = scan.ranges[world.left.i];
        getMaxMinDist();
    }
    else
        ret = 0;
    if (io.readOdometryData(odom))
        world.angle = odom.a;
    else
        ret -= 2;
    log(to_string(world.center.d)+','+to_string(world.right.d)+','+to_string(world.left.d));
    log(to_string(world.farthest.d)+','+to_string(world.nearest.d));
    log(to_string(world.front_clear)+','+to_string(world.right_clear)+','+to_string(world.left_clear));
    return ret;
}


void Measurement::getMaxMinDist()
{
    world.farthest.assignPoint(scan.ranges[0],scan.range_min);
    world.nearest.assignPoint(scan.ranges[0],scan.range_max);
    world.front_clear = true;
    world.right_clear = true;
    world.left_clear = true;
    for (int i = padding; i < scan_span-padding; ++i)
    {
        if (scan.ranges[i] > world.farthest.d)
        {
            world.farthest.assignPoint(scan.ranges[i],i);
        }
        else if (scan.ranges[i] < world.nearest.d && scan.ranges[i] > min_range)
        {
            world.nearest.assignPoint(scan.ranges[i],i);
        }
        if (abs(i - world.right.i) < side_range)
        {
            if (scan.ranges[i] < min_permit_dist && scan.ranges[i] > min_range)
                world.right_clear = false;
        }
        else if (abs(i - world.left.i) < side_range)
        {
            if (scan.ranges[i] < min_permit_dist && scan.ranges[i] > min_range)
            {
                world.left_clear = false;
            }
        }
        else if (abs(i - world.center.i) < side_range)
        {
            if (scan.ranges[i] < min_permit_dist && scan.ranges[i] > min_range)
            {
                world.front_clear = false;
            }
        }
    }
}

void Measurement::log(string text)
{
#if LOG_FLAG == 1 || LOG_FLAG == 3
    measure_log << text << endl;
#endif
#if LOG_FLAG == 2 || LOG_FLAG == 3
    cout << text << endl;
#endif
}

#endif // MEASUREMENT
