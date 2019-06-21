#ifndef MEASUREMENT
#define MEASUREMENT

#include <emc/io.h>
#include <fstream>
#include <iostream>
#include <ctime>
#include "common_resources.h"

#ifndef MEASURE_LOG_FLAG
#define MEASURE_LOG_FLAG    1
#endif

using namespace std;

class Measurement
{
    // Operational variables
    emc::IO &io;
    double ang_inc;
    int scan_span;
    const int side_range;
    const int padding;
    const int av_range;
    const int min_range;
    double min_angle;
    double max_angle;
    const float min_permit_dist;
    ofstream measure_log;

    //Measured variables
    World &world;

    // Private Member Functions
    void getMaxMinDist();
    void log(string text);

public:
    Measurement(emc::IO *io, World *world,
                const Performance specs);
    ~Measurement();
    int measure();
    int sectorClear(int i);
    double alignedToWall(int side);
    double getAngInc();
    double getMinAngle();
    double getMaxAngle();
};


Measurement::Measurement(emc::IO *io, World *world,
                         const Performance specs)
    : io(*io), world(*world),
      side_range(specs.side_range), padding(specs.padding), av_range(specs.av_range),
      min_range(specs.min_range), min_permit_dist(specs.min_permit_dist)
{
    ang_inc = world->scan.angle_increment;
    scan_span = world->scan.ranges.size();
    min_angle = world->scan.angle_min;
    max_angle = world->scan.angle_max;
    int center_index = (scan_span-1)/2;
    int right_index = center_index - (M_PI/2)/ang_inc;
    int left_index = center_index + (M_PI/2)/ang_inc;
    this->world.center.assignPoint(world->scan.ranges[center_index],center_index);
    this->world.right.assignPoint(world->scan.ranges[right_index],right_index);
    this->world.left.assignPoint(world->scan.ranges[left_index],left_index);
    measure_log.open("../logs/measure_log.txt", ios::out | ios::trunc);
    time_t now = time(0);
    log("Measurment Log: " + string(ctime(&now)));
    log("scan_span: " + to_string(scan_span));
    log("ang_inc: " + to_string(ang_inc));
    log("side_range: " + to_string(side_range));
    log("padding: " + to_string(padding));
    log("av_range: " + to_string(av_range));
    log("min_range: " + to_string(min_range));
    log("min_permit_dist: " + to_string(min_permit_dist));
    log("center, right, left, farthest, nearest, front_clear, right_clear, left_clear, odom.x, odom.y, odom.theta");
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
    if (io.readLaserData(world.scan))
    {
        world.center.d = world.scan.ranges[world.center.i];
        world.right.d = world.scan.ranges[world.right.i];
        world.left.d = world.scan.ranges[world.left.i];
        getMaxMinDist();
    }
    else
        ret = 0;
    if (io.readOdometryData(world.odom))
    {
        world.theta = world.odom.a + world.theta_off;
        world.x = world.odom.x + world.x_off;
        world.y = world.odom.y + world.y_off;
    }
    else
        ret -= 2;
    log(to_string(world.center.d)+','+to_string(world.right.d)+','+to_string(world.left.d)+','+
        to_string(world.farthest.d)+','+to_string(world.nearest.d)+','+
        to_string(world.front_clear)+','+to_string(world.right_clear)+','+to_string(world.left_clear) + ',' +
        to_string(world.x) + ',' + to_string(world.y) + ',' + to_string(world.theta));
    return ret;
}


void Measurement::getMaxMinDist()
{
    world.farthest.assignPoint(world.scan.ranges[0],world.scan.range_min);
    world.nearest.assignPoint(world.scan.ranges[0],world.scan.range_max);
    world.front_clear = true;
    world.right_clear = true;
    world.left_clear = true;
    for (int i = padding; i < scan_span-padding; ++i)
    {
        if (world.scan.ranges[i] > world.farthest.d)
        {
            world.farthest.assignPoint(world.scan.ranges[i],i);
        }
        else if (world.scan.ranges[i] < world.nearest.d && world.scan.ranges[i] > min_range)
        {
            world.nearest.assignPoint(world.scan.ranges[i],i);
        }
        if (abs(i - world.right.i) < side_range)
        {
            if (world.scan.ranges[i] < min_permit_dist && world.scan.ranges[i] > min_range)
                world.right_clear = false;
        }
        else if (abs(i - world.left.i) < side_range)
        {
            if (world.scan.ranges[i] < min_permit_dist && world.scan.ranges[i] > min_range)
            {
                world.left_clear = false;
            }
        }
        else if (abs(i - world.center.i) < side_range)
        {
            if (world.scan.ranges[i] < min_permit_dist && world.scan.ranges[i] > min_range)
            {
                world.front_clear = false;
            }
        }
    }
}


int Measurement::sectorClear(int i = 499)
{
    /* Return values:
     * 0 -> Not clear, object < min_permit_dist
     * 1 -> Clear
     * 2 -> Caution, object < 2*min_permit_dist
     * 3 -> Out of vision range_error
     */
    if (i < padding+av_range || i > scan_span-padding-av_range)
        return 3;
    for (int j = 0; j < side_range; ++j)
    {
        if (world.scan.ranges[i+j] < min_permit_dist/cos(j*ang_inc) ||
                world.scan.ranges[i-j] < min_permit_dist/cos(j*ang_inc))
            return 0;
        if (world.scan.ranges[i+j] < 2*min_permit_dist/cos(j*ang_inc) ||
                world.scan.ranges[i-j] < 2*min_permit_dist/cos(j*ang_inc))
            return 2;
    }
    return 1;
}


double Measurement::alignedToWall(int side = RIGHT)
{
    int i = side==RIGHT?world.right.i:side==LEFT?world.left.i:world.center.i;
    const double compare_length = 0.3; //Check for 0.5m length
    int range = atan(compare_length/world.scan.ranges[i])/ang_inc;
    double diff = 0;
    int count = 0;
    for(int j = 0; j < range; ++j)
    {
        if ((i+j) < scan_span)
        {
            if (world.scan.ranges[i+j] > min_range)
            {
                diff += fabs(world.scan.ranges[i+j] - world.scan.ranges[i]/cos(j*ang_inc));
                count += 1;
            }
        }
        if ((i-j) > 0)
        {
            if (world.scan.ranges[i-j] > min_range)
            {
                diff += fabs(world.scan.ranges[i-j] - world.scan.ranges[i]/cos(j*ang_inc));
                count += 1;
            }
        }
    }
    return diff/count;
}

double Measurement::getAngInc()
{
    return ang_inc;
}

double Measurement::getMinAngle()
{
    return min_angle;
}

double Measurement::getMaxAngle()
{
    return max_angle;
}

void Measurement::log(string text)
{
#if MEASURE_LOG_FLAG == 1 || MEASURE_LOG_FLAG == 3
    measure_log << text << endl;
#endif
#if MEASURE_LOG_FLAG == 2 || MEASURE_LOG_FLAG == 3
    cout << "sense: " << text << endl;
#endif
}

#endif // MEASUREMENT
