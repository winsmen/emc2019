#ifndef MAIN
#define MAIN

#include <iostream>
#include <unistd.h>
#include <emc/io.h>
#include <emc/rate.h>
#include "config.h"
#include <math.h>
#include <stdlib.h>

using namespace std;

enum sys_state {
    STARTUP,
    SCAN_FOR_EXIT,
    FIND_WALL,
    ALIGN_TO_WALL,
    FOLLOW_WALL,
    CORNER,
    EXIT_FOUND,
    FOLLOW_CORRIDOR
};

struct robot
{
    emc::IO io;
    emc::LaserData scan;
    emc::OdometryData odom;
    emc::Rate r;
    float maxTrans;
    float maxRot;

    sys_state state;

    double angInc;
    int center;
    int right;
    int left;
    double dist_center;
    double dist_right;
    double dist_left;
    double max_dist;
    double max_dist_dir;
    double min_dist;
    double min_dist_dir;
    double angle;

    robot(int rate, sys_state state, int maxTrans, int maxRot)
        : maxTrans(maxTrans), maxRot(maxRot), r(rate), state(state) {}
    robot(int maxTrans, int maxRot)
        : maxTrans(maxTrans), maxRot(maxRot), r(10), state(state) {}
    robot(int rate)
        : maxTrans(0.5), maxRot(1.2), r(rate), state(state) {}

    bool sensorsReady()
    {
        //Attempt to read data from LFR and Odometer
        if (!io.readLaserData(scan) || !io.readOdometryData(odom))
            return false;
        return true;
    }

    void getMaxMinDist()
    {
        max_dist = min_dist = scan.ranges[0];
        max_dist_dir = min_dist_dir = 0;
        for (int i = 0; i < scan.ranges.size(); ++i)
        {
            if (scan.ranges[i] > max_dist)
            {
                max_dist = scan.ranges[i];
                max_dist_dir = i;
            }
            else if (scan.ranges[i] < min_dist)
            {
                min_dist = scan.ranges[i];
                min_dist_dir = i;
            }
        }
    }

    void startup()
    {
        int maxIter = 20;
        while(!sensorsReady() && maxIter > 0)
            r.sleep();
        angInc = scan.angle_increment;
        center = scan.ranges.size()/2 - 1;
        right = center - (M_PI/2)/angInc - 1;
        left = center + (M_PI/2)/angInc - 1;
        dist_center = scan.ranges[center];
        dist_right = scan.ranges[right];
        dist_left = scan.ranges[left];
        getMaxMinDist();
        angle = odom.a;
    }

    void getSensorData()
    {
        if (io.readLaserData(scan))
        {
            dist_center = scan.ranges[center];
            dist_right = scan.ranges[right];
            dist_left = scan.ranges[left];
        }
        if (io.readOdometryData(odom))
            angle = odom.a;
        //std::cout << dist_center << "\t" << dist_right << "\t" << dist_left << std::endl;
    }

    void determineState()
    {
        //TODO convert constants to performance variables
//        if (min_dist > 0.5)
//            state = FIND_WALL;
//        else if (min_dist < 0.5 && state == FIND_WALL)
//            state = ALIGN_TO_WALL;
//        else if (alignedToRight() && dist_center < 0.5)
//            state = CORNER;
//        else if (alignedToRight())
//            state = FOLLOW_WALL;
//        else if (foundGap())
//            state = EXIT_FOUND;
//        else
        if (state == STARTUP)
            state == SCAN_FOR_EXIT;
//        else if (state == SCAN_FOR_EXIT && exitFound() > -1)
//            state = EXIT_FOUND;

    }

    float exitFound()
    {
        bool found = false;
        float maxSep;
        for (int i = 0; i < scan.ranges.size()-1; ++i)
        {
            if (abs(scan.ranges[i]-scan.ranges[i+1]) > maxSep)
                maxSep = abs(scan.ranges[i]-scan.ranges[i+1]);
            if (abs(scan.ranges[i]-scan.ranges[i+1]) > 0.25)
            {
                if (found)
                {
                    break;
                }
                else
                    found = true;
            }
            else
                found = false;
        }
        return maxSep;
    }

    int locateExit()
    {
        bool found = false;
        float maxSep = 0;
        int foundAt = -1;
        cout << "Exit Located at " << foundAt << "Max sep" << maxSep << endl;

        for (int i = 0; i < scan.ranges.size()-1; ++i)
        {
            //cout << abs(scan.ranges[i]-scan.ranges[i+1]) << endl;
            if (fabs(scan.ranges[i]-scan.ranges[i+1]) > maxSep)
                maxSep = fabs(scan.ranges[i]-scan.ranges[i+1]);
            if (fabs(scan.ranges[i]-scan.ranges[i+1]) > 0.25)
            {
//                if (found)
//                {
                    foundAt = i;
                    break;
//                }
//                else
//                    found = true;
            }
            else
                found = false;
        }
        cout << "Exit Located at " << foundAt << "Max sep" << maxSep << endl;
        return foundAt;
    }

    void faceExit(int locIndex)
    {
        if (io.readLaserData(scan))
        {
            dist_center = scan.ranges[center];
            dist_right = scan.ranges[right];
            dist_left = scan.ranges[left];
        }
        while(abs(locateExit()-center) > 5)
        {
            io.sendBaseReference(0,0,0.1);
            if (io.readLaserData(scan))
            {
                dist_center = scan.ranges[center];
                dist_right = scan.ranges[right];
                dist_left = scan.ranges[left];
            }
            r.sleep();
        }
        return;
    }

    void goToWall()
    {

    }

    void followWall()
    {

    }

    void corner()
    {

    }

    void exit()
    {

    }

    void corridor()
    {

    }
};

#endif // MAIN

