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
    EXIT_UNDETECTABLE,
    ORIENT_TO_EXIT_WALL,
    DRIVE_TO_EXIT,
    ENTER_EXIT_CORRIDOR,
    EXIT_CORRIDOR_FOLLOW,
    // TODO Eliminate unnecessary states
    FIND_WALL,
    ALIGN_TO_WALL,
    FOLLOW_WALL,
    CORNER,
    EXIT_FOUND,
    FOLLOW_CORRIDOR
};


struct robot
{
    // Robot Variables
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

    // Constructors
    robot(int rate);
    robot(int rate, float maxTrans, float maxRot);
    robot(int rate, sys_state state, float maxTrans, float maxRot);

    // Functions
    int measure();
    int map();
    int plan();
    int actuate();

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
            io.sendBaseReference(0,0,maxTrans);
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


robot::robot(int rate, sys_state state, float maxTrans, float maxRot)
    : r(rate), state(state), maxTrans(maxTrans), maxRot(maxRot)
{
    int maxIter = 20;
    while((!io.readLaserData(scan) || !io.readOdometryData(odom))
          && maxIter > 0)
    {
        r.sleep();
        --maxIter;
    }
    angInc = scan.angle_increment;
    center = scan.ranges.size()/2 - 1;
    right = center - (M_PI/2)/angInc - 1;
    left = center + (M_PI/2)/angInc - 1;
}


robot::robot(int rate, float maxTrans, float maxRot)
    : r(rate), state(STARTUP), maxTrans(maxTrans), maxRot(maxRot)
{
    robot(rate, STARTUP, maxTrans, maxRot);
}


robot::robot(int rate)
    : r(rate), state(STARTUP), maxTrans(0.5), maxRot(1.2)
{
    robot(rate, STARTUP, 0.5, 1.2);
}


int robot::measure()
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


int robot::map()
{
    // Identify general features: Corners, Walls, etc.
    // Identify specific objects: exits, cabinets, obstacles, doors, etc.
    return 0;
}


int robot::plan()
{
    switch(state)
    {
    case STARTUP:
        state = SCAN_FOR_EXIT;
        break;

    case SCAN_FOR_EXIT:
        // Exit found
            // state = ORIENT_TO_EXIT_WALL;
        // Sweep complete
            // save maxDist and maxDistDir
            // state = EXIT_UNDETECTABLE;
        break;

    case EXIT_UNDETECTABLE:
        // Obtain direction and half distance of maxDist
        break;

    case ORIENT_TO_EXIT_WALL:
        // if facing wall
            // state = DRIVE_TO_EXIT
        break;

    case DRIVE_TO_EXIT:
        // set exit midpoint as destination
        // if close enough to exit
            // state = ENTER_EXIT_CORRIDOR
        break;
//    //TODO Is this case necessary?
//    case ENTER_EXIT_CORRIDOR:

//        break;

    case EXIT_CORRIDOR_FOLLOW:

        break;
    }

    return 0;
}


int robot::actuate()
{
    switch(state)
    {
    case STARTUP:
        io.sendBaseReference(0,0,0);
        break;

    case SCAN_FOR_EXIT:
        io.sendBaseReference(0,0,0.1);
        break;

    case EXIT_UNDETECTABLE:

        break;

    case ORIENT_TO_EXIT_WALL:

        break;

    case DRIVE_TO_EXIT:

        break;

    case ENTER_EXIT_CORRIDOR:

        break;

    case EXIT_CORRIDOR_FOLLOW:

        break;
    }

    return 0;
}

#endif // MAIN
