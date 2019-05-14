#ifndef MAIN
#define MAIN

#include <iostream>
#include <unistd.h>
#include <emc/io.h>
#include <emc/rate.h>
#include "config.h"
#include <math.h>
#include <stdlib.h>
#include <cmath>

#define LEFT    1
#define RIGHT   2

using namespace std;

enum sys_state
{
    STARTUP,
    SCAN_FOR_EXIT,
    MOVE_TO_MAX,
    FACE_EXIT,
    EXIT_UNDETECTABLE,
    ORIENT_TO_EXIT_WALL,
    DRIVE_TO_EXIT,
    ENTER_EXIT_CORRIDOR,
    EXIT_CORRIDOR_FOLLOW,
    STOP,
    // TODO Eliminate unnecessary states
    FIND_WALL,
    GO_TO_WALL,
    ALIGN_TO_WALL,
    FOLLOW_WALL,
    CORNER,
    EXIT_FOUND,
    FOLLOW_CORRIDOR,
    GO_TO_EXIT
};

struct Corner
{
    double dist;
    double ang_index;
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
    static const float min_dist_from_wall = 0.6;
    static const float dist_compare_tol = 0.01;
    static const float corner_compare_tol = 0.07;
    static const float angle_compare_tol = 0.05;
    sys_state state;

    // Measurement Constants
    double ang_inc;
    int scan_range;
    int center;
    int right;
    int left;
    // Measurement Variables
    double dist_center;
    double dist_right;
    double dist_left;
    bool front_clear;
    bool right_clear;
    bool left_clear;
    double max_dist;
    double max_dist_dir;
    double min_dist;
    double min_dist_dir;
    double angle;
    static const int padding = 15;
    static const int av_range = 20;
    double distance[1000-2*padding];
    int wall_side;
    int n_corners;
    int corner_dist[1000];
    int corner_angle[1000];
    int corridor_center;
    double corridor_center_dist;

    // Actuation Variables
    double vx;
    double vy;
    double vtheta;
    double dx;
    double dy;
    double theta;
    double start_angle;

    // Constructor
    robot(int rate, float maxTrans, float maxRot);

    // Main Functions
    int measure();
    int map();
    int plan();
    int actuate();

    // Measurement
    void getMaxMinDist();
    void smoothen();

    // Mapping
    bool computeTrajectoryToExit();
    int locateExit();

    // Planning
    void faceExit(int);

    // Actuation

    // Misc
    void printState();

};


robot::robot(int rate, float maxTrans, float maxRot)
    : r(rate), maxTrans(maxTrans), maxRot(maxRot)
{
    int maxIter = 20;
    while((!io.readLaserData(scan) || !io.readOdometryData(odom))
          && maxIter > 0)
    {
        r.sleep();
        --maxIter;
    }
    ang_inc = scan.angle_increment;
    scan_range = scan.ranges.size();
    center = scan_range/2 - 1;
    right = center - (M_PI/2)/ang_inc - 1;
    left = center + (M_PI/2)/ang_inc - 1;
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
    n_corners = 0;
    for (int i = 2*padding; i < scan_range-2*padding; ++i)
    {
        float dist_av = (1)*scan.ranges[i];
        int den = av_range+1;
        for (int j = 1; j < av_range; ++j)
        {
            dist_av += j*scan.ranges[i+j] + j*scan.ranges[i-j];
            den += 2*j;
        }
        dist_av /= den;
        if (dist_av - scan.ranges[i] > corner_compare_tol || fabs(scan.ranges[i]-scan.ranges[i-1]) > 0.2)
        {
            corner_dist[n_corners] = scan.ranges[i];
            corner_angle[n_corners] = i;
            ++n_corners;
        }
    }
    if (n_corners > 0)
    {
//        cout << "Number of corners: " << n_corners << endl;
//        cout << "Corner Start: " << corner_angle[0] << " Corner End: " << corner_angle[n_corners-1] << endl;
        corridor_center = (corner_angle[0]+corner_angle[n_corners-1])/2;
        corridor_center_dist = (corner_dist[0]+corner_dist[n_corners-1])/2;
        if (fabs(scan.ranges[corridor_center] - scan.ranges[corner_angle[0]]) < corner_compare_tol ||
                fabs(scan.ranges[corridor_center] - scan.ranges[corner_angle[n_corners-1]]) < corner_compare_tol)
            corridor_center = -1;
        else
            cout << "Found Exit at: " << corridor_center << endl;
    }
    else
        corridor_center = -1;
    return 0;
}


int robot::plan()
{
    sys_state prevState = state;
    switch(state)
    {
    case STARTUP:
        state = SCAN_FOR_EXIT;
        start_angle = angle;
        theta = start_angle + (2*M_PI-4);
        if (theta > M_PI)
            theta -= 2*M_PI;
        cout << "Start angle: " << start_angle << " End Angle: " << theta << endl;
        break;

    case SCAN_FOR_EXIT:
        vtheta = maxRot;
        // Exit found
        if (corridor_center != -1)
        {
            start_angle = corridor_center;
            //state = FACE_EXIT;
            state = GO_TO_EXIT;
            break;
        }
        // Sweep complete
        cout << fabs(angle - theta) << endl;
        if (fabs(angle - theta) < angle_compare_tol)
        {
            // save maxDist and maxDistDir
            state = MOVE_TO_MAX;
            theta = angle+(max_dist_dir-center)*ang_inc;
            dx = max_dist;
            if (theta > M_PI)
                theta -= 2*M_PI;
            else if (theta < -M_PI)
                theta += 2*M_PI;
            if (theta > angle)
                vtheta = maxRot;
            else
                vtheta = -maxRot;
            cout << "Max at index: " << max_dist_dir << endl;
            cout << "Angle to turn: " << theta-angle << endl;
        }
        break;

    case MOVE_TO_MAX:
        cout << angle-theta << endl;
        if (fabs(angle-theta) < angle_compare_tol)
        {
            vtheta = 0;
            vx = maxTrans;
            cout << "dx: " << dx << " odom.x: " << odom.x << endl;
            if (dist_center < dx/2 || !front_clear)
            {
                vx = 0;
                state = SCAN_FOR_EXIT;
                start_angle = angle;
                theta = start_angle + M_PI;
                if (theta > M_PI)
                    theta -= 2*M_PI;
                cout << "Start angle: " << start_angle << " End Angle: " << theta << endl;
            }
            if (corridor_center != -1)
            {
                start_angle = corridor_center;
                //state = FACE_EXIT;
                state = GO_TO_EXIT;
            }
        }
        break;

    case FACE_EXIT:
        if (corridor_center-center > 5)
            vtheta = maxRot;
        else if (corridor_center-center < -5)
            vtheta = -maxRot;
        else
        {
            start_angle = corridor_center;
            vy = sin((start_angle-center)*ang_inc)*maxTrans;
            vx = cos((start_angle-center)*ang_inc)*maxTrans;
            vtheta = 0;
            state = DRIVE_TO_EXIT;
        }
        break;
        
    case GO_TO_EXIT:
    vx = 0;
    vy = 0;
    vtheta = 0;
    int corner1 = corner_angle[0];
	int corner2 = corner_angle[n_corners-1];
	float exit_center_tol=0.08;                                               // the creterion for centrad

	bool arriveExit = false;
	bool arriveCenter = false;


	
	if (scan.ranges[corner1]< scan.ranges[corner2])                           // compare which corner is closer
	{
		int A = corner1;                                                      // closer corner always names as A
		int B = corner2;                                                      // far corner always names as B
	}
	else
	{
		int B = corner1;
		int A = corner2;
	}
	
	float dist_A = scan.ranges[A];                                            // define the distance to close corner
	float dist_B = scan.ranges[B];                                            // define the distance to far corner
	float dir_A = A*ang_inc;                                                  // define the angle of A 
	float dir_B = B*ang_inc;                                                  // define the angle of B
	float theta = dir_A-dir_B;                                                // angle between line a and b

	float width_exit = sqrt(pow(dist_A,2)+pow(dist_B^2,2)-2*dist_A*dist_B*cos(theta));   //width of exit
	float beta = asin((dist_B/width_exit)*sin(theta));                        // angle of line b with respect the exit wall
	float theta_1 = (center-B)*ang_inc;                                       // angle of line b with repect to center

	float error = dist_A-dist_B;
	
	if (fabs(scan.ranges[corner1]-scan.ranges[corner2])>exit_center_tol || arriveCenter = false)      //not at the center line of the exit
	{
		vx = maxTrans*cos(theta_1+beta);
		vy = maxTrans*sin(theta_1+beta);
	}
	else                                                                      // at the center line, line a = line b
	{
		arriveCenter = true;
		vx = 0;
		vy = 0;
		float alpha = M_PI/2-(beta+theta_1);                                  // spinning until the front towards the exit
		if (alpha>0.1)
		{
		    if (A < B)
		    {
			    vtheta = maxRot;
		    }
		    else
		    {
		        vtheta = -maxRot;
		    }
		}
		else
		{
			if (dist_B>width_exit/2+0.3)
			{
				if (error < 0.15)
				{
					vy = 0;
					vx = maxTrans;
				}
				else
				{
					arriveCenter = false;
				}
			}
			else
			{
				arriveExit = true;
				vtheta = 0;
				vx = 0;
				vy = 0;
			}
		}
        break;
         
    case EXIT_UNDETECTABLE:
        // Obtain direction and half distance of maxDist
        break;

    case ORIENT_TO_EXIT_WALL:
        // if facing wall
            // state = DRIVE_TO_EXIT
        break;

    case DRIVE_TO_EXIT:
        if (corridor_center_dist > 1.5*min_dist_from_wall)
            vx = maxTrans;
        else
        {
            vx = 0;
            state = FOLLOW_CORRIDOR;
        }
        cout << left_clear << " " << right_clear << endl;
        if (!left_clear)
            vy = -maxTrans;
        else if (!right_clear)
            vy = maxTrans;
        else
            vy = 0;
        if (corridor_center-center > 2)
            vtheta = 0.3;
        else if (corridor_center-center < -2)
            vtheta = -0.3;
        else
            vtheta = 0;
        break;

    case FIND_WALL:
        start_angle = min_dist_dir;
        vy = sin((min_dist_dir-center)*ang_inc)*maxTrans;
        vx = cos((min_dist_dir-center)*ang_inc)*maxTrans;
cout << vx << " " << vy << endl;
        vtheta = 0;
        state = GO_TO_WALL;
        cout << "Minimum Distance: " << min_dist << "\nFound at index: " << min_dist_dir << endl;
        break;

    case GO_TO_WALL:
//        cout << "min_dist: " << min_dist << " at start_angle: " << scan.ranges[start_angle] << endl;
//	cout << vx << " " << vy << endl;
        vy = sin((start_angle-center)*ang_inc)*maxTrans;
        vx = cos((start_angle-center)*ang_inc)*maxTrans;
        cout << vx << " " << vy << endl;
        if (min_dist - min_dist_from_wall < dist_compare_tol)
        {
//            vx = 0;
//            vy = 0;
//            vtheta = 0;
            state = ALIGN_TO_WALL;
            if (min_dist_dir < center)
                wall_side = RIGHT;
            else
                wall_side = LEFT;
            cout << "Arrived at wall on " << ((wall_side == RIGHT)?"Right":"Left") << endl;
        }
        break;

    case ALIGN_TO_WALL:
        if (wall_side == RIGHT)
        {
            vtheta = 0;
            float dist_sum = 0;
            for (int i = 0; i < 50; ++i)
            {
                dist_sum += fabs(scan.ranges[right+i]-scan.ranges[right-i]);
            }
            if (dist_sum < 50*dist_compare_tol*0.8)
                state = ALIGN_TO_WALL;
            else
            {
                if (start_angle < right)
                    vtheta = -maxRot;
                else
                    vtheta = maxRot;
            }
        }
        else
        {
            vtheta = 0;
            float dist_sum = 0;
            for (int i = 0; i < 50; ++i)
            {
                dist_sum += fabs(scan.ranges[left+i]-scan.ranges[left-i]);
            }
            if (dist_sum < 50*dist_compare_tol*0.8)
                state = ALIGN_TO_WALL;
            else
            {
                if (start_angle < left)
                    vtheta = -maxRot;
                else
                    vtheta = maxRot;
            }
        }
        break;

    case FOLLOW_WALL:
        vx = maxTrans;
        if (wall_side == RIGHT)
        {
            // check for exit
            bool flag = false;
            for (int i = 0; i < 50; ++i)
            {
//                cout << scan.ranges[right+i] << " " << dist_right*cos(i*ang_inc) << endl;
                if (scan.ranges[right+i] - dist_right*cos(i*ang_inc) > 20*dist_compare_tol)
                {
                    flag = true;
                    break;
                }
            }
            if (flag && dist_right - min_dist_from_wall < 2*dist_compare_tol)
            {
                state = ENTER_EXIT_CORRIDOR;
                break;
            }
            // check for deviation
            vy = min(max(float(min_dist_from_wall-min_dist),-maxTrans),maxTrans);
            int count = 0;
            for (int i = 0; i < 20; ++i)
            {
                if (scan.ranges[right+i] - (min_dist_from_wall/cos(i)) > dist_compare_tol)
                    ++count;
                else if (scan.ranges[right+i] - (min_dist_from_wall/cos(i)) < -dist_compare_tol)
                    --count;
            }
            if (count > 1)
                vtheta = -0.1;
            else if (count < -1)
                vtheta = 0.1;
            else
                vtheta = 0;
            // check for corner
            for (int i = center-100; i < center; ++i)
            {
                if (scan.ranges[i] < (min_dist_from_wall)/cos((i-center)*ang_inc) && scan.ranges[i] > 0.01)
                {
                    state = CORNER;
                    start_angle = angle;
                    break;
                }
            }
        }
        else
        {
            bool flag = false;
            for (int i = 0; i < 50; ++i)
            {
//                cout << scan.ranges[left-i] << " " << dist_left*cos(i*ang_inc) << endl;
                if (scan.ranges[left-i] - dist_left*cos(i*ang_inc) > 5*dist_compare_tol)
                {
                    flag = true;
                    break;
                }
            }
            cout << scan.ranges[left+50] - min_dist_from_wall/cos(50*ang_inc) << endl;
            if (flag && fabs(scan.ranges[left+50] - min_dist_from_wall/cos(50*ang_inc)) < 2*dist_compare_tol)
            {
                state = ENTER_EXIT_CORRIDOR;
                break;
            }
            vy = max(min(float(min_dist-min_dist_from_wall),maxTrans),-maxTrans);
            int count = 0;
            for (int i = 0; i < 20; ++i)
            {
                if (scan.ranges[left-i] - (min_dist_from_wall/cos(i)) > dist_compare_tol)
                    ++count;
                else if (scan.ranges[left-i] - (min_dist_from_wall/cos(i)) < -dist_compare_tol)
                    --count;
            }
            if (count > 1)
                vtheta = 0.1;
            else if (count < -1)
                vtheta = -0.1;
            else
                vtheta = 0;
            for (int i = center; i < center+100; ++i)
            {
                if (scan.ranges[i] < (min_dist_from_wall)/cos((i-center)*ang_inc) && scan.ranges[i] > 0.01)
                {
                    state = CORNER;
                    start_angle = angle;
                    break;
                }
            }
        }
        break;

    case CORNER:
        vx = 0;
        vy = 0;
        if (wall_side == RIGHT)
        {
            double end_angle = start_angle+(M_PI/2);
            if (end_angle > M_PI)
                end_angle -= 2*M_PI;
            if (fabs(angle-end_angle) > angle_compare_tol)
               vtheta = maxRot;
            else
            {
                vtheta = 0;
                state = FOLLOW_WALL;
                start_angle = end_angle;
            }
//            cout << "Start angle: " << start_angle << " End Angle: " << end_angle << "Diff: " << fabs(angle-end_angle) << endl;
        }
        else
        {
            double end_angle = start_angle-(M_PI/2);
            if (end_angle < -M_PI)
                end_angle += 2*M_PI;
            if (fabs(angle-end_angle) > angle_compare_tol)
               vtheta = -maxRot;
            else
            {
                vtheta = 0;
                state = FOLLOW_WALL;
                start_angle = end_angle;
            }
//            cout << "Start angle: " << start_angle << " End Angle: " << end_angle << "Diff: " << fabs(angle-end_angle) << endl;
        }
        break;

    case ENTER_EXIT_CORRIDOR:
        vx = maxTrans;
        vy = 0;
        if (wall_side == RIGHT)
        {
            if (dist_left < 1.5*min_dist_from_wall || dist_right < min_dist_from_wall-0.1)
            {
                vtheta = 0;
                state = FOLLOW_CORRIDOR;
            }
            else
                vtheta = -maxRot;
        }
        else
        {
            if (dist_right < 1.5*min_dist_from_wall || dist_left < min_dist_from_wall-0.1)
            {
                vtheta = 0;
                state = FOLLOW_CORRIDOR;
            }
            else
                vtheta = maxRot;
        }
        break;

    case FOLLOW_CORRIDOR:
        vx = maxTrans;
        if ((dist_left > 2*min_dist_from_wall && dist_right > 2*min_dist_from_wall) || dist_center < min_dist_from_wall)
        {
            cout << bool(dist_left > 2*min_dist_from_wall) << " " <<  bool(dist_right > 2*min_dist_from_wall) << " " <<  bool(dist_center < min_dist_from_wall) << endl;
            state = STOP;
            break;
        }
        else if (dist_left > 1.5*min_dist_from_wall || dist_right > 1.5*min_dist_from_wall)
        {
            state = SCAN_FOR_EXIT;
            break;
        }
        else
        {
            double right_av = 0;
            double left_av = 0;
            for (int i = 0; i < 20; ++i)
            {
                right_av += scan.ranges[right+i] - (scan.ranges[right]/cos(i));
                left_av += scan.ranges[left-i] - (scan.ranges[left]/cos(i));
            }
            right_av /= 20;
            left_av /= 20;
            if (right_av > dist_compare_tol && left_av < -dist_compare_tol)
                vtheta = -maxRot;
            else if (left_av > dist_compare_tol && right_av < -dist_compare_tol)
                vtheta = maxRot;
            else
                vtheta = 0;
        }
        vy = min(max(float(dist_left-dist_right),-maxTrans/2),maxTrans/2);
        break;

    case STOP:
        vx = 0;
        vy = 0;
        vtheta = 0;
        break;

    }

    if (prevState != state)
        printState();

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
        io.sendBaseReference(0,0,vtheta);
        break;

    case MOVE_TO_MAX:
        io.sendBaseReference(vx,0,vtheta);
        break;

    case FACE_EXIT:
        io.sendBaseReference(0,0,vtheta);
        break;

    case EXIT_UNDETECTABLE:
        io.sendBaseReference(vx,0,vtheta);
        break;

    case ORIENT_TO_EXIT_WALL:
        io.sendBaseReference(0,0,vtheta);
        break;

    case DRIVE_TO_EXIT:
        io.sendBaseReference(vx,vy,vtheta);
        break;

    case EXIT_CORRIDOR_FOLLOW:
        io.sendBaseReference(vx,vy,vtheta);
        break;

    case STOP:
        io.sendBaseReference(0,0,0);
        break;

    case FIND_WALL:
        io.sendBaseReference(0,0,0);
        break;

    case GO_TO_WALL:
        io.sendBaseReference(vx,vy,0);
        break;

    case ALIGN_TO_WALL:
        io.sendBaseReference(vx,vy,vtheta);
        break;

    case FOLLOW_WALL:
        io.sendBaseReference(vx,vy,vtheta);
        break;

    case CORNER:
        io.sendBaseReference(0,0,vtheta);
        break;

    case ENTER_EXIT_CORRIDOR:
        io.sendBaseReference(vx,vy,vtheta);
        break;

    case FOLLOW_CORRIDOR:
        io.sendBaseReference(vx,vy,vtheta);
        break;
    
    case GO_TO_EXIT:
        io.sendBaseReference(vx,vy,vtheta);
        break;
    }

    return 0;
}

void robot::printState()
{
    cout << "Pico State: ";
    switch(state)
    {
    case STARTUP:
        cout << "STARTUP" << endl;
        break;
    case SCAN_FOR_EXIT:
        cout << "SCAN_FOR_EXIT" << endl;
        break;
    case FACE_EXIT:
        cout << "FACE_EXIT" << endl;
        break;
    case EXIT_UNDETECTABLE:
        cout << "EXIT_UNDETECTABLE" << endl;
        break;
    case ORIENT_TO_EXIT_WALL:
        cout << "ORIENT_TO_EXIT_WALL" << endl;
        break;
    case DRIVE_TO_EXIT:
        cout << "DRIVE_TO_EXIT" << endl;
        break;
    case EXIT_CORRIDOR_FOLLOW:
        cout << "EXIT_CORRIDOR_FOLLOW" << endl;
        break;
    case STOP:
        cout << "STOP" << endl;
        break;
    case FIND_WALL:
        cout << "FIND_WALL" << endl;
        break;
    case GO_TO_WALL:
        cout << "GO_TO_WALL" << endl;
        break;
    case ALIGN_TO_WALL:
        cout << "ALIGN_TO_WALL" << endl;
        break;
    case FOLLOW_WALL:
        cout << "FOLLOW_WALL" << endl;
        break;
    case CORNER:
        cout << "CORNER" << endl;
        break;
    case ENTER_EXIT_CORRIDOR:
        cout << "ENTER_EXIT_CORRIDOR" << endl;
        break;
    case FOLLOW_CORRIDOR:
        cout << "FOLLOW_CORRIDOR" << endl;
        break;
    case MOVE_TO_MAX:
        cout << "MOVE_TO_MAX" << endl;
        break;
    }
}

void robot::smoothen()
{
    int smoothing[11] = {0.1,0.2,0.3,0.4,0.5,0.6,0.5,0.4,0.3,0.2,0.1};
    for (int i = 0; i < scan_range-2*padding; ++i)
    {
        distance[i] = 0;
        for (int j = 0; j < sizeof(smoothing); ++j)
        {
            distance[i] = smoothing[j]*distance[i+j];
        }
        distance[i] /= 3.6;
    }
}


void robot::getMaxMinDist()
{
    max_dist = scan.ranges[0];
    min_dist = 20;
    max_dist_dir = min_dist_dir = 0;
    front_clear = true;
    right_clear = true;
    left_clear = true;
    for (int i = padding; i < scan_range-padding; ++i)
    {
        if (scan.ranges[i] > max_dist)
        {
            max_dist = scan.ranges[i];
            max_dist_dir = i;
        }
        else if (scan.ranges[i] < min_dist && scan.ranges[i] > 0.1)
        {
            min_dist = scan.ranges[i];
            min_dist_dir = i;
        }
        if (i < (right+center)/2)
        {
            if (scan.ranges[i] < min_dist_from_wall)
                right_clear = false;
        }
        else if (i > (left+center)/2)
        {
            if (scan.ranges[i] < min_dist_from_wall)
            {
                left_clear = false;
            }
        }
        else
            if (scan.ranges[i] < min_dist_from_wall)
                front_clear = false;
    }
}


int robot::locateExit()
{
    bool found = false;
    float maxSep = 0;
    int foundAt = -1;
    for (int i = 0; i < scan.ranges.size()-1; ++i)
    {
        if (fabs(scan.ranges[i]-scan.ranges[i+1]) > maxSep)
        {
            maxSep = fabs(scan.ranges[i]-scan.ranges[i+1]);
            foundAt = i;
        }
    }
    cout << "Exit Located at " << foundAt << "Max sep" << maxSep << endl;
    return foundAt;
}


void robot::faceExit(int locIndex)
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


bool robot::computeTrajectoryToExit()
{
//    bool arriveCenter = false;                        // not on the center line
//	bool arriveExit = false;                          // not arrived at the exit
//	vtheta = 0;
//    if (scan.range[B] > 0.65 || arriveCenter = false)  // not arrived at the exit
//    {
//		if (scan.range[B]-scan.range[A]>0.1)          // the exit is at the right side
//		{
//			vx = 1.5;
//		    vy = 0;
//		}
//		elseif (scan.range[B]-scan.range[A]<-0.1)     // the exit is at the left side
//		{
//			vx = -1.5;
//		    vy = 0;
//		}
//		else
//		{
//			vx = 0;
//			vy = 1.5;
//			arriveCenter = true;
//			float error = scan.range[B]-scan.range[A];   // error during driving through the center line
//			if (error > 0.5)                          // the robot away from the center line
//			{
//				arriveCenter = false;
//			}
//		}
//	}
//	else
//	{
//		vx = 0;
//		vy = 0;
//		arriveExit = true;
//	}
//	return arriveExit;
    // @Muliang
}

bool robot::goToExit()
{
	int corner1 = corner_angle[0];
	int corner2 = corner_angle[n_corners-1];
	float exit_center_tol=0.08;                                               // the creterion for centrad

	bool arriveExit = false;
	bool arriveCenter = false;


	
	if (scan.ranges[corner1]< scan.ranges[corner2])                           // compare which corner is closer
	{
		int A = corner1;                                                      // closer corner always names as A
		int B = corner2;                                                      // far corner always names as B
	}
	else
	{
		int B = corner1;
		int A = corner2;
	}
	
	float dist_A = scan.ranges[A];                                            // define the distance to close corner
	float dist_B = scan.ranges[B];                                            // define the distance to far corner
	float dir_A = A*ang_inc;                                                  // define the angle of A 
	float dir_B = B*ang_inc;                                                  // define the angle of B
	float theta = dir_A-dir_B;                                                // angle between line a and b

	float width_exit = sqrt(pow(dist_A,2)+pow(dist_B^2,2)-2*dist_A*dist_B*cos(theta));   //width of exit
	float beta = asin((dist_B/width_exit)*sin(theta));                        // angle of line b with respect the exit wall
	float theta_1 = (center-B)*ang_inc;                                       // angle of line b with repect to center

	float error = dist_A-dist_B;
	
	if (fabs(scan.ranges[corner1]-scan.ranges[corner2])>exit_center_tol || arriveCenter = false)      //not at the center line of the exit
	{
		vx = maxTrans*cos(theta_1+beta);
		vy = maxTrans*sin(theta_1+beta);
	}
	else                                                                      // at the center line, line a = line b
	{
		arriveCenter = true;
		vx = 0;
		vy = 0;
		float alpha = M_PI/2-(beta+theta_1);                                  // spinning until the front towards the exit
		if (alpha>0.1)
		{
		    if (A < B)
		    {
			    vtheta = maxRot;
		    }
		    else
		    {
		        vtheta = -maxRot;
		    }
		}
		else
		{
			if (dist_B>width_exit/2+0.3)
			{
				if (error < 0.15)
				{
					vy = 0;
					vx = maxTrans;
				}
				else
				{
					arriveCenter = false;
				}
			}
			else
			{
				arriveExit = true;
				vtheta = 0;
				vx = 0;
				vy = 0;
			}
		}
	}
return arriveExit
#endif
