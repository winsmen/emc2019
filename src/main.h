#ifndef MAIN
#define MAIN

#include <iostream>
#include <unistd.h>
#include <emc/io.h>
#include <emc/rate.h>
#include <stdlib.h>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <string.h>
#include <fstream>
#include <sstream>
#include <ctime>

#include "measurement.h"
#include "mapping.h"
#include "planning.h"

#define LEFT        1
#define RIGHT       2

#ifndef LOG_FLAG
#define LOG_FLAG    3
#endif

using namespace cv;
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
    FOLLOW_CORRIDOR
};

string text;
string comma(",");


class Robot
{
public:
    // Robot Variables
    emc::IO io;
    emc::LaserData scan;
    emc::OdometryData odom;
    emc::Rate r;
    float maxTrans;
    float maxRot;
    float min_dist_from_wall = 0.6;
    float dist_compare_tol = 0.01;
    float corner_compare_tol = 0.1;
    float angle_compare_tol = 0.1;
    sys_state state;
    Performance specs;
    Measurement *sense;
    Mapping *map;
    World world;

    // Measurement Constants
    double ang_inc;
    int scan_span;
    static const int padding = 15;
    static const int av_range = 20;
    vector<double> distance;    //unused
    int wall_side;
    int n_corners;
    vector<LRFpoint> corner;
    int corridor_center;
    double corridor_center_dist;
    int found_corridor;
    int scan_count;

    //Mapping Variables
    Mat frame;

    ofstream outfile;

    // Actuation Variables
    double vx;
    double vy;
    double vtheta;
    double dx;
    double dy;
    double theta;
    double start_angle;

    // Constructor
    Robot(Performance specs, sys_state state);
    ~Robot();

    // Main Functions
    int plan();
    int actuate();

    // Measurement

    // Mapping

    // Planning

    // Actuation

    // Misc
    void printState();
    void log(string text);
    sys_state getState();
    void setState(sys_state s);

};


sys_state Robot::getState()
{
    return state;
}

void Robot::setState(sys_state s)
{
    state = s;
    printState();
}

void Robot::log(string text)
{
#if LOG_FLAG == 1 || LOG_FLAG == 3
    outfile << text << endl;
#endif
#if LOG_FLAG == 2 || LOG_FLAG == 3
    cout << "main: " << text << endl;
#endif
}
//
//    outfile.open("laser.csv", ios::out | ios::trunc);
//    outfile << "state" << ',' << "n_corners" << ',' << "found_corridor"  << ',' <<"dist_center"  << "dist_right"  << ','<< "dist_left"  << ','<< "vx"  << ','<< "vy"  << ','<< "vtheta" << endl;
//        outfile << state << ',' << n_corners << ',' << found_corridor << dist_center << dist_right << dist_left << vx << vy << vtheta << endl;
//    outfile.close();


Robot::Robot(Performance s, sys_state state=STARTUP)
    : specs(s), r(s.heartbeat), maxTrans(s.maxTrans), maxRot(s.maxRot), state(state)
{
    outfile.open("../log.txt", ios::out | ios::trunc);
    time_t now = time(0);
    log("Pico Main Log: " + string(ctime(&now)));
    log("Waiting for IO modules");
    int maxIter = 20;
    while((!io.readLaserData(scan) || !io.readOdometryData(odom))
          && maxIter > 0)
    {
        r.sleep();
        --maxIter;
    }
    log("IO modules ready");
    ang_inc = scan.angle_increment;
    scan_span = scan.ranges.size();
    found_corridor = 0;
    scan_count = 0;
    log("Initialising Measurement Module");
    sense = new Measurement(&io,&scan,&odom,&world,specs);
    log("Initialising Mapping Module");
    map = new Mapping(&scan,&odom,&world,specs);
    log("Pico State: STARTUP");
    io.speak("Pico ready");
}

Robot::~Robot()
{
    io.speak("Goodbye");
    io.sendBaseReference(0,0,0);
    outfile.close();
}


int Robot::plan()
{
    sys_state prevState = state;
    switch(state)
    {
    case STARTUP:
        state = SCAN_FOR_EXIT;
        start_angle = world.angle;
        theta = start_angle + (2*M_PI-4);
        if (theta > M_PI)
            theta -= 2*M_PI;
        text = "Start angle: " + to_string(start_angle) + " End Angle: " + to_string(theta);    
        log(text);
        //cout << "Start angle: " << start_angle << " End Angle: " << theta << endl;
        break;

    case SCAN_FOR_EXIT:
        vtheta = maxRot;
        // Exit found
        if (found_corridor == 10)
        {
            start_angle = corridor_center;
            text = "Found exit at: " + to_string(corridor_center);
            log(text);
            //cout << "Found exit at: " << corridor_center << endl;
            state = FACE_EXIT;
            break;
        }
        // Sweep complete
        cout << "Current angle: " << world.angle << " Destination Angle: " << theta << endl;
        if (fabs(world.angle - theta) < angle_compare_tol)
        {
            ++scan_count;
            if (scan_count > 4)
                state = FIND_WALL;
            else
                state = MOVE_TO_MAX;
            vtheta = 0;
            dx = world.center.d/3;
            text =  "Did not find exit in this scan. Moving forward by: " + to_string(dx); 
            log(text);
            //cout << "Did not find exit in this scan. Moving forward by: " << dx << endl;
            text = "Min at index: " + to_string(world.nearest.i);
            log(text);
            //cout << "Min at index: " << min_dist_dir << endl;
        }
        break;

    case MOVE_TO_MAX:
        vtheta = 0;
        vx = maxTrans;
        text = "dx: " + to_string(dx) + " dist_center: " + to_string(world.center.d) + " Front clear: " + to_string(world.front_clear);
        log(text);
        //cout << "dx: " << dx << " dist_center: " << dist_center << " Front clear: " << front_clear << endl;
        if (world.center.d < dx || !world.front_clear)
        {
            //cout << "Reached max distance" << endl;
            text = "Reached max distance";
            log(text);
            vx = 0;
            state = SCAN_FOR_EXIT;
            start_angle = world.angle;
            theta = start_angle + (2*M_PI-4);
            if (theta > M_PI)
                theta -= 2*M_PI;
            //cout << "Start angle: " << start_angle << " End Angle: " << theta << endl;
            text = "Start angle: " + to_string(start_angle) + " End Angle: " + to_string(theta);
            log(text);
        }
        if (found_corridor == 10)
        {
            //cout << "Found exit at: " << corridor_center << endl;
            text = "Found exit at: " + to_string(corridor_center);
            log(text);
            start_angle = corridor_center;
            state = FACE_EXIT;
        }
        break;

    case FACE_EXIT:
        if (found_corridor == 0)
        {
            //cout << "Lost exit; returning to scan" << endl;
            text = "Lost exit; returning to scan";
            log(text);
            state = SCAN_FOR_EXIT;
            break;
        }
        //cout << "Corridor at: " << corridor_center << endl;
        text = "Corridor at: " + to_string(corridor_center);
        log(text);
        if (corridor_center-world.center.i > 5)
            vtheta = min((corridor_center-world.center.i)/35.0,double(maxRot));
        else if (corridor_center-world.center.i < -5)
            vtheta = max((corridor_center-world.center.i)/35.0,double(-maxRot));
        else
        {
            start_angle = corridor_center;
            vy = sin((start_angle-world.center.i)*ang_inc)*maxTrans;
            vx = cos((start_angle-world.center.i)*ang_inc)*maxTrans;
            vtheta = 0;
            state = DRIVE_TO_EXIT;
        }
        break;

    case DRIVE_TO_EXIT:
        if (found_corridor == 0)
        {
            //cout << "Lost exit; returning to scan" << endl;
            text = "Lost exit; returning to scan";
            log(text);
            state = SCAN_FOR_EXIT;
            break;
        }
        //cout << left_clear << front_clear << right_clear <<" Corridor center distance: " << corridor_center_dist << " at " << corridor_center << endl;
        text = to_string(world.left_clear) + to_string(world.front_clear) + to_string(world.right_clear) +" Corridor center distance: " + to_string(corridor_center_dist) + " at " + to_string(corridor_center);
        if ((world.left.d < 2*min_dist_from_wall && world.right.d < 2*min_dist_from_wall))
        {
            //cout << "Arrived at corridor" << endl;
            text = "Arrived at corridor";
            log(text);
            vx = 0;
            state = FOLLOW_CORRIDOR;
            break;
        }
        if (world.front_clear)
        {
            vx = maxTrans;
            if (corridor_center-world.center.i > 2)
            {
                if (vtheta < 0)
                    vtheta = 0;
                vtheta += 0.1;
                if (vtheta > maxRot)
                    vtheta = maxRot;
            }
            else if (corridor_center-world.center.i < -2)
            {
                if (vtheta > 0)
                    vtheta = 0;
                vtheta -= 0.1;
                if (vtheta < -maxRot)
                    vtheta = -maxRot;
            }
            else
                vtheta = 0;
        }
        else
        {
            vx = 0.2;
            if (corridor_center-world.center.i > 2)  //Changed from max_dist_front_dir to corridor_center
            {
                if (vtheta < 0)
                    vtheta = 0;
                vtheta += 0.1;
                if (vtheta > maxRot/2)
                    vtheta = maxRot/2;
            }
            else if (corridor_center-world.center.i < -2)    //Changed from max_dist_front_dir to corridor_center
            {
                if (vtheta > 0)
                    vtheta = 0;
                vtheta -= 0.1;
                if (vtheta < -maxRot/2)
                    vtheta = -maxRot/2;
            }
            else
                vtheta = 0;
        }
        if (!world.left_clear)
        {
            if (vy > 0)
                vy = 0;
            vy -= 0.1;
            if (vy < -maxTrans)
                vy = -maxTrans;
        }
        if (!world.right_clear)
        {
            if (vy < 0)
                vy = 0;
            vy += 0.1;
            if (vy > maxTrans)
                vy = maxTrans;
        }
        if (world.left_clear && world.right_clear)
            vy = 0;
        break;

    case FIND_WALL:
        start_angle = world.nearest.i;
        vy = sin((world.nearest.i-world.center.i)*ang_inc)*maxTrans;
        vx = cos((world.nearest.i-world.center.i)*ang_inc)*maxTrans;
cout << vx << " " << vy << endl;
        vtheta = 0;
        state = GO_TO_WALL;
        cout << "Minimum Distance: " << world.nearest.d << "\nFound at index: " << world.nearest.i << endl;
        break;

    case GO_TO_WALL:
//        cout << "min_dist: " << min_dist << " at start_angle: " << scan.ranges[start_angle] << endl;
//	cout << vx << " " << vy << endl;
        vy = sin((start_angle-world.center.i)*ang_inc)*maxTrans;
        vx = cos((start_angle-world.center.i)*ang_inc)*maxTrans;
        cout << vx << " " << vy << endl;
        if (world.nearest.d- min_dist_from_wall < dist_compare_tol)
        {
//            vx = 0;
//            vy = 0;
//            vtheta = 0;
            state = ALIGN_TO_WALL;
            if (world.nearest.i < world.center.i)
                wall_side = RIGHT;
            else
                wall_side = LEFT;
            cout << "Arrived at wall on " << ((wall_side == RIGHT)?"Right":"Left") << endl;
        }
        break;

    case ALIGN_TO_WALL:
        if (wall_side == RIGHT)
        {
            double right_av = 0;
            for (int i = 0; i < 20; ++i)
            {
                right_av += scan.ranges[world.right.i+i] - (scan.ranges[world.right.i]/cos(i*ang_inc));
            }
            right_av /= 20;
            cout << "Right Average: " << right_av << endl;
            if (right_av > dist_compare_tol)
                vtheta = -maxRot;
            else if (right_av < -dist_compare_tol)
                vtheta = maxRot;
            else
            {
                vtheta = 0;
                state = FOLLOW_WALL;
            }
        }
        else
        {
            double left_av = 0;
            for (int i = 0; i < 20; ++i)
            {
                left_av += scan.ranges[world.left.i-i] - (scan.ranges[world.left.i]/cos(i*ang_inc));
            }
            left_av /= 20;
            cout << "Left Average: " << left_av << endl;
            if (left_av < -dist_compare_tol)
                vtheta = -maxRot;
            else if (left_av > dist_compare_tol)
                vtheta = maxRot;
            else
            {
                vtheta = 0;
                state = FOLLOW_WALL;
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
                if (scan.ranges[world.right.i+i] - world.right.d*cos(i*ang_inc) > 20*dist_compare_tol)
                {
                    flag = true;
                    break;
                }
            }
            if (flag && world.right.d - min_dist_from_wall < 2*dist_compare_tol)
            {
                state = ENTER_EXIT_CORRIDOR;
                break;
            }
            // check for deviation
            vy = min(max(float(min_dist_from_wall-world.nearest.d),-maxTrans),maxTrans);
            int count = 0;
            for (int i = 0; i < 20; ++i)
            {
                if (scan.ranges[world.right.i+i] - (min_dist_from_wall/cos(i)) > dist_compare_tol)
                    ++count;
                else if (scan.ranges[world.right.i+i] - (min_dist_from_wall/cos(i)) < -dist_compare_tol)
                    --count;
            }
            if (count > 1)
                vtheta = -0.1;
            else if (count < -1)
                vtheta = 0.1;
            else
                vtheta = 0;
            // check for corner
            for (int i = world.center.i-100; i < world.center.i; ++i)
            {
                if (scan.ranges[i] < (min_dist_from_wall)/cos((i-world.center.i)*ang_inc) && scan.ranges[i] > 0.01)
                {
                    state = CORNER;
                    start_angle = world.angle;
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
                if (scan.ranges[world.left.i-i] - world.left.d*cos(i*ang_inc) > 5*dist_compare_tol)
                {
                    flag = true;
                    break;
                }
            }
            cout << scan.ranges[world.left.i+50] - min_dist_from_wall/cos(50*ang_inc) << endl;
            if (flag && fabs(scan.ranges[world.left.i+50] - min_dist_from_wall/cos(50*ang_inc)) < 2*dist_compare_tol)
            {
                state = ENTER_EXIT_CORRIDOR;
                break;
            }
            vy = max(min(float(world.nearest.i-min_dist_from_wall),maxTrans),-maxTrans);
            int count = 0;
            for (int i = 0; i < 20; ++i)
            {
                if (scan.ranges[world.left.i-i] - (min_dist_from_wall/cos(i)) > dist_compare_tol)
                    ++count;
                else if (scan.ranges[world.left.i-i] - (min_dist_from_wall/cos(i)) < -dist_compare_tol)
                    --count;
            }
            if (count > 1)
                vtheta = 0.1;
            else if (count < -1)
                vtheta = -0.1;
            else
                vtheta = 0;
            for (int i = world.center.i; i < world.center.i+100; ++i)
            {
                if (scan.ranges[i] < (min_dist_from_wall)/cos((i-world.center.i)*ang_inc) && scan.ranges[i] > 0.01)
                {
                    state = CORNER;
                    start_angle = world.angle;
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
            if (fabs(world.angle-end_angle) > angle_compare_tol)
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
            if (fabs(world.angle-end_angle) > angle_compare_tol)
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
            if (world.left.d < 1.5*min_dist_from_wall || world.right.d < min_dist_from_wall-0.1)
            {
                vtheta = 0;
                state = FOLLOW_CORRIDOR;
            }
            else
                vtheta = -maxRot;
        }
        else
        {
            if (world.right.d < 1.5*min_dist_from_wall || world.left.d < min_dist_from_wall-0.1)
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
        cout << world.left_clear << world.right_clear;
        if (world.left_clear && world.right_clear)
                //|| dist_center < min_dist_from_wall)
        {
            //cout << dist_left << " " <<  dist_right << " " <<  dist_center << endl;
            text = to_string(world.left.d) + " " + to_string(world.right.d) + " " +  to_string(world.center.d);
            log(text);
            state = STOP;
            break;
        }
        else if (scan.ranges[world.left.i-50] > 2*min_dist_from_wall && scan.ranges[world.right.i+50] > 2*min_dist_from_wall)
        {
            state = SCAN_FOR_EXIT;
            break;
        }
        else
        {
            double right_av = 0;
            double left_av = 0;
            for (int i = 0; i < 40; ++i)
            {
                right_av += scan.ranges[world.right.i+i] - (scan.ranges[world.right.i]/cos(i*ang_inc));
                left_av += scan.ranges[world.left.i-i] - (scan.ranges[world.left.i]/cos(i*ang_inc));
            }
            right_av /= 20;
            left_av /= 20;
            text = to_string(left_av) + " " + to_string(right_av);
            log(text);
            //cout << left_av << " " << right_av << endl;
            if (right_av > dist_compare_tol && left_av < -dist_compare_tol)
            {
                if (vtheta > 0)
                    vtheta = 0;
                else if (vtheta < -maxRot)
                    vtheta = -maxRot;
                else
                    vtheta -= 0.1;
            }
            else if (left_av > dist_compare_tol && right_av < -dist_compare_tol)
            {
                if (vtheta < 0)
                    vtheta = 0;
                else if (vtheta > maxRot)
                    vtheta = maxRot;
                else
                    vtheta += 0.1;
            }
            else
                vtheta = 0;
        }
        vy = min(max(float(world.left.d-world.right.d),-maxTrans/2),maxTrans/2);
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


int Robot::actuate()
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
    }

    return 0;
}

void Robot::printState()
{
    text = "Pico State: ";
    log(text);
    //cout << "Pico State: ";
    switch(state)
    {
    case STARTUP:
        text = "STARTUP";
        log(text);
        //cout << "STARTUP" << endl;
        break;
    case SCAN_FOR_EXIT:
        text = "SCAN_FOR_EXIT";
        log(text);
        //cout << "SCAN_FOR_EXIT" << endl;
        break;
    case FACE_EXIT:
        text = "FACE_EXIT";
        log(text);
        //cout << "FACE_EXIT" << endl;
        break;
    case EXIT_UNDETECTABLE:
        text = "EXIT_UNDETECTABLE";
        log(text);
        //cout << "EXIT_UNDETECTABLE" << endl;
        break;
    case ORIENT_TO_EXIT_WALL:
        text = "ORIENT_TO_EXIT_WALL";
        log(text);
        //cout << "ORIENT_TO_EXIT_WALL" << endl;
        break;
    case DRIVE_TO_EXIT:
        text = "DRIVE_TO_EXIT";
        log(text);
        //cout << "DRIVE_TO_EXIT" << endl;
        break;
    case EXIT_CORRIDOR_FOLLOW:
        text = "EXIT_CORRIDOR_FOLLOW";
        log(text);
        //cout << "EXIT_CORRIDOR_FOLLOW" << endl;
        break;
    case STOP:
        text = "STOP";
        log(text);
        //cout << "STOP" << endl;
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
        text = "ENTER_EXIT_CORRIDOR";
        log(text);
        //cout << "ENTER_EXIT_CORRIDOR" << endl;
        break;
    case FOLLOW_CORRIDOR:
        text = "FOLLOW_CORRIDOR";
        log(text);
        //cout << "FOLLOW_CORRIDOR" << endl;
        break;
    case MOVE_TO_MAX:
        text = "MOVE_TO_MAX";
        log(text);
        //cout << "MOVE_TO_MAX" << endl;
        break;
    }
    io.sendBaseReference(0,0,0);
    sleep(1);
}

#endif
