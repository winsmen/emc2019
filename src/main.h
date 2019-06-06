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
    static const float min_dist_from_wall = 0.6;
    static const float dist_compare_tol = 0.01;
    static const float corner_compare_tol = 0.1;
    static const float angle_compare_tol = 0.1;
    sys_state state;
    Performance specs;
    Measurement *sense;
    Mapping *map;
    World world;
    vector<int> cabinet_order;

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
    Robot(Performance specs, sys_state state, cabinet_order);
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


Robot::Robot(Performance s, sys_state state=STARTUP,cabinet_order)
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

int Robot::measure()
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
        world.left.d = scan.ranges[world.left.d];
        getMaxMinDist();
    }
    else
        ret = 0;
    if (io.readOdometryData(odom))
        world.angle = odom.a;
    else
        ret -= 2;
    return ret;
}


int Robot::identify()
{
//    cout << scan.ranges[0] << endl;
    int mult = 2;
	corner.clear();    
	distance.clear();
    for (int i = padding+av_range; i < scan_span-padding-av_range; ++i)
    {
        float dist_av = pow((av_range+1),mult)*scan.ranges[i];
        int den = pow((av_range+1),mult);
        for (int j = 1; j <= av_range; ++j)
        {
            dist_av += pow(j,mult)*scan.ranges[i+j] + pow(j,mult)*scan.ranges[i-j];
            den += 2*pow(j,mult);
        }
        dist_av /= den;
        distance.push_back(dist_av);
        if ((dist_av - scan.ranges[i] > corner_compare_tol /*|| fabs(scan.ranges[i]-scan.ranges[i-1]) > 0.01*/)
                && scan.ranges[i] > 0.1)
        {
            corner.push_back(LRFpoint(scan.ranges[i],i));
        }
    }
    n_corners = corner.size();
    log("Number of corners: " + to_string(n_corners));
//    cout << "Number of corners: " << n_corners << endl;
    if (n_corners > 0)
    {
//        cout << "Number of corners: " << n_corners << endl;
//        cout << "Corner Start: " << corner_angle[0] << " Corner End: " << corner_angle[n_corners-1] << endl;
        corridor_center = (corner[0].i+corner[n_corners-1].i)/2;
//        cout << corner_dist[0] << " " << corner_dist[n_corners-1] << endl;
        corridor_center_dist = (corner[0].d+corner[n_corners-1].d)/2;
        if (fabs(corridor_center_dist - scan.ranges[corridor_center]) < corner_compare_tol)
        {
            corridor_center = -1;
            found_corridor -= 1;
            if (found_corridor < 0)
                found_corridor = 0;
        }
        else
        {
            found_corridor += 1;
            if (found_corridor > 10)
                found_corridor = 10;
        }
//        else
//            cout << "Found Exit at: " << corridor_center << endl;
//        cout << "Corner start and end: " << corner_angle[0] << " " << corner_angle[n_corners-1] << endl;
    }
    else
    {
        corridor_center = -1;
        corridor_center = -1;
        found_corridor -= 1;
        if (found_corridor < 0)
            found_corridor = 0;
    }
    displayMap();
    return 0;
}


void Robot::displayMap()
{
    int frame_dim = 600;
    frame = Mat::zeros(frame_dim,frame_dim,CV_8UC3);
    double x_c = frame_dim/2.0;
    double y_c = frame_dim/2.0;
    double x,y;
    for (int i = av_range+padding; i < scan_span-padding-av_range; ++i)
    {
        polar2cart(scan.ranges[i],(i*-ang_inc)+2,x,y,x_c,y_c);
        circle(frame,Point(x,y),1,Scalar(255,0,0),1,8);
//        polar2cart(distance[i-padding-av_range],(i*-ang_inc)+2,x,y,x_c,y_c);
//        circle(frame,Point(x,y),1,Scalar(0,255,255),1,8);
    }
//    polar2cart(0,0,x,y,x_c,y_c);
//    circle(frame,Point(x,y),3,Scalar(255,0,0),1,8);

    switch(state)
    {
    case FOLLOW_CORRIDOR:
//        double right_av = 0;
//        double left_av = 0;
        for (int i = 0; i < 40; ++i)
        {
//            right_av += scan.ranges[right+i] - (scan.ranges[right]/cos(i*ang_inc));
//            left_av += scan.ranges[left-i] - (scan.ranges[left]/cos(i*ang_inc));
            polar2cart((scan.ranges[world.right.i]/cos(i*ang_inc))+dist_compare_tol,((world.right.i+i)*-ang_inc)+2,x,y,x_c,y_c);
            circle(frame,Point(x,y),1,Scalar(255,255,255),1,8);
            polar2cart((scan.ranges[world.right.i]/cos(i*ang_inc))-dist_compare_tol,((world.right.i+i)*-ang_inc)+2,x,y,x_c,y_c);
            circle(frame,Point(x,y),1,Scalar(255,255,255),1,8);
            polar2cart((scan.ranges[world.left.i]/cos(i*ang_inc))+dist_compare_tol,((world.left.i-i)*-ang_inc)+2,x,y,x_c,y_c);
            circle(frame,Point(x,y),1,Scalar(255,255,255),1,8);
            polar2cart((scan.ranges[world.left.i]/cos(i*ang_inc))-dist_compare_tol,((world.left.i-i)*-ang_inc)+2,x,y,x_c,y_c);
            circle(frame,Point(x,y),1,Scalar(255,255,255),1,8);
        }
//        right_av /= 20;
//        left_av /= 20;
//        cout << left_av << " " << right_av << endl;
//        if (right_av > dist_compare_tol && left_av < -dist_compare_tol)
//            vtheta = -maxRot;
//        else if (left_av > dist_compare_tol && right_av < -dist_compare_tol)
//            vtheta = maxRot;
//        else
//            vtheta = 0;
        break;
    default:
        for (int i = av_range+padding; i < scan_span-padding-av_range; ++i)
        {
            polar2cart(distance[i-padding-av_range]+corner_compare_tol,(i*-ang_inc)+2,x,y,x_c,y_c);
            circle(frame,Point(x,y),1,Scalar(255,255,255),1,8);
            polar2cart(distance[i-padding-av_range]-corner_compare_tol,(i*-ang_inc)+2,x,y,x_c,y_c);
            circle(frame,Point(x,y),1,Scalar(255,255,255),1,8);
        }
        if (n_corners > 1)
        {
            for (int i = 0; i < n_corners; ++i)
            {
                polar2cart(corner[i].d,(corner[i].i*-ang_inc)+2,x,y,x_c,y_c);
                circle(frame,Point(x,y),1,Scalar(255,255,0),1,8);
            }
        }
        if (found_corridor == 10)
        {
            polar2cart(corridor_center_dist,(corridor_center*-ang_inc)+2,x,y,x_c,y_c);
            circle(frame,Point(x,y),3,Scalar(0,255,0),2,8);
            polar2cart(corner[0].d,(corner[0].i*-ang_inc)+2,x,y,x_c,y_c);
            circle(frame,Point(x,y),1,Scalar(0,0,255),1,8);
            polar2cart(corner[n_corners-1].d,(corner[n_corners-1].i*-ang_inc)+2,x,y,x_c,y_c);
            circle(frame,Point(x,y),1,Scalar(0,0,255),1,8);
        }
    }

    imshow("Visualization",frame);
    waitKey(25);
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
        log("Start angle: " + to_string(start_angle) + " End Angle: " + to_string(theta));
        //cout << "Start angle: " << start_angle << " End Angle: " << theta << endl;
        break;

    case SCAN_FOR_EXIT:
        vtheta = maxRot;
        // Exit found
        if (found_corridor == 10)
        {
            start_angle = corridor_center;
            log("Found exit at: " + to_string(corridor_center));
            //cout << "Found exit at: " << corridor_center << endl;
            state = FACE_EXIT;
            break;
        }
        // Sweep complete
        cout << "Current angle: " << world.angle << " Destination Angle: " << theta << endl;
        log("Current angle: " + to_string(world.angle) + " Destination Angle: " + to_string(theta));
        if (fabs(world.angle - theta) < angle_compare_tol)
        {
            ++scan_count;
            if (scan_count > 4)
                state = FIND_WALL;
            else
                state = MOVE_TO_MAX;
            vtheta = 0;
            dx = world.center.d/3;
            log("Did not find exit in this scan. Moving forward by: " + to_string(dx));
            //cout << "Did not find exit in this scan. Moving forward by: " << dx << endl;
            log("Min at index: " + to_string(world.nearest.i));
            //cout << "Min at index: " << min_dist_dir << endl;
        }
        break;

    case MOVE_TO_MAX:
        vtheta = 0;
        vx = maxTrans;
        log("dx: " + to_string(dx) + " dist_center: " + to_string(world.center.d) + " Front clear: " + to_string(world.front_clear);
        //cout << "dx: " << dx << " dist_center: " << dist_center << " Front clear: " << front_clear << endl;
        if (world.center.d < dx || !world.front_clear)
        {
            //cout << "Reached max distance" << endl;
            log("Reached max distance");
            vx = 0;
            state = SCAN_FOR_EXIT;
            start_angle = world.angle;
            theta = start_angle + (2*M_PI-4);
            if (theta > M_PI)
                theta -= 2*M_PI;
            //cout << "Start angle: " << start_angle << " End Angle: " << theta << endl;
            log("Start angle: " + to_string(start_angle) + " End Angle: " + to_string(theta));
        }
        if (found_corridor == 10)
        {
            //cout << "Found exit at: " << corridor_center << endl;
            log("Found exit at: " + to_string(corridor_center));
            start_angle = corridor_center;
            state = FACE_EXIT;
        }
        break;

    case FACE_EXIT:
        if (found_corridor == 0)
        {
            //cout << "Lost exit; returning to scan" << endl;
            log("Lost exit; returning to scan);
            state = SCAN_FOR_EXIT;
            break;
        }
        //cout << "Corridor at: " << corridor_center << endl;
        log("Corridor at: " + to_string(corridor_center));
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
            log("Lost exit; returning to scan");
            state = SCAN_FOR_EXIT;
            break;
        }
        //cout << left_clear << front_clear << right_clear <<" Corridor center distance: " << corridor_center_dist << " at " << corridor_center << endl;
        text = to_string(world.left_clear) + to_string(world.front_clear) + to_string(world.right_clear) +" Corridor center distance: " + to_string(corridor_center_dist) + " at " + to_string(corridor_center);
        if ((world.left.d < 2*min_dist_from_wall && world.right.d < 2*min_dist_from_wall))
        {
            //cout << "Arrived at corridor" << endl;
            log("Arrived at corridor");
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
    log("Pico State: ");
    //cout << "Pico State: ";
    switch(state)
    {
    case STARTUP:
        log("STARTUP");
        //cout << "STARTUP" << endl;
        break;
    case SCAN_FOR_EXIT:
        log("SCAN_FOR_EXIT");
        //cout << "SCAN_FOR_EXIT" << endl;
        break;
    case FACE_EXIT:
        log("FACE_EXIT");
        //cout << "FACE_EXIT" << endl;
        break;
    case EXIT_UNDETECTABLE:
        log("EXIT_UNDETECTABLE");
        //cout << "EXIT_UNDETECTABLE" << endl;
        break;
    case ORIENT_TO_EXIT_WALL:
        log("ORIENT_TO_EXIT_WALL");
        //cout << "ORIENT_TO_EXIT_WALL" << endl;
        break;
    case DRIVE_TO_EXIT:
        log("DRIVE_TO_EXIT");
        //cout << "DRIVE_TO_EXIT" << endl;
        break;
    case EXIT_CORRIDOR_FOLLOW:
        log("EXIT_CORRIDOR_FOLLOW");
        //cout << "EXIT_CORRIDOR_FOLLOW" << endl;
        break;
    case STOP:
        log("STOP");
        //cout << "STOP" << endl;
        break;
    case FIND_WALL:
        log("FIND_WALL");
        break;
    case GO_TO_WALL:
        log("GO_TO_WALL");
        break;
    case ALIGN_TO_WALL:
        log("ALIGN_TO_WALL");
        break;
    case FOLLOW_WALL:
        log("FOLLOW_WALL");
        break;
    case CORNER:
        log("CORNER");
        break;
    case ENTER_EXIT_CORRIDOR:
        log("ENTER_EXIT_CORRIDOR");
        //cout << "ENTER_EXIT_CORRIDOR" << endl;
        break;
    case FOLLOW_CORRIDOR:
        log("FOLLOW_CORRIDOR");
        //cout << "FOLLOW_CORRIDOR" << endl;
        break;
    case MOVE_TO_MAX:
        log("MOVE_TO_MAX");
        //cout << "MOVE_TO_MAX" << endl;
        break;
    }
    io.sendBaseReference(0,0,0);
    sleep(1);
}

#endif
