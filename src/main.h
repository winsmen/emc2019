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
#include "actuation.h"


#ifndef LOG_FLAG
#define LOG_FLAG    3
#endif

using namespace cv;
using namespace std;

class Robot
{
public:
    // Robot Variables
    emc::IO io;
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
    Planning *planner;
    Actuation *actuator;
    World world;
    vector<int> cabinet_list;

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
    void printState(sys_state s);
    void log(string text);
    sys_state getState();
    void setState(sys_state s);

};


Robot::Robot(Performance s, sys_state state=STARTUP)
    : specs(s), r(s.heartbeat), maxTrans(s.max_trans), maxRot(s.max_rot), state(state)
{
    outfile.open("../logs/main_log.txt", ios::out | ios::trunc);
    time_t now = time(0);
    log("Pico Main Log: " + string(ctime(&now)));
    log("Waiting for IO modules");
    int maxIter = 20;
    while((!io.readLaserData(world.scan) || !io.readOdometryData(world.odom))
          && maxIter > 0)
    {
        r.sleep();
        --maxIter;
    }
    log("IO modules ready");
    ang_inc = world.scan.angle_increment;
    scan_span = world.scan.ranges.size();
    found_corridor = 0;
    scan_count = 0;
    log("Initialising Measurement Module");
    sense = new Measurement(&io,&world,specs);
    log("Initialising Mapping Module");
    map = new Mapping(&world,specs);
    log("Initialising Planning Module");
    planner = new Planning(&io,&world,sense,map,specs);
    log("Initialising Actuation Module");
    actuator = new Actuation(&io,&world,specs,sense);
    log("Pico State: STARTUP");
    io.speak("Pico ready");
    cabinet_list.clear();
}

Robot::~Robot()
{
    io.speak("Goodbye");
    io.sendBaseReference(0,0,0);
    outfile.close();
}


sys_state Robot::getState()
{
    return state;
}

void Robot::setState(sys_state s)
{
    if (s != state)
    {
        printState(s);
        state = s;
//        io.sendBaseReference(0,0,0);
//        sleep(1);
    }
}


void Robot::printState()
{
    log(to_string(1));
    printState(state);
}

void Robot::printState(sys_state s)
{
    switch(s)
    {
    case STARTUP:
        log("Pico State: STARTUP");
        break;
    case FIRST_LOCALISATION:
        log("Pico State: FIRST_LOCALIZATION");
        break;
    case GET_NEXT_CABINET:
        log("Pico State: GET_NEXT_CABINET");
        break;
    case GO_TO_DESTINATION:
        log("Pico State: GO_TO_DESTINATION");
        break;
    case AT_CABINET:
        log("Pico State: AT_CABINET");
        break;
    case STOP:
        log("Pico State: STOP");
        break;
    }
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

#endif
