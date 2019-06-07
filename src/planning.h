#ifndef PLANNING
#define PLANNING

#include <fstream>
#include <iostream>
#include <ctime>

#include "common_resources.h"

#define PLANNING_LOG_FLAG 3

class Planning
{
    World &world;
    ofstream mapping_log;
public:
    Planning(World *w, const Performance specs);
    ~Planning();
    void plan();
    void log(string text);
};

Planning::Planning(World *w, const Performance specs) : world(*w)
{
    mapping_log.open("../planning_log.txt", ios::out | ios::trunc);
    time_t now = time(0);
    log("Planning Log: " + string(ctime(&now)));
}

Planning::~Planning()
{
    mapping_log.close();
}

void Planning::plan()
{

}

void Planning::log(string text)
{
#if PLANNING_LOG_FLAG == 1 || PLANNING_LOG_FLAG == 3
    mapping_log << text << endl;
#endif
#if PLANNING_LOG_FLAG == 2 || PLANNING_LOG_FLAG == 3
    cout << "planner: " << text << endl;
#endif
}

#endif // PLANNING

