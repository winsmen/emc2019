#ifndef PLANNING
#define PLANNING

#include <fstream>
#include <iostream>
#include <ctime>

#include "common_resources.h"
#include "measurement.h"

#define PLANNING_LOG_FLAG 3

using namespace std;

class Planning
{
    World &world;
    Measurement &sense;
    ofstream planning_log;
    double dist_compare_tol;
    double max_rot,max_trans;

    sys_state startup(sys_state s);
    sys_state localize();
    sys_state stop(sys_state s);

public:
    Planning(World *w, Measurement *s, const Performance specs);
    ~Planning();
    sys_state plan(sys_state s);
    void log(string text);
};

Planning::Planning(World *w, Measurement *s, const Performance specs) :
    world(*w), sense(*s), dist_compare_tol(specs.dist_compare_tol),
    max_rot(specs.max_rot), max_trans(specs.max_trans)
{
    planning_log.open("../logs/planning_log.txt", ios::out | ios::trunc);
    time_t now = time(0);
    log("Planning Log: " + string(ctime(&now)));
}

Planning::~Planning()
{
    planning_log.close();
}

sys_state Planning::plan(sys_state s)
{
    switch(s)
    {
    case STARTUP:
        return startup(s);
    case FIRST_LOCALIZATION:
        return localize();
    case STOP:
        return stop(s);
    default:
        break;
    }
    return s;
}


inline sys_state Planning::startup(sys_state s)
{
    double alignment_diff = fabs(sense.alignedToWall(LEFT));
    log("alignment_diff: " + to_string(alignment_diff));
    bool exit_in_front = false;
    for (int i = 0; i < world.exits.size(); ++i)
    {
        log(to_string((world.right.i+world.center.i)/3)+" "+to_string(world.exits[i].center.i)+" "+
            to_string((world.left.i+world.center.i)*2.0/3.0));
        if (world.exits[i].center.i > (world.right.i+world.center.i)/3 &&
                world.exits[i].center.i < (world.left.i+world.center.i)*2.0/3.0)
        {
            exit_in_front = true;
            break;
        }
    }
    log("exit in front: " + to_string(exit_in_front));
    if (exit_in_front && alignment_diff < dist_compare_tol)
        return FIRST_LOCALIZATION;
    world.des_vtheta = -max_rot/2;
    world.des_vx = 0;
    world.des_vy = 0;

    return s;
}

inline sys_state Planning::localize()
{
    for (int i = 0; i < world.exits.size(); ++i)
    {
        if (world.exits[i].center.i > (world.right.i+world.center.i)/3 &&
                world.exits[i].center.i < (world.left.i+world.center.i)*2.0/3.0)
        {

        }
    }
    return STOP;
}

inline sys_state Planning::stop(sys_state s)
{
    world.des_vtheta = 0;
    world.des_vx = 0;
    world.des_vy = 0;
    return STOP;
}

void Planning::log(string text)
{
#if PLANNING_LOG_FLAG == 1 || PLANNING_LOG_FLAG == 3
    planning_log << text << endl;
#endif
#if PLANNING_LOG_FLAG == 2 || PLANNING_LOG_FLAG == 3
    cout << "planner: " << text << endl;
#endif
}

#endif // PLANNING

