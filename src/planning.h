#ifndef PLANNING
#define PLANNING

#include <fstream>
#include <iostream>
#include <ctime>

#include "common_resources.h"
#include "measurement.h"
#include "mapping.h"

#define PLANNING_LOG_FLAG 3

using namespace std;

class Planning
{
    emc::IO &io;
    World &world;
    Measurement &sense;
    Mapping &mapper;
    ofstream planning_log;
    double dist_compare_tol;
    double max_rot,max_trans;
    double min_angle,max_angle;
    double min_permit_dist;
    double ang_inc;
    vector<int> cabinet_list;
    int cab_num;

    sys_state startup(sys_state s);
    sys_state localise();
    sys_state getNextCabinet();
    sys_state goToDestination();
    sys_state atCabinet();
    sys_state stop(sys_state s);

public:
    Planning(emc::IO *io, World *w, Measurement *s, Mapping *m, const Performance specs);
    ~Planning();
    sys_state plan(sys_state s);
    void log(string text);
    void updateCabinetList(vector<int> &cabinet_list);
};

Planning::Planning(emc::IO *io, World *w, Measurement *s, Mapping *m, const Performance specs) :
    world(*w), sense(*s), dist_compare_tol(specs.dist_compare_tol), ang_inc(s->getAngInc()),
    max_rot(specs.max_rot), max_trans(specs.max_trans), min_permit_dist(specs.min_permit_dist),
    io(*io), mapper(*m)

{
    planning_log.open("../logs/planning_log.txt", ios::out | ios::trunc);
    time_t now = time(0);
    log("Planning Log: " + string(ctime(&now)));
    min_angle = sense.getMinAngle();
    max_angle = sense.getMaxAngle();
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
        return localise();

    case GET_NEXT_CABINET:
        return getNextCabinet();

    case GO_TO_DESTINATION:
        return goToDestination();

    case AT_CABINET:
        return atCabinet();

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
    if (exit_in_front && alignment_diff < 0.8*dist_compare_tol) //add side conditions
    {
        world.des_vtheta = 0;
        return FIRST_LOCALIZATION;
    }
    world.des_vtheta = -max_rot;
    world.des_vx = 0;
    world.des_vy = 0;

    return s;
}

inline sys_state Planning::localise()
{
    // Left Edge - Point 22 - x = 5.1, y = 4.4
    // Right Edge - Point 17 - x = 5.9, y = 4.4
    double x_diff,y_diff;
    double Rexit_x,Rexit_y,Lexit_x,Lexit_y;
    for (int i = 0; i < world.exits.size(); ++i)
    {
        if (world.exits[i].center.i > (world.right.i+world.center.i)/3 &&
                world.exits[i].center.i < (world.left.i+world.center.i)*2.0/3.0)
        {
            polar2cart(world.exits[i].leftEdge.d,world.exits[i].leftEdge.i*-sense.getAngInc()+max_angle,Lexit_x,Lexit_y);
            polar2cart(world.exits[i].rightEdge.d,world.exits[i].rightEdge.i*-sense.getAngInc()+max_angle,Rexit_x,Rexit_y);
            x_diff = (world.points[17].x-Rexit_x + world.points[22].x-Lexit_x)/2;
            y_diff = (world.points[17].y-Rexit_y + world.points[22].y-Lexit_y)/2;
        }
    }
    log("Number of exits: " + to_string(world.exits.size()));
    log("x_diff: " + to_string(x_diff));
    log("y_diff: " + to_string(y_diff));
    log("Lexit_x: " + to_string(Lexit_x) + " Rexit_x" + to_string(Rexit_x));
    log("Lexit_y: " + to_string(Lexit_y) + " Rexit_y" + to_string(Rexit_y));
    log("Left Exit Corner - x: " + to_string(world.points[22].x) + " y: " + to_string(world.points[22].y));
    log("Right Exit Corner - x: " + to_string(world.points[17].x) + " y: " + to_string(world.points[17].y));
    world.x = x_diff;
    world.y = y_diff;
    world.theta = 0;
    world.off_x = 0;
    world.off_y = 0;
    world.off_theta = 0;
    io.speak("Initial Localisation complete");
    return STOP;
}

inline sys_state Planning::getNextCabinet()
{
    if (cabinet_list.empty())
    {
        io.speak("No more cabinets to visit. Goodbye.");
        log("End of Cabinet List");
        sleep(4);
        return STOP;
    }
    cab_num = cabinet_list[0];
    cabinet_list.erase(cabinet_list.begin());
    log("Next Cabinet number: " + to_string(cab_num) + " at x: " + to_string(world.cabinets[cab_num].front.x) +
        " at y: " + to_string(world.cabinets[cab_num].front.y));
    io.speak("Next Cabinet " + to_string(cab_num));
    sleep(3);
    return GO_TO_DESTINATION;
}

inline sys_state Planning::goToDestination()
{
    log("Path Planning");
    // If reached destination
    world.des_vtheta = 0;
    world.des_vx = 0;
    world.des_vy = 0;
    world.vx = 0;
    world.vy = 0;
    world.vtheta = 0;
    return AT_CABINET;
}

inline sys_state Planning::atCabinet()
{
    log("At cabinet: " + to_string(cab_num));
    io.speak("Reached cabinet");
    mapper.captureImage(cab_num);
    world.des_vtheta = 0;
    world.des_vx = 0;
    world.des_vy = 0;
    world.vx = 0;
    world.vy = 0;
    world.vtheta = 0;
    return GET_NEXT_CABINET;
}

inline sys_state Planning::stop(sys_state s)
{
    world.des_vtheta = 0;
    world.des_vx = 0;
    world.des_vy = 0;
    world.vx = 0;
    world.vy = 0;
    world.vtheta = 0;
    return STOP;
}

void Planning::updateCabinetList(vector<int> &cabinet_list)
{
    this->cabinet_list = cabinet_list;
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

