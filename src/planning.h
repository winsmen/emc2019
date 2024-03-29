#ifndef PLANNING
#define PLANNING

#include <fstream>
#include <iostream>
#include <ctime>

#include "common_resources.h"
#include "measurement.h"
#include "mapping.h"
#include "astar.h"

#define PLANNING_LOG_FLAG 3

using namespace std;

class Planning
{
    emc::IO &io;
    World &world;
    Measurement &sense;
    Mapping &mapper;
    ofstream planning_log;
    double dist_compare_tol,angle_compare_tol;
    double max_rot,max_trans,min_rot,min_trans;
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
    io(*io), mapper(*m), min_rot(specs.min_rot), min_trans(specs.min_trans),
    angle_compare_tol(specs.angle_compare_tol)

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

    case FIRST_LOCALISATION:
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
    // Get Left Wall Alignment Measure
    double alignment_diff = fabs(sense.alignedToWall(LEFT));
    log("alignment_diff: " + to_string(alignment_diff));

    // Check if Exit is in front sector
    bool exit_in_front = false;
    for (int i = 0; i < world.exits.size(); ++i)
    {
        if (world.exits[i].center.i > (world.right.i+world.center.i)/3 &&
                world.exits[i].center.i < (world.left.i+world.center.i)*2.0/3.0)
        {
            exit_in_front = true;
            break;
        }
    }
    log("exit in front: " + to_string(exit_in_front) + " right_dist: " + to_string(world.right.d));

    // If aligned to left wall and exit visible in front sector and right is clear,
    // switch to FIRST_LOCALISATION
    if (exit_in_front && alignment_diff < 0.7*dist_compare_tol && world.right.d > 2*min_permit_dist)
    {
        world.des_vtheta = 0;
        world.des_vx = 0;
        world.des_vy = 0;
        return FIRST_LOCALISATION;
    }

    // If exit not visible in front sector -> full speed turn,
    // else use alignment value for vtheta for gradual stop
    // alignment_diff max value ~= 0.9
    if (!exit_in_front)
        world.des_vtheta = -max_rot;
    else
        world.des_vtheta = max(-max_rot,min(-min_rot,-alignment_diff*10*max_rot));

    // No Translational movement at startup
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

    // Chek if exit is lost
    log("Number of exits: " + to_string(world.exits.size()));
    if (world.exits.size() == 0)
        return STARTUP;

    // Load cartesian coordinates of Exit located in front sector only
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
    log("world.Lexit_x: " + to_string(Lexit_x) + " world.Rexit_x" + to_string(Rexit_x));
    log("world.Lexit_y: " + to_string(Lexit_y) + " world.Rexit_y" + to_string(Rexit_y));
    log("Map Left Exit Corner - x: " + to_string(world.points[22].x) + " y: " + to_string(world.points[22].y));
    log("Map Right Exit Corner - x: " + to_string(world.points[17].x) + " y: " + to_string(world.points[17].y));

    // Set robot x,y position to difference between detected and provided exit coordinates
    world.x_off = x_diff - world.x;
    world.y_off = y_diff - world.y;
    world.theta_off = 0 - world.theta;
    world.x = x_diff;
    world.y = y_diff;
    world.theta = 0;
    log("world.x_off: " + to_string(world.x_off) + " world.y_off: " + to_string(world.y_off) + " world.theta_off: " + to_string(world.theta_off));
    log("world.x: " + to_string(world.x) + " world.y: " + to_string(world.y) + " world.theta: " + to_string(world.theta));
    io.speak("Initial Localisation complete");
    //sleep(4);
    return GET_NEXT_CABINET;
}

inline sys_state Planning::getNextCabinet()
{
    // Check if list is empty
    if (cabinet_list.empty())
    {
        io.speak("No more cabinets to visit. Goodbye.");
        log("End of Cabinet List");
        sleep(4);
        return STOP;
    }

    // Get next cabinet and store coordinates as destination for path planning
    cab_num = cabinet_list[0];
    log("Next Cabinet number: " + to_string(cab_num) + " at x: " + to_string(world.cabinets[cab_num].front.x) +
        " at y: " + to_string(world.cabinets[cab_num].front.y));
    world.des_x = world.cabinets[cab_num].front.x;
    world.des_y = world.cabinets[cab_num].front.y;
    io.speak("Next Cabinet " + to_string(cab_num));
    //sleep(3);
    return GO_TO_DESTINATION;
}

inline sys_state Planning::goToDestination()
{
    // Set source and destination points to feed to path planner
    log("src: " + to_string(world.x) + " " + to_string(world.y));
    log("dest: " + to_string(world.des_x) + " " + to_string(world.des_y));
    Pair src = make_pair(world.y/MAP_RES,world.x/MAP_RES);
    Pair dest = make_pair(world.des_y/MAP_RES,world.des_x/MAP_RES);

    // Run path planner
    aStarSearch(world.global_gridmap,src,dest);
    log(to_string(path_x.size()));
    if (!path_x.empty())
    {
        world.path_x = path_x;
        world.path_y = path_y;
    }
    if (world.path_x.empty())
        return GO_TO_DESTINATION;
    if (distance(world.path_x[0]*MAP_RES,world.path_y[0]*MAP_RES,world.x,world.y) >=
            distance(world.path_x[1]*MAP_RES,world.path_y[1]*MAP_RES,world.x,world.y))
    {
        world.path_x.erase(world.path_x.begin());
        world.path_y.erase(world.path_y.begin());
    }
    if (world.path_x.empty())
        return GO_TO_DESTINATION;
    // Calculate vtheta to face expected direction of motion
    int next = min(int(world.path_x.size())-1,10);
    double angle = atan2(world.path_x[0]-world.path_x[next],world.path_y[next]-world.path_y[0]);
    if (fabs(angle - world.theta) > angle_compare_tol)
    {
        // Compute vtheta based on deviation from expected direction
        // If difference is greater than pi/4, vtheta = +/-max_rot
        if (fabs(angle - world.theta) < M_PI)
            world.des_vtheta = max(-1.0,min(1.0,(angle - world.theta)/(M_PI/4)))*max_rot;
        else
            world.des_vtheta = -max(-1.0,min(1.0,(angle - world.theta)/(M_PI/4)))*max_rot;
    }
    else
        world.des_vtheta = 0;

    // Resolve velocity vector for current movement
    // if pico is facing more than 60 degrees away from path direction, do not translate
    if (fabs(angle - world.theta) > (M_PI/6))
    {
        world.des_vx = 0;
    }
    else
    {
        world.des_vx = max_trans*cos(angle-world.theta);
    }

    // No right-left movement
    world.des_vy = 0;

    cout << "vx: " << world.des_vx << " vy: " << world.des_vy << endl;

    //If reached destination
    if (distance(world.x,world.y,world.des_x,world.des_y) < 0.1)
        return AT_CABINET;
    else
        return GO_TO_DESTINATION;
}

inline sys_state Planning::atCabinet()
{
    log("Cabinet direction: " + to_string(world.cabinets[cabinet_list[0]].dir) + " angle: " + to_string(world.theta));
    if (fabs(world.cabinets[cabinet_list[0]].dir - world.theta) > M_PI/4)
    {
        // Compute vtheta based on deviation from expected direction
        // If difference is greater than pi/4, vtheta = +/-max_rot
        if (fabs(world.cabinets[cabinet_list[0]].dir  - world.theta) < M_PI)
            world.des_vtheta = max(-1.0,min(1.0,(world.cabinets[cabinet_list[0]].dir - world.theta)/(M_PI/4)))*max_rot;
        else
            world.des_vtheta = -max(-1.0,min(1.0,(world.cabinets[cabinet_list[0]].dir - world.theta)/(M_PI/4)))*max_rot;
        world.vx = 0;
        world.vy = 0;
        return AT_CABINET;
    }
    else
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

        cabinet_list.erase(cabinet_list.begin());
        return GET_NEXT_CABINET;
    }
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

