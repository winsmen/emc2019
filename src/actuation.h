#ifndef ACTUATION
#define ACTUATION

#include <emc/io.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include "common_resources.h"
#include "measurement.h"

#define ACTUATION_LOG_FLAG 1

#define ACC_TRANS           0.05
#define ACC_ROT             0.02

using namespace std;

class Actuation
{
    emc::IO &io;
    World &world;
    Measurement &m;
    const double min_permit_dist;
    const double dist_compare_tol;
    const double max_rot,max_trans,min_rot,min_trans;
    ofstream act_log;
    void log(string text);
    int getHeading();
public:
    Actuation(emc::IO *io, World *w, const Performance s, Measurement *m);
    ~Actuation();
    void actuate();
};

Actuation::Actuation(emc::IO *io, World *w, const Performance s, Measurement *m)
    : io(*io), world(*w), m(*m), min_permit_dist(s.min_permit_dist),
      dist_compare_tol(s.dist_compare_tol), max_rot(s.max_rot), max_trans(s.max_trans),
      min_rot(s.min_rot), min_trans(s.min_trans)
{
    act_log.open("../logs/actuation_log.txt", ios::out | ios::trunc);
    time_t now = time(0);
    log("Measurment Log: " + string(ctime(&now)));
}


Actuation::~Actuation()
{
    act_log.close();
}

void Actuation::log(string text)
{
#if ACTUATION_LOG_FLAG == 1 || ACTUATION_LOG_FLAG == 3
    act_log << text << endl;
#endif
#if ACTUATION_LOG_FLAG == 2 || ACTUATION_LOG_FLAG == 3
    cout << "actuator: " << text << endl;
#endif
}


void Actuation::actuate()
{
    // Set Rotational Velocity
    if (fabs(world.des_vtheta) <= min_rot)
        world.vtheta = 0;
    else if (world.des_vtheta > world.vtheta)
        world.vtheta += ACC_ROT;
    else if (world.des_vtheta < world.vtheta)
        world.vtheta -= ACC_ROT;
    // Saturate Rotational Velocity
    world.vtheta = min(max_rot,max(-max_rot,world.des_vtheta));

    // Set X Translational Velocity
    if (world.des_vx == 0)
        world.vx = 0;
    else if (world.des_vx > world.vx)
        world.vx += ACC_TRANS;
    else if (world.des_vx < world.vx)
        world.vx -= ACC_TRANS;
    // Saturate vx
    world.vx = min(max_trans,max(-max_trans,world.vx));

    // Set Y Translational Velocity
    if (world.des_vy == 0)
        world.vy = 0;
    else if (world.des_vy > world.vy)
        world.vy += ACC_TRANS;
    else if (world.des_vy < world.vy)
        world.vy -= ACC_TRANS;
    // Saturate vx
    world.vy = min(max_trans,max(-max_trans,world.vy));

    // Check for obstacle in immediate path
    if (!m.sectorClear(getHeading()))
    {
        world.vy /= 3;
        world.vx /= 3;
    }

    //Check for obstacle on right and left
    if (!world.right_clear || !m.sectorClear(world.right.i + 75))
        world.vy += min_trans;
    if (!world.left_clear || !m.sectorClear(world.left.i - 75))
        world.vy -= min_trans;
    world.vy = max(-min_trans, min(world.vy,min_trans));

    //Send computed values to base
    log("vx: " + to_string(world.vx) + "vy: " + to_string(world.vy) + "vtheta: " + to_string(world.vtheta) +
        " sector clear?: " + to_string(m.sectorClear(getHeading())));
    io.sendBaseReference(world.vx,world.vy,world.vtheta);
}


int Actuation::getHeading()
{
    return world.center.i-atan2(world.des_vy,world.des_vx)/m.getAngInc();
}

#endif // ACTUATION

