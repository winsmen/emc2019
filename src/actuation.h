#ifndef ACTUATION
#define ACTUATION

#include <emc/io.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include "common_resources.h"
#include "measurement.h"

#define ACTUATION_LOG_FLAG 3

#define ACC_TRANS           0.005
#define ACC_ROT             0.04

using namespace std;

class Actuation
{
    emc::IO &io;
    World &world;
    Measurement &m;
    const double min_permit_dist;
    const double dist_compare_tol;
    const float max_rot;
    const float max_trans;
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
      dist_compare_tol(s.dist_compare_tol), max_rot(s.max_rot), max_trans(s.max_trans)
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
    double max_rot_ = max_rot, max_trans_ = max_trans;
    int i = getHeading();
    int forward_clear = m.sectorClear(i);
    if (forward_clear == 0 || forward_clear == 3)
        max_trans_ = 0;
    else if (forward_clear == 2)
        max_trans_ /= 2;
    //log("sector clear: " + to_string(i));

    // Set Rotational Velocity
    //Turn to face heading direction if no theta velocity is supplied
    if (world.des_vtheta == 0)
    {
        world.des_vtheta = world.des_vy/max_trans_*max_rot_;
        world.vtheta = 0;
    }
    //Accelerate/Decelerate if desired and actual theta velocities are unequal
    if (world.des_vtheta-world.vtheta > 0)
        world.vtheta += ACC_ROT;
    else if (world.des_vtheta-world.vtheta < 0)
        world.vtheta -=ACC_ROT;
    //Shift to 0 if velocity direction has switched.
    else if (fabs(world.des_vtheta-world.vtheta) > max_rot_)
        world.vtheta = 0;
    //Saturate to maximum permissable velocity
    if (world.vtheta > max_rot_)
        world.vtheta = max_rot_;
    else if (world.vtheta < -max_rot_)
        world.vtheta = -max_rot_;

    // Y-direction velocity
    //Accelerate/Decelerate if desired and actual Y velocities are unequal
    if (world.des_vy-world.vy > 0)
        world.vy += ACC_ROT;
    else if (world.des_vy-world.vy < 0)
        world.vy -= ACC_ROT;
    //Shift to 0 if velocity direction has switched
    else if (fabs(world.des_vy-world.vy) > max_trans_)
        world.vy = 0;
    //Saturate to maximum permissable velocity
    if (world.vy > max_trans_)
        world.vy = max_trans_;
    else if (world.vy < -max_trans_)
        world.vy = -max_trans_;

    // X-direction velocity
    //Accelerate/Decelerate if desired and actual X velocities are unequal
    if (world.des_vx-world.vx > 0)
        world.vx += ACC_ROT;
    else if (world.des_vx-world.vx < 0)
        world.vx -= ACC_ROT;
    //Shift to 0 if velocity direction has switched
    else if (fabs(world.des_vx-world.vx) > max_trans_)
        world.vx = 0;
    //Saturate to maximum permissable velocity
    if (world.vx > max_trans_)
        world.vx = max_trans_;
    else if (world.vx < -max_trans_)
        world.vx = -max_trans_;

    //log("Vtheta: " + to_string(world.vtheta));
    //Send computed values to base
    io.sendBaseReference(world.vx,world.vy,world.vtheta);
}


int Actuation::getHeading()
{
    return world.center.i-atan2(world.des_vy,world.des_vx)/m.getAngInc();
}

#endif // ACTUATION

