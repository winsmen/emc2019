#include "main.h"
#include <iostream>
#include <fstream>

using namespace std;

int main()
{
    // Setup
    cout << "Starting up" << endl;
    int pico_rate = 10;
    float maxRot = 0.8;
    float maxTrans = 0.5;
    robot pico(pico_rate,maxTrans,maxRot);
    pico.io.speak("Pico Ready!");

    ofstream outfile;
    outfile.open("laser.csv", ios::out | ios::trunc);
    outfile << "state" << ',' << "n_corners" << ',' << "found_corridor"  << ',' <<"dist_center"  << "dist_right"  << ','<< "dist_left"  << ','<< "vx"  << ','<< "vy"  << ','<< "vtheta" << endl;

    pico.state = STARTUP;
    cout << "Pico State: STARTUP" << endl;
    while(pico.io.ok())
    {
        if (pico.measure() != 1)
        {
            pico.r.sleep();
            continue;
        }
        pico.map();
        pico.plan();
        pico.actuate();

        pico.r.sleep();
        if (pico.state == STOP)
            break;
        outfile << state << ',' << n_corners << ',' << found_corridor << dist_center << dist_right << dist_left << vx << vy << vtheta << endl;

    }
    outfile.close();
    pico.io.speak("I am free!!");
    pico.io.sendBaseReference(0,0,0);

    cout << "Goodbye!" << endl;
    return 0;
}

// TODO
/*
 * pico setState, getState functions to assign and read robot state - merge with printState()
 * measure least distance to visible walls
 *
 */
