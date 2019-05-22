#include "main.h"
#include <iostream>
#include <fstream>

int main()
{
    // Setup
    cout << "Starting up" << endl;
    int pico_rate = 10;
    float maxRot = 0.8;
    float maxTrans = 0.25;
    robot pico(pico_rate,maxTrans,maxRot);
    pico.io.speak("Pico Ready!");

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
    }
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
