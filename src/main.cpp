#include "main.h"
#include <iostream>
#include <vector>

#define PLAN_ENABLED    1
#define ACTUATE_ENABLED 1

void cabinet_order()
{
        int myint;
        vector<int> cabinet_number;
        cout << "Please enter the order of the cabinets (enter 1000 to end):\n";
        do {
    		cin >> myint;
    		cabinet_number.push_back (myint);
        } while (myint!=1000);
    	cabinet_number.pop_back();
        cout << int(cabinet_number.size()) << " cabinets will be reached.\n";
}
        
int main()
{
    // Setup
    int pico_rate = 10;
    float maxRot = 0.5;
    float maxTrans = 0.25;

    // type in cabinet order
    cabinet_order();
    robot pico(pico_rate,maxTrans,maxRot,STARTUP);

    while(pico.io.ok())
    {
        if (pico.measure() != 1)
        {
            pico.r.sleep();
            continue;
        }
        pico.map();
#if PLAN_ENABLED == 1
        pico.plan();
#endif
#if ACTUATE_ENABLED == 1
        pico.actuate();
#endif

        pico.r.sleep();
        if (pico.state == STOP)
            break;
    }

    return 0;
}

// TODO
/*
 * measure least distance to visible walls
 *
 */
