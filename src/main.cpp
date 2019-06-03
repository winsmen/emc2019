#include "main.h"
#include <string>
#include <iostream>
#include <vector>

#define PLAN_ENABLED        false
#define ACTUATE_ENABLED     false

#define MIN_PERMIT_DIST     0.6
#define DIST_COMPARE_TOL    0.01
#define CORNER_COMPARE_TOL  0.1
#define ANGLE_COMPARE_TOL   0.1
#define PADDING             15
#define AV_RANGE            20
#define SIDE_RANGE          20
#define MIN_RANGE           0.1
#define HEARTBEAT           10
#define MAX_ROT             0.5
#define MAX_TRANS           0.25

vector<int> cabinet_number;
void cabinet_order()
{
        int myint;
        cout << "Please enter the order of the cabinets (enter 1000 to end):\n";
        do {
    		cin >> myint;
    		cabinet_number.push_back (myint);
        } while (myint != "1000");
    	cabinet_number.pop_back();
        cout << int(cabinet_number.size()) << " cabinets will be reached.\n";
}

int main()
{
    cabinet_order();

    // Setup
    Performance specs(MIN_PERMIT_DIST,DIST_COMPARE_TOL,CORNER_COMPARE_TOL,ANGLE_COMPARE_TOL,
                      PADDING,AV_RANGE,SIDE_RANGE,MIN_RANGE,HEARTBEAT,MAX_TRANS,MAX_ROT);
    Robot pico(specs,STARTUP,cabinet_number);

    while(pico.io.ok())
    {
        if (pico.sense->measure() != 1)
        {
            pico.r.sleep();
            continue;
        }
        pico.map();
#if PLAN_ENABLED
        pico.plan();
#endif
#if PLAN_ENABLED && ACTUATE_ENABLED
        pico.actuate();
#endif

        //pico.r.sleep();
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
