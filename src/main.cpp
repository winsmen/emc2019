#include "main.h"
#include <iostream>
#include <vector>
#include <string>

#define PLAN_ENABLED        true
#define ACTUATE_ENABLED     true

#define MIN_PERMIT_DIST     0.6
#define DIST_COMPARE_TOL    0.01
#define CORNER_COMPARE_TOL  0.08
#define ANGLE_COMPARE_TOL   0.1
#define PADDING             15
#define AV_RANGE            30
#define SIDE_RANGE          50
#define MIN_RANGE           0.1
#define HEARTBEAT           10
#define MAX_ROT             1.2
#define MAX_TRANS           0.5


void readCabinetList(int argc, char *argv[]);


int main(int argc, char *argv[])
{
    // cabinet order
    
    // Setup
    Performance specs(MIN_PERMIT_DIST,DIST_COMPARE_TOL,CORNER_COMPARE_TOL,ANGLE_COMPARE_TOL,
                      PADDING,AV_RANGE,SIDE_RANGE,MIN_RANGE,HEARTBEAT,MAX_TRANS,MAX_ROT);
    Robot pico(specs,STARTUP);

    //Read Cabinet List
    {
        if (argc < 2)
        {
            pico.log("No inputs provided. Using default list: 0 1 2 3");
            pico.io.speak("No inputs provided. Using default list: 0 1 2 3");
            pico.cabinet_list.push_back(0);
            pico.cabinet_list.push_back(1);
            pico.cabinet_list.push_back(2);
            pico.cabinet_list.push_back(3);
        }
        else
        {
            string text = "I will visit cabinets ";
            for (int i = 1; i < argc; ++i)
            {
                pico.cabinet_list.push_back(int(argv[i][0])-'0');
                if (i == argc-1 && i != 2)
                    text += ("and ");
                text += to_string(pico.cabinet_list.back()) + " ";
            }
            pico.log(text);
            pico.io.speak(text);
            sleep(3);
        }
    }

    int loss_count = 0;
    while(pico.io.ok())
    {
        //cout << 1;
        if (pico.sense->measure() != 1)
        {
            pico.r.sleep();
            loss_count += 1;
            if (loss_count >= 10)
            {
                cout << "Warning: Sensing failure for more than 10 iterations! Switching off locomotion.";
                pico.io.sendBaseReference(0,0,0);
            }
            continue;
        }
        //cout << 2;
        pico.map->identify();
        //break;
        //cout << 3 <<endl;
#if PLAN_ENABLED
        pico.setState(pico.planner->plan(pico.getState()));
#endif
#if PLAN_ENABLED && ACTUATE_ENABLED
        pico.actuator->actuate();
#endif

        pico.r.sleep();
//        if (pico.state == STOP)
//            break;*/
        loss_count = 0;
    }
    //sleep(10);
    return 0;
}

// TODO
/*
 * measure least distance to visible walls
 *
 */
