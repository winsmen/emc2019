#include "main.h"
#include <iostream>
#include <vector>
#include <string>

#define PLAN_ENABLED        true
#define ACTUATE_ENABLED     true

#define MIN_PERMIT_DIST     0.4
#define DIST_COMPARE_TOL    0.01
#define CORNER_COMPARE_TOL  0.08
#define ANGLE_COMPARE_TOL   0.1
#define PADDING             15
#define AV_RANGE            30
#define SIDE_RANGE          50
#define MIN_RANGE           0.1
#define HEARTBEAT           10
#define MAX_ROT             0.6
#define MAX_TRANS           0.25
#define MIN_ROT             0.2
#define MIN_TRANS           0.1


void readCabinetList(int argc, char *argv[]);


int main(int argc, char *argv[])
{
    // Setup
    Performance specs(MIN_PERMIT_DIST,DIST_COMPARE_TOL,CORNER_COMPARE_TOL,ANGLE_COMPARE_TOL,
                      PADDING,AV_RANGE,SIDE_RANGE,MIN_RANGE,HEARTBEAT,MAX_TRANS,MAX_ROT,MIN_TRANS,MIN_ROT);
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
        pico.planner->updateCabinetList(pico.cabinet_list);
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
                pico.log("Warning: Sensing failure for more than 10 iterations! Switching off locomotion.");
                pico.io.sendBaseReference(0,0,0);
            }
            continue;
        }
        //cout << 2;
        if (pico.world.x == -1 || pico.world.y == -1) //world x and y values are -1 at program start till the first localization is performed
            pico.map->identify();
        else
            pico.map->localise();
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
//        {
//            pico.printState();
//            break;
//        }
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


// Weighted Map creator test

//const int map_x = 15;
//const int map_y = 15;
//int sgmap[map_y][map_x] = {{0}};
//int gmap[map_y][map_x];

//for (int i = 0; i < map_x; ++i)
//{
//    sgmap[0][i] = 1;
//    sgmap[map_y-1][i] = 1;
//    sgmap[i][0] = 1;
//    sgmap[i][map_x-1] = 1;
//}
//sgmap[7][7] = 1;

//int layers = 5;
//fill_n(&gmap[0][0],sizeof(gmap)/sizeof(**gmap),0);
//for (int i = 0; i < map_x; ++i)
//{
//    for (int j = 0; j < map_y; ++j)
//    {
//        if (sgmap[i][j] == 1)
//        {
//            for (int k = 0; k <= layers; ++k)
//            {
//                for (int l = -k; l <= k; ++l)
//                {
//                    if (j+k < map_y && i+l > -1 && i+l < map_x && gmap[j+k][i+l] < layers-k)
//                        gmap[j+k][i+l] = layers-k;
//                    if (j-k > -1 && i+l > -1 && i+l < map_x && gmap[j-k][i+l] < layers-k)
//                        gmap[j-k][i+l] = layers-k;
//                    if (i+k < map_x && j+l > -1 && j+l < map_y && gmap[j+l][i+k] < layers-k)
//                        gmap[j+l][i+k] = layers-k;
//                    if (i-k > -1 && j+l > -1 && j+l < map_y && gmap[j+l][i-k] < layers-k)
//                        gmap[j+l][i-k] = layers-k;
//                }
//            }
////                for (int i = 0; i < map_x; ++i)
////                {
////                    for (int j = 0; j < map_y; ++j)
////                    {
////                        cout << gmap[j][i];
////                    }
////                    cout << "  ";
////                    for (int j = 0; j < map_y; ++j)
////                    {
////                        cout << sgmap[j][i];
////                    }
////                    cout << endl;
////                }
//        }
//    }
//}
//for (int i = 0; i < map_x; ++i)
//{
//    for (int j = 0; j < map_y; ++j)
//    {
//        cout << gmap[j][i];
//    }
//    cout << "  ";
//    for (int j = 0; j < map_y; ++j)
//    {
//        cout << sgmap[j][i];
//    }
//    cout << endl;
//}
//return 0;
