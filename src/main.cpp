#include "main.h"

#define PLAN_ENABLED        true
#define ACTUATE_ENABLED     true

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

int main()
{
    // Setup
    Performance specs(MIN_PERMIT_DIST,DIST_COMPARE_TOL,CORNER_COMPARE_TOL,ANGLE_COMPARE_TOL,
                      PADDING,AV_RANGE,SIDE_RANGE,MIN_RANGE,HEARTBEAT,MAX_TRANS,MAX_ROT);
    Robot pico(specs,STARTUP);

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
