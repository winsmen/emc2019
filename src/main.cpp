#include "main.h"

#define PLAN_ENABLED    true
#define ACTUATE_ENABLED true

int main()
{
    // Setup
    int pico_rate = 10;
    float maxRot = 0.5;
    float maxTrans = 0.25;
    robot pico(pico_rate,maxTrans,maxRot,STARTUP);

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
