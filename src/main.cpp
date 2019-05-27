#include "main.h"

#define PLAN_ENABLED    1
#define ACTUATE_ENABLED 1

int main()
{
    // Setup
    int pico_rate = 10;
    float maxRot = 0.5;
    float maxTrans = 0.25;
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
