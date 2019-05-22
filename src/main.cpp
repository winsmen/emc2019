#include "main.h"

int main()
{
    // Setup
    int pico_rate = 10;
    float maxRot = 0.8;
    float maxTrans = 0.25;
    robot pico(pico_rate,maxTrans,maxRot);

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

    return 0;
}

// TODO
/*
 * measure least distance to visible walls
 *
 */
