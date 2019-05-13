#include "main.h"

using namespace std;

int main()
{
    // Setup
    cout << "Starting up" << endl;
    int pico_rate = 10;
    float maxRot = 0.5;
    float maxTrans = 0.5;
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

    pico.io.speak("Goodbye");
    pico.io.sendBaseReference(0,0,0);

    cout << "Hello World" << endl;
    return 0;
}
