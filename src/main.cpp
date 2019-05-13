#include "main.h"

using namespace std;

int main()
{
    // Setup
    cout << "Starting up" << endl;
    int pico_rate = 10;
    float maxRot = 1.2;
    float maxTrans = 0.5;
    robot pico(pico_rate,maxTrans,maxRot);
    pico.io.speak("Pico Ready!");

    pico.state = FIND_WALL;
    cout << "Pico State: STARTUP" << endl;
//pico.io.sendBaseReference(0.5,0.5,0);
//sleep(1);
//pico.io.speak("Done");
//return 0;
//    for (int i = 0; i < pico.scan_range; ++i)
//    {
//        cout << pico.scan.ranges[i] << " ";
//    }
//    cout << endl;
//    return 0;
    while(pico.io.ok())
    {
        if (pico.measure() != 1)
        {
            pico.r.sleep();
            continue;
        }
//        for (int i = 0; i < pico.scan_range; ++i)
//        {
//            cout << pico.scan.ranges[i] << " ";
//        }
        pico.map();
//        break;
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
