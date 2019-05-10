#include "main.h"

using namespace std;

int main()
{
    // Setup
    int pico_rate = 10;
    robot pico(pico_rate);
    pico.state = STARTUP;
    pico.maxRot = 1.2;
    pico.maxTrans = 0.5;

    pico.startup();
    pico.io.speak("Pico Ready!");

    pico.state = SCAN_FOR_EXIT;

    while(pico.io.ok())
    {
        pico.measure();
        pico.map();
        pico.plan();
        pico.actuate();
//        int locIndex = -1;
//        do
//        {
//            pico.measure();
//            pico.io.sendBaseReference(0,0,pico.maxTrans);
//            locIndex = pico.locateExit();
//            pico.r.sleep();
//        }while(locIndex <= -1);
//        pico.io.sendBaseReference(0,0,0);
//        cout << "turning to exit" << endl;
//        sleep(2);
//        pico.faceExit(locIndex);
        break;
    }
    pico.io.sendBaseReference(0,0,0);

    cout << "Hello World" << picoReady() << endl;
	return 0;
}
