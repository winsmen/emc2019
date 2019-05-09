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

    sleep(2);
    pico.io.speak("Waiting for Sensors to startup");
    pico.startup();
    pico.io.speak("Pico Ready!");

    while(pico.io.ok())
    {
        int locIndex = -1;
        do
        {
            pico.getSensorData();
            pico.io.sendBaseReference(0,0,pico.maxTrans);
            locIndex = pico.locateExit();
            pico.r.sleep();
        }while(locIndex <= -1);
        pico.io.sendBaseReference(0,0,0);
        cout << "turning to exit" << endl;
        sleep(2);
        pico.faceExit(locIndex);
        break;
    }
    pico.io.sendBaseReference(0,0,0);

    cout << "Hello World" << picoReady() << endl;
	return 0;
}
