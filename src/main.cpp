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
    pico.min_distance = 0.05;

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
    pico.io.sendBaseReference(0,pico.maxTrans,0);  //Command to drive in the direction straight forward towards the exit (Since the pico is facing the exit)

    if(std::min(pico.measure)<=pico.min_distance)  //Once pico comes to close to the opening, it alligns itself again and makes sure it drives trough the opening in a straight manner. Note that this loop has to be repeated several times and this statement should be checked real time.
    {
        pico.io.sendBaseReference(0,0,0);  //This will ensure that pico stands still
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
    }

    pico.io.sendBaseReference(0,pico.maxTrans,0);
    cout << "Destination reached" << picoReady() << endl;
	return 0;
}
