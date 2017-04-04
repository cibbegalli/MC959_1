//#include <QCoreApplication>
#include "Robot.h"
#include "Simulator.h"
#include <iostream>
#include <unistd.h>

extern "C" {
   #include "extApi.h"
    #include "v_repLib.h"
}

int main(int argc, char *argv[])
{
    Robot *robot;
    Simulator *vrep = new Simulator("127.0.0.1", 25000);
    if (vrep->connect() ==-1){
        cout << "Failed to Connect" << endl;
        return 0;
    }

    robot = new Robot(vrep, "Pioneer_p3dx");

    robot->initializeSonars();
    
    for (int i=0; i<5000; ++i)
    {
        cout << "Here we go... " << i;
        robot->writeSonars();
        robot->writeGT();
        robot->adjustDirection(i);
        robot->update();        
        extApi_sleepMs(70);//extApi_sleepMs(50);
    }
    robot->writeGridMap();
    
    vrep->disconnect();
    
    return 0;
    //exit(0);
}
