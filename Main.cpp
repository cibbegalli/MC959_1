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
    
    int num_iteracoes = 5000;
    for (int i=0; i<num_iteracoes; ++i)
    {
        cout << "Iteração (" << (i+1) << "/" << num_iteracoes << ")\n";
        robot->writeSonars();
        robot->writeGT();
        robot->adjustDirection(i);
        robot->update(i);        
        extApi_sleepMs(50);
    }
    robot->writeGridMap();
    
    vrep->disconnect();
    
    return 0;
}
