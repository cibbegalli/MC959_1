//#include <QCoreApplication>
#include "Robot.h"
#include "Simulator.h"
#include <iostream>
#include <unistd.h>
#include <iomanip>

extern "C" {
#include "extApi.h"
#include "v_repLib.h"
}

int main(int argc, char *argv[])
{
        Robot *robot;
        std::cout << std::fixed;
        std::cout << std::setprecision(4);
        Simulator *vrep = new Simulator("127.0.0.1", 25000);
        if (vrep->connect() ==-1){
                cout << "Failed to Connect" << endl;
                return 0;
        }

        robot = new Robot(vrep, "Pioneer_p3dx");

        robot->initializeSonars();

        int num_iteracoes = NUM_ITERATIONS;
        for (int i=0; i<num_iteracoes; ++i)
        {
                if (!(i%10)) {
                cout << "Iteração (" << (i+1) << "/" << num_iteracoes << ")\n";
                }
                robot->writeSonars();
                if (i >= 1)
                        robot->writeGT();
                robot->update(i);        
                extApi_sleepMs(50);
        }
        robot->writeGridMap();

        vrep->disconnect();

        return 0;
}
