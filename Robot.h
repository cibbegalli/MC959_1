#ifndef ROBOT_H
#define ROBOT_H

#define NUM_ITERATIONS 5000
#define DEBUG 0
#define NUM_SONARS  16
#define LOG         1
#define DIMENSION_SCENE 13.50
#define INTERVAL_GRID 0.05
#define PI 3.14159265
#define MAX_X 5.6
#define MAX_Y 7.8

#include <fstream>
#include <iostream>
#include "Simulator.h"
#include <stdlib.h>
#include <time.h>
#include <string>
#include <vector>
#include <math.h>
#include <cfloat>

extern "C" {
   #include "extApi.h"
   #include "v_repLib.h"
}

using namespace std;

typedef enum DIRECTION {
    FORWARD,
    LEFT, 
    RIGHT
} DirectionType;


class Robot
{
public:
    Robot(Simulator *sim, std::string name);
    void initializeSonars();
    void adjustDirection(int i);
    void update(int i);
    void braitenberg(int i);
    void wallFollow();
    void updateSensors();
    void updatePose();
    void updateLaser();
    void ajusteEsquerda();
    void ajusteDireita();
    void viraEsquerda(int vLinear);
    void viraDireita(int vLinear);
    void updateGridMap(int i);
    void writeGridMap();
    
    void calcPositionObstacle(float dist, int sonar, double& sonarReadingX, double& sonarReadingY);
    void calcPositionObstacleLaser(float laserReadingX, float laserReadingY,float& coordX, float& coordY);

    simxFloat vEsq, vDir, vEsqOld, vDirOld;
    simxFloat vLinear, vAngular;
    simxInt rear = 0;
    int vConst = 0, countConst = 0;
    int stopping = 0, turnRight = 0, turnLeft = 0, followingRightWall = 0, followingLeftWall, frontWall = 0, desvFront = 0;
    simxFloat initialPosition[3] = {0,0,0};
    simxFloat RelEsq = 0, RelDir = 0, RelDif = 0, a = 0, initial_a, xRel = 0, yRel = 0, xOd, yOd;
    simxInt timeStart, currentTime, lastTime, diffTime;
    int rotated = 0;
    void writeGT();
    void writeSonars();
    void printSonars();
    void stop();
    void drive(double vLinear, double vAngular);
    double vRToDrive(double vLinear, double vAngular);
    double vLToDrive(double vLinear, double vAngular);
    //void move(float vLeft, float vRight);

private:
    const float L = 0.381;                                   // distance between wheels
    const float R = 0.0975;                                  // wheel radius
    std::string name;
    Simulator *sim;

    simxInt handle;                                        // robot handle
    simxFloat velocity[2] = {0,0};                         // wheels' speed
    simxInt sonarHandle[16];                               // handle for sonars
    simxInt visionHandle;
    simxInt laserHandle;
    simxInt motorHandle[2] = {0,0};                        // [0]-> leftMotor [1]->rightMotor
    simxInt encoderHandle[2] = {0,0};
    simxFloat encoder[2] = {0,0};
    simxFloat lastEncoder[2] = {0,0};
    
    /* Robot Position  */
    simxFloat robotPosition[3] = {0,0,0};                    // current robot position
    simxFloat robotOrientation[3] = {0,0,0};                 // current robot orientation
    simxFloat initialOrientation[3] = {0,0,0};
    float initialPose[3] = {0,0,0};
    simxFloat robotLastPosition[3] = {0,0,0};                // last robot position
    simxInt inverting = 0;
    
    float sonarReadings[NUM_SONARS];
    float lastSonarReadings[NUM_SONARS];
    float sonarAngles[8] = {90, 50, 30, 10, -10, -30, -50, -90};

    vector<vector<int> > gridMapbySonars;
    int countReadings;


    simxUChar* laserScannerData;
    simxInt dataSize;

    simxUChar* laserScannerData2;
    simxInt dataSize2;

    vector<vector<int> > gridMapbyLaser;
    
    simxFloat targetCoord[2];
    DirectionType direction;
};

#endif // ROBOT_H
