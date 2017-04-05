#include "Robot.h"

Robot::Robot(Simulator *sim, string name) {
    this->sim = sim;
    this->name = name;
    handle = sim->getHandle(name);

    if (LOG) {
        FILE *data =  fopen("files/gt.txt", "wt");
        if (data!=NULL)
            fclose(data);
        data =  fopen("files/sonar.txt", "wt");
        if (data!=NULL)
            fclose(data);
        data = fopen("files/gridmapbysonar.txt", "wt");
        if (data!=NULL)
            fclose(data);
        data = fopen("files/gridmapbylaser.txt", "wt");
        if (data!=NULL)
            fclose(data);
    }

    /* Get handles of sensors and actuators */
    encoderHandle[0] = sim->getHandle("Pioneer_p3dx_leftWheel");
    encoderHandle[1] = sim->getHandle("Pioneer_p3dx_rightWheel");
    cout << "Left Encoder: "<< encoderHandle[0] << endl;
    cout << "Right Encoder: "<< encoderHandle[1] << endl;

    /* Get handles of sensors and actuators */
    motorHandle[0] = sim->getHandle("Pioneer_p3dx_leftMotor");
    motorHandle[1] = sim->getHandle("Pioneer_p3dx_rightMotor");
    cout << "Left motor: "<<  motorHandle[0] << endl;
    cout << "Right motor: "<<  motorHandle[1] << endl;

    simxChar sensorName[32];
    /* Connect to sonar sensors. Requires a handle per sensor. Sensor name: Pioneer_p3dx_ultrasonicSensorX, where
     * X is the sensor number, from 1 - 16 */
    for(int i = 0; i < 16; i++)
    {
        sprintf(sensorName,"%s%d","Pioneer_p3dx_ultrasonicSensor",i+1);
        sonarHandle[i] = sim->getHandle(sensorName);
        if (sonarHandle[i] == -1){
            cout <<  "Error on connecting to sensor " + (i+1) << endl;
        }
        else {
        	cout << (i+1)  << " connected to sensor \n";    
        }
    }

    /* Get the robot current absolute position */
    sim->getObjectPosition(handle,robotPosition);
    sim->getObjectOrientation(handle,robotOrientation);

    initialPose[0]=robotPosition[0];
    initialPose[1]=robotPosition[1];
    initialPose[2]=robotOrientation[2]; 
    for(int i = 0; i < 3; i++) {
        robotLastPosition[i] = FLT_MAX; 
    }

    /* Get the encoder data */
    sim->getJointPosition(motorHandle[0],&encoder[0]);
    sim->getJointPosition(motorHandle[1],&encoder[1]);
    cout << "Set enconder=[" << encoder[0] << "," << encoder[1] << "]" << endl;

    /* Initialize random seed */
    srand(time(NULL));

    int n = DIMENSION_SCENE/INTERVAL_GRID;
    cout << "Grid Map: " << n << "x" << n << endl;
    gridMapbySonars.resize(n, vector<int> (n, 0));
    gridMapbyLaser.resize(n, vector<bool> (n, false));

    // define random target 
    targetCoord[0] = ((rand() % n)*(-INTERVAL_GRID)) + MAX_X;
    targetCoord[1] = ((rand() % n)*(-INTERVAL_GRID)) + MAX_Y;

    cout << "Target(RANDOM) x=" << targetCoord[0] << ", y=" << targetCoord[1] << endl;

    direction = FORWARD;

	simxGetStringSignal(sim->getId(),"ScannerData",&laserScannerData,&dataSize,simx_opmode_streaming);
	
	cout << "--------------------------------------\n";
}

void Robot::initializeSonars() {
	/* Initialize sonars */
    for(int i = 0; i < NUM_SONARS; i++)
    {

        simxUChar state;       // sensor state
        simxFloat coord[3];    // detected point coordinates [only z matters]

        simxReadProximitySensor(sim->getId(),sonarHandle[i],&state,coord,NULL,NULL,simx_opmode_streaming);

        if(state > 0 ) {
        	sonarReadings[i] = coord[2];
        } else {
        	sonarReadings[i] = -1;
        }
    	lastSonarReadings[i] = -1;
    }
}

enum Quadrant {
	Q1 = 1,
	Q2 = 2,
	Q3 = 3,
	Q4 = 4
};

Quadrant quadrant(float angle) {
	if(angle >=0 && angle <= 90)
		return Q1;
	if(angle > 90 && angle <= 180)
		return Q2;
	if(angle < 0 && angle >= -90) 
		return Q4;
	return Q3;
}

float absoluteValue(float x) {
	if(x < 0) return (-1)*x;
	return x;
}

void Robot::adjustDirection(int i) {

	if(i % 5 > 0 || i == 0) {
		return;
	}

	float orientationTarget;
	float epsilon = 0.001;

	float diff_X = targetCoord[0]-robotPosition[0];
	float diff_Y = targetCoord[1]-robotPosition[1];
	if(absoluteValue(diff_X) > epsilon && absoluteValue(diff_Y) > epsilon) {
		
		float m = (targetCoord[1]-robotPosition[1])/(targetCoord[0]-robotPosition[0]);
		double angle = atan((double)m)*180/PI;

		if(targetCoord[0] <  robotPosition[0] && targetCoord[1] > robotPosition[1]) {
			orientationTarget = (float)(180 + angle);
		} else if(targetCoord[0] <  robotPosition[0] && targetCoord[1] < robotPosition[1]){
			orientationTarget = (float)(-180 + angle);
		} else{
			orientationTarget = (float) angle;
		}
		
		Quadrant quadrantRobot = quadrant(robotOrientation[2]);
		Quadrant quadrantTarget = quadrant(orientationTarget);

		
		if(quadrantRobot != quadrantTarget) {

			if((quadrantRobot != Q4 && quadrantRobot < quadrantTarget) ||
				(quadrantRobot == Q4 && quadrantRobot > quadrantTarget)) {
				direction = LEFT; 
			} else {
				direction = RIGHT; 
				
			}

		} else {

			if(robotOrientation[2] <= orientationTarget) {
				direction = LEFT;
			}
			else {
				direction = RIGHT;
			}

		}

	} else {
		direction = FORWARD;
	}

	if(DEBUG) {
		cout << "-adjustDirection\n";
		cout << "\tRobot position =(" << robotPosition[0] << "," << robotPosition[1] <<")\n";
		cout << "\tTarget position =(" << targetCoord[0] << "," << targetCoord[1] <<")\n";
		cout << "\tRobot orientation=" << robotOrientation[2] << " -> " << orientationTarget << " - direction " << direction << endl;
		cout << "\n";
	}
}

void Robot::update(int i) {
    braitenberg(); 
    //drive(10,0);
    updateSensors();
    updatePose();//?? 
    
    stop();
    //extApi_sleepMs(50);
    updateLaser();

    updateGridMap(i);
    braitenberg(); 
}

void Robot::braitenberg() {

    double maxDetectionDist = 0.2;
    double noDetectionDist = 0.6;
    
    double wL[8] = {-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6};
    double wR[8] = {-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2};

    double detect[8];

    for(int i=0; i<NUM_SONARS/2; i++) {
    	double dist = sonarReadings[i];
    	if(dist != -1 && dist < noDetectionDist) {
    		if(dist < maxDetectionDist) {
    			dist = maxDetectionDist;	
    		}
    		detect[i] = 1.0-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist));
    	} else {
    		detect[i] = 0.0;
    	}
    }
    
    double vLeft = 2.0;
    double vRight = 2.0;
    
    for(int i=0; i<NUM_SONARS/2; i++) {
	    vLeft=vLeft+wL[i]*detect[i];
        vRight=vRight+wR[i]*detect[i];
    }

    if(vLeft >= 1.4 && vRight >= 1.4) {
	    if(direction == LEFT) {
			vLeft -= 0.20;
		} else if(direction == RIGHT) {
			vRight -= 0.20;			
		}
    }

    sim->setJointTargetVelocity(motorHandle[0], vLeft);
    sim->setJointTargetVelocity(motorHandle[1], vRight);

    if(DEBUG) {
    	cout << "-braitenberg\n";
    	cout << "\tvLeft=" << vLeft << " vRight=" << vRight << endl;	
    }
}

void Robot::updateSensors() {
    /* Update sonars */
    for(int i = 0; i < NUM_SONARS; i++)
    {
    	lastSonarReadings[i] = sonarReadings[i];

        simxUChar state;       // sensor state
        simxFloat coord[3];    // detected point coordinates [only z matters]

        /* simx_opmode_streaming -> Non-blocking mode */

        /* read the proximity sensor
         * detectionState: pointer to the state of detection (0=nothing detected)
         * detectedPoint: pointer to the coordinates of the detected point (relative to the frame of reference of the sensor) */
        if (sim->readProximitySensor(sonarHandle[i], &state, coord)==1)
        {
            if(state > 0)
                sonarReadings[i] = coord[2];
            else
                sonarReadings[i] = -1;

        }
    }

    /* Update encoder data */
    lastEncoder[0] = encoder[0];
    lastEncoder[1] = encoder[1];

    /* Get the encoder data */
    if(DEBUG) {
    	cout << "-updateEncoders\n";
    	if (sim->getJointPosition(motorHandle[0], &encoder[0]) == 1) {
        	cout << "\tok left enconder"<< encoder[0] << endl;  // left
	    }
	    if (sim->getJointPosition(motorHandle[1], &encoder[1]) == 1){
	        cout << "\tok right enconder"<< encoder[1] << endl;  // right
	    }
	    cout << endl;
    }
}

void Robot::updatePose() {
    for(int i = 0; i < 3; i++)
    {
        robotLastPosition[i] = robotPosition[i];
    }

    /* Get the robot current position and orientation */
    sim->getObjectPosition(handle,robotPosition);
    sim->getObjectOrientation(handle,robotOrientation);

    if(DEBUG) {
    	cout << "-updatePose";
    	cout << "\tPosition = (" << robotPosition[0] << ","<<robotPosition[1] << "," << robotPosition[2] <<")\n";
    	cout << "\tOrientation = (" << robotOrientation[0] << ","<<robotOrientation[1] << "," << robotOrientation[2] <<")\n";
    	cout << "\n";
    }
}

/*	
	data=simPackFloats(points) -- points is a table that contains all scanned points (3 values per point)
    simSetStringSignal("ScannerData",data)
    */
void Robot::updateLaser() {

	//if(DEBUG) {
     	cout << "-updateLaser" << robotPosition[0] << " - " << robotPosition[1] << robotOrientation[2] << "\n"; 
     	for (int i=0;i<dataSize/(4*3);i++) { // if each point has 3 coordinates
			float x = ((simxFloat*)(laserScannerData+4*3*i))[0];
			float y = ((simxFloat*)(laserScannerData+4*3*i))[1];
			//float z = ((simxFloat*)(laserScannerData+4*3*i))[2];
			cout << "(" << x << "," << y << ")" << "\t";	
		}
		cout << "\n";

	//}
	for(int i=0; i<dataSize; i++) {
		//lastScannerData[i] = laserScannerData[i];
	}
	simxGetStringSignal(sim->getId(),"ScannerData",&laserScannerData,&dataSize,simx_opmode_buffer);
	


    //}
}

void Robot::calcPositionObstacle(float dist, int sonar, double& sonarReadingX, double& sonarReadingY) {

	float xTruth = robotPosition[0];
	float yTruth = robotPosition[1];
	float alpha = robotOrientation[2];
	float angle = alpha + sonarAngles[sonar];

	sonarReadingX = xTruth + ((dist+R) * cos(angle*PI/180));
	sonarReadingY = yTruth + ((dist+R) * sin(angle*PI/180));
}

void Robot::calcPositionObstacleLaser(float laserReadingX, float laserReadingY,float& coordX, float& coordY) {

	float x1 = robotPosition[0] + laserReadingX;
	float y1 = robotPosition[1] + laserReadingY;

	float angle = robotOrientation[2];

	coordX = x1*cos(angle*PI/180) - y1*sin(angle*PI/180);
	coordY = x1*sin(angle*PI/180) + y1*cos(angle*PI/180);
}


void Robot::updateGridMap(int i) {
	if(i < 20) return;

	if(DEBUG) {
		cout << "-updateGridMap\n";
	}

	bool different = false;
    for(int i = 0; (i < NUM_SONARS && !different); i++) {
    	if(lastSonarReadings[i] != sonarReadings[i]) {
    		different = true;
    	}
    }
    if(different) {

    	for(int i = 0; (i < NUM_SONARS); i++) {

    		if(sonarReadings[i] != -1) {
    		//if(sonarReadings[i] != -1 && sonarReadings[i] < 0.7) {
    		
    			double sonarReadingX;
    			double sonarReadingY;
    		
    			calcPositionObstacle(sonarReadings[i], i, sonarReadingX, sonarReadingY);
    		
    			int c = (-sonarReadingX + MAX_X)/INTERVAL_GRID;
    			int l = (-sonarReadingY + MAX_Y)/INTERVAL_GRID;
    		
				if(gridMapbySonars[c][l] < 10) {
    				gridMapbySonars[c][l]++;	
    			}
    		
    			if(DEBUG) {
    				cout << "sonarReading X=" << sonarReadingX << ", Y=" << sonarReadingY << " => [" << c << "," << l << "]" << endl;	
    			}
    		}
    	}
    }

    int n = DIMENSION_SCENE/INTERVAL_GRID;

    for(int i=0; i<dataSize/(4*3); i++) {

		//calcPositionObstacle()    	
    	float laserReadingX = ((simxFloat*)(laserScannerData+4*3*i))[0];
		float laserReadingY = ((simxFloat*)(laserScannerData+4*3*i))[1];
		

		float coordX, coordY;
		calcPositionObstacleLaser(laserReadingX, laserReadingY, coordX, coordY);
		//coordX = ((simxFloat*)(laserScannerData+4*3*i))[0] + robotPosition[0];
		//coordY = ((simxFloat*)(laserScannerData+4*3*i))[1] + robotPosition[1];

		int c = (-coordX + MAX_X)/INTERVAL_GRID;
		int l = (-coordY + MAX_Y)/INTERVAL_GRID;

		if(c >= 0 && l >= 0 && c < n && l < n) {
			gridMapbyLaser[c][l] = true;
		}
    }
}

void Robot::writeGridMap() {
	

	if (LOG) {
    	int n = DIMENSION_SCENE/INTERVAL_GRID;

        FILE *data =  fopen("files/gridmapbysonar.txt", "at");
        
        if (data!=NULL)
        {
        	for(int i=0; i<n; i++) {
				for(int j=0; j<n; j++) {
					if(gridMapbySonars[i][j] > 0) {					
						float x = (i * INTERVAL_GRID) - MAX_X;
						float y = (j * INTERVAL_GRID) - MAX_Y;
						fprintf(data, "%.3f\t%.3f\n", x, y);
					}
				}
			}
            fflush(data);
	        fclose(data);
        }

        data = fopen("files/gridmapbylaser.txt", "at");
		if (data!=NULL)
        {
        	for(int i=0; i<n; i++) {
				for(int j=0; j<n; j++) {
					if(gridMapbyLaser[i][j]) {
						float x =(i * INTERVAL_GRID) - MAX_X;
						float y =(j * INTERVAL_GRID) - MAX_Y;
						fprintf(data, "%.3f\t%.3f\n", x, y);
					}
				}
			}
		}
    }
}

void Robot::writeGT() {
    /* write data to file */
    /* file format: robotPosition[x] robotPosition[y] robotPosition[z] robotLastPosition[x] robotLastPosition[y] robotLastPosition[z]
     *              encoder[0] encoder[1] lastEncoder[0] lastEncoder[1] */

	float epsilon = 0.001;
    if (LOG) {
    	bool different = false;
    	for (int i=0; i<3 && !different; ++i) {
    		if(absoluteValue(robotPosition[i]-robotLastPosition[i]) > epsilon) different = true;
    	}
    	for(int i=0; i<2 && !different; ++i) {
    		if(absoluteValue(encoder[i]-lastEncoder[i]) > epsilon) different = true;	
    	}

    	if(different) {
    		FILE *data =  fopen("files/gt.txt", "at");
	        if (data!=NULL)
	        {
	            for (int i=0; i<3; ++i)
	                fprintf(data, "%.2f\t",robotPosition[i]);
	            for (int i=0; i<3; ++i)
	                fprintf(data, "%.2f\t",robotLastPosition[i]);
	            for (int i=0; i<3; ++i)
	                fprintf(data, "%.2f\t",robotOrientation[i]);
	            for (int i=0; i<2; ++i)
	                fprintf(data, "%.2f\t",encoder[i]);
	            for (int i=0; i<2; ++i)
	                fprintf(data, "%.2f\t",lastEncoder[i]);
	            fprintf(data, "\n");
	            fflush(data);
	            fclose(data);
	          }
	          else
	            cout << "Unable to open file gt.txt\n";
    	}
    }
}

void Robot::writeSonars() {
    /* write data to file */
    float epsilon = 0.0001;
    if (LOG) {
    	bool different = false;
    	for (int i=0; i<NUM_SONARS && !different; ++i) {
    		if(absoluteValue(sonarReadings[i]-lastSonarReadings[i]) > epsilon) different = true;
    	}

    	//if(different) {
    		FILE *data =  fopen("files/sonar.txt", "at");
	        if (data!=NULL)
	        {

        		for (int i=0; i<NUM_SONARS; ++i) {
                	fprintf(data, "%.3f\t",sonarReadings[i]);
                }
                fprintf(data, "\n");
                fflush(data);
                fclose(data);	
	        }
	        else
	            cout << "Unable to open file sonar.txt\n";	
    	//}
    }
}

void Robot::printSonars() {
	cout << "sonarReadings = [ ";
	for(int i = 0; i < NUM_SONARS; i++) {
		cout << sonarReadings[i] << " ";
    } 
    cout << "]\n";
}

void Robot::stop() {
    sim->setJointTargetVelocity(motorHandle[0], 0);
    sim->setJointTargetVelocity(motorHandle[1], 0);
}

void Robot::drive(double vLinear, double vAngular) {
    sim->setJointTargetVelocity(motorHandle[0], vLToDrive(vLinear,vAngular));
    sim->setJointTargetVelocity(motorHandle[1], vRToDrive(vLinear,vAngular));
}

double Robot::vRToDrive(double vLinear, double vAngular) {
    return (((2*vLinear)+(L*vAngular))/2*R);
}

double Robot::vLToDrive(double vLinear, double vAngular) {  
    return (((2*vLinear)-(L*vAngular))/2*R);
}

/*
void Robot::move(float vLeft, float vRight) {
    sim->setJointTargetVelocity(motorHandle[0], vLeft);
    sim->setJointTargetVelocity(motorHandle[1], vRight);
}
*/
