#include "Robot.h"

Robot::Robot(Simulator *sim, string name) {
        this->sim = sim;
        this->name = name;
        cout << "Iniciando robô ...\n";
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

        cout << " > Adquirindo handles...\n";

        /* Get handles of sensors and actuators */
        encoderHandle[0] = sim->getHandle("Pioneer_p3dx_leftWheel");
        encoderHandle[1] = sim->getHandle("Pioneer_p3dx_rightWheel");
        cout << "  >> Left Wheel Encoder Handle: "<< encoderHandle[0] << endl;
        cout << "  >> Right Wheel Encoder Handle: "<< encoderHandle[1] << endl;

        /* Get handles of sensors and actuators */
        motorHandle[0] = sim->getHandle("Pioneer_p3dx_leftMotor");
        motorHandle[1] = sim->getHandle("Pioneer_p3dx_rightMotor");
        cout << "  >> Left Motor Encoder Handle: "<<  motorHandle[0] << endl;
        cout << "  >> Right Motor Encoder Handle: "<<  motorHandle[1] << endl;

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
                        cout << "  >> Sonar Handle (" << (i+1) << "/" << NUM_SONARS << "): " << sonarHandle[i] << "\n";    
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
        //cout << "Set enconder=[" << encoder[0] << "," << encoder[1] << "]" << endl;

        /* Initialize random seed */
        srand(time(NULL));

        int n = DIMENSION_SCENE/INTERVAL_GRID;
        cout << " > Iniciando grid map de dimensão: " << n << "x" << n << endl;
        gridMapbySonars.resize(n, vector<int> (n, 0));
        gridMapbyLaser.resize(n, vector<int> (n, 0)); //gridMapbyLaser.resize(n, vector<bool> (n, false));

        // define random target 
        targetCoord[0] = ((rand() % n)*(-INTERVAL_GRID)) + MAX_X;
        targetCoord[1] = ((rand() % n)*(-INTERVAL_GRID)) + MAX_Y;

        cout << "> Inicializando coordenada target aleatoriamente. Target: x=" << targetCoord[0] << ", y=" << targetCoord[1] << endl;

        while(simxGetObjectFloatParameter(sim->getId(),motorHandle[0],sim_objfloatparam_abs_rot_velocity,&vEsq,simx_opmode_streaming) != 0);
        while(simxGetObjectFloatParameter(sim->getId(),motorHandle[1],sim_objfloatparam_abs_rot_velocity,&vDir,simx_opmode_streaming) != 0);
        while(simxGetObjectFloatParameter(sim->getId(),encoderHandle[0],sim_objfloatparam_abs_rot_velocity,&vEsq,simx_opmode_streaming) != 0);
        while(simxGetObjectFloatParameter(sim->getId(),encoderHandle[1],sim_objfloatparam_abs_rot_velocity,&vDir,simx_opmode_streaming) != 0);
        cout << "> Colocando motores em modo de leitura...\n";

        direction = FORWARD;

        simxGetStringSignal(sim->getId(),"ScannerData",&laserScannerData,&dataSize,simx_opmode_streaming);
        simxGetStringSignal(sim->getId(),"ScannerData2",&laserScannerData2,&dataSize2,simx_opmode_streaming);
        cout << "Fim da inicialização do robô.\n";
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


void Robot::update(int i) {

        // após o primeiro update pose, grava a posicao e orientacao iniciais do robo
        if (i==1) {
                initialOrientation[2] = robotOrientation[2];
                initialPosition[0] = robotPosition[0];
                initialPosition[1] = robotPosition[1];
        }

        vEsqOld = vEsq;
        vDirOld = vDir;
        // coleta velocidade nas rodas
        while(simxGetObjectFloatParameter(sim->getId(),encoderHandle[0],sim_objfloatparam_abs_rot_velocity,&vEsq,simx_opmode_buffer) != 0);
        while(simxGetObjectFloatParameter(sim->getId(),encoderHandle[1],sim_objfloatparam_abs_rot_velocity,&vDir,simx_opmode_buffer) != 0);
        // cout << "vEsq: " << vEsq  << " | vDir: " << vDir << "\n";
        lastTime = currentTime;
        currentTime = simxGetLastCmdTime(sim->getId());
        diffTime = currentTime - lastTime;
        // se passou algum tempo na referencia do simulador
        if (lastTime != currentTime) {
                // calcular por sorvetão a distancia percorrida entre a iteracao anterior
                // e a atual para as duas rodas
                // s =     s0 +              v0*t +                      (a*t^2)/2
                //RelEsq =  0 + (vEsqOld*R * diffTime/1000) + ((vEsq*R - vEsqOld*R)/1000*diffTime)/2;
                RelEsq = (vEsqOld*R * diffTime/1000) + ((vEsq*R - vEsqOld*R)*diffTime/1000)/2;
                RelDir = (vDirOld*R * diffTime/1000) + ((vDir*R - vDirOld*R)*diffTime/1000)/2;
                // calcula o deslocaomento médio
                RelDif = RelEsq - RelDir;
                // a é o angulo em radianos em relacao à orientacao inicial do robo
                a += (RelDif/L);//*1.44;
                // acumula o quanto foi andado em x e y com relacao à orientacao inicial do robo
                xRel += ((RelEsq + RelDir)/2)* (-cos(a));
                yRel += ((RelEsq + RelDir)/2)* sin(a);
                xOd = xRel*cos(initialOrientation[2]) - yRel*sin(initialOrientation[2]);
                yOd = xRel*sin(initialOrientation[2]) + yRel*cos(initialOrientation[2]);
                //angle = a;
        }
        updatePose();
        updateSensors(); 
        updateLaser();
        updateGridMap(i);

        /* Descomente apenas _UMA_ das linhas abaixo para selecionar o método
         * de navegaçao */
        braitenberg(i); 
        //wallFollow();

        if (i == 999) {
                //printa GT
                cout << " | (GT): " << initialPosition[0] - robotPosition[0] << ", " << initialPosition[1] - robotPosition[1] << "\n";
                //printa odometria
                cout << "a: " << a << " | Gt_a: " << initialOrientation[2] << " | Final :" << xRel*cos(initialOrientation[2]) - yRel*sin(initialOrientation[2]) << ", " << xRel*sin(initialOrientation[2]) + yRel*cos(initialOrientation[2]) << "\n";
        }
}

void Robot::ajusteDireita() {
        viraDireita(16);
        viraEsquerda(13);
        //cout << "Ajustando para a direita...\n";

}
void Robot::ajusteEsquerda() {
        viraEsquerda(16);
        viraDireita(13);
        //cout << "Ajustando para a esquerda...\n";
}

void Robot::viraDireita(int vLinear)  {
        sim->setJointTargetVelocity(motorHandle[0], vLToDrive(vLinear,0));
}

void Robot::viraEsquerda(int vLinear)  {
        sim->setJointTargetVelocity(motorHandle[1], vRToDrive(vLinear,0));
}

bool isCriticalFrontDistance(float* sonarReadings) {
        if ((fabs(sonarReadings[2]) < 0.15 ||
        fabs(sonarReadings[3]) < 0.15 ||
        fabs(sonarReadings[4]) < 0.15 ||
        fabs(sonarReadings[5]) < 0.15 ||
        fabs(sonarReadings[6]) < 0.15) || (fabs(sonarReadings[3]) < 0.5 &&
        fabs(sonarReadings[4] < 0.5)))
                return true;
        
        return false;


}

void Robot::wallFollow() {
        double wallDist = 0.2;

        int closeSonarsEsq, closeSonarsDir, criticalFrontDistance;
        closeSonarsEsq = closeSonarsDir = criticalFrontDistance = 0;

        // Conta quantos sensores passaram do threshold
        for(int i=0; i<(NUM_SONARS) - 1; i++) {
                if (fabs(sonarReadings[i]) < wallDist) {
                        if (i >= 4 && i <= 11)
                                closeSonarsDir++;
                        else if (i <= 3 || i >= 12) 
                                closeSonarsEsq++;
                }
        }


        if (isCriticalFrontDistance(sonarReadings) && !desvFront) {
                cout << "tem bagaça na frente...\n";
                desvFront = frontWall = 1;
                followingRightWall = followingLeftWall = turnLeft = turnRight = 0;
                Robot::stop();
                return;
        }

        while(simxGetObjectFloatParameter(sim->getId(),encoderHandle[0],sim_objfloatparam_abs_rot_velocity,&vEsq,simx_opmode_buffer) != 0);
        while(simxGetObjectFloatParameter(sim->getId(),encoderHandle[1],sim_objfloatparam_abs_rot_velocity,&vDir,simx_opmode_buffer) != 0);

        // esperando parar
        if (stopping && (vEsq || vDir))  {
                cout << "parando...\n";
                return;
        }
        // parado
        // stopping, vEsq e vDir = 0
        else {
                stopping = 0;
                if (frontWall) {
                        if (fabs(sonarReadings[0]) > fabs(sonarReadings[7]))
                                turnLeft = 1;
                        else
                                turnRight = 1;
                        frontWall = 0;
                }
                if (turnLeft) { 
                        viraEsquerda(20);
                        // alinha sensores laterais paralelos a parede
                        if (!desvFront && sonarReadings[7] != -1 && sonarReadings[8] != -1 && fabs(sonarReadings[7] - sonarReadings[8]) < 0.005) {
                                cout << "ALINHOU\n";
                                turnLeft = 0;
                                followingRightWall = 1;
                                drive(15,0);
                        // desvio de parede
                        } else if (desvFront && !isCriticalFrontDistance(sonarReadings)) {
                                cout << "DESVIOU DE PAREDE\n";
                                desvFront = 0;
                                turnLeft = 0;
                                followingRightWall = 1;
                                drive(15,0);
                        }
                        // fim de parede
                        else if (!desvFront && fabs(sonarReadings[1]) < 0.3) {
                                cout << "FEZ CURVA P/ SEGUIR PAREDE\n";
                                turnLeft = 0;
                                followingLeftWall = 0;
                                drive(15,0);
                        }
                        return;
                }
                else if (turnRight) {
                        viraDireita(15);
                        // caso em que alinha sensores da esquerda
                        if (!desvFront && sonarReadings[0] != -1 && sonarReadings[15] != -1 && fabs(sonarReadings[0] - sonarReadings[15]) < 0.005) {
                                cout << "ALINHOU\n";
                                turnRight = 0;
                                followingLeftWall = 1;
                                drive(15,0);
                        // desvio de parede
                        } else if (desvFront && !isCriticalFrontDistance(sonarReadings)) {
                                cout << "DESVIOU DE PAREDE\n";
                                desvFront = 0;
                                turnRight = 0;
                                followingLeftWall = 1;
                                drive(15,0);
                        } else if (!desvFront && fabs(sonarReadings[6]) < 0.3) {
                                cout << "FEZ CURVA P/ SEGUIR PAREDE\n";
                                turnRight = 0;
                                followingRightWall = 0;
                                drive(15,0);
                        }
                        return;
                }
        }

        if (!followingRightWall && !followingLeftWall)  {
                if (closeSonarsDir > 2) {
                        cout << "too close right\n";
                        Robot::stop();
                        turnLeft = 1;
                } else if (closeSonarsEsq > 2) {
                        cout << "too close left\n";
                        Robot::stop();
                        turnRight = 1;
                }
                else {
                        cout << "deu drive\n";
                        drive(15,0);
                }
        } else if (followingRightWall) {
                // encontrou fim de uma parede
                if (fabs(sonarReadings[7]) > 0.6) {
                        viraDireita(15);
                        if (fabs(sonarReadings[8]) > 0.45) {
                                turnRight = 1;
                                Robot::stop();
                        }
                }
                else if (sonarReadings[7] < sonarReadings[8]) {
                        ajusteEsquerda();
                }
                else  {
                        ajusteDireita();
                }
        } else if (followingLeftWall) {
                // encontrou fim de uma parede
                if (fabs(sonarReadings[0]) >= 0.6) {
                        viraEsquerda(15);
                        if (fabs(sonarReadings[15]) >= 0.45) {
                                turnLeft = 1;
                                Robot::stop();
                        }
                }
                else if (sonarReadings[0] < sonarReadings[15]) {
                        ajusteDireita();
                }
                else  {
                        ajusteEsquerda();
                }
        }
}

void Robot::braitenberg(int i) {

        double maxDetectionDist = 0.2;
        double noDetectionDist = 0.5;

        double wL[8] = {-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6};
        double wR[8] = {-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2};

        double detect[8];
        double vLeft = 4.0;
        double vRight = 4.0;

        while(simxGetObjectFloatParameter(sim->getId(),encoderHandle[0],sim_objfloatparam_abs_rot_velocity,&vEsq,simx_opmode_buffer) != 0);
        while(simxGetObjectFloatParameter(sim->getId(),encoderHandle[1],sim_objfloatparam_abs_rot_velocity,&vDir,simx_opmode_buffer) != 0);

        if (i == 1) {
                sim->setJointTargetVelocity(motorHandle[0], vLeft);
                sim->setJointTargetVelocity(motorHandle[1], vRight);
                return;
        }

        if (inverting) {
                inverting--;
                cout << "invertendo... " << inverting << " | vEsq,vDir: " << vEsq << "," << vDir << "\n";
                return;
        }

        // Le metade dos sonares
        for(int i=1; i<(NUM_SONARS/2) - 1; i++) {
                double dist;
                if (!rear) {
                        dist = sonarReadings[i];
                } else {
                        dist = sonarReadings[i+(NUM_SONARS/2) - 1];
                }

                // Se leitura do sonar válida e em distancia legivel
                if(dist != -1 && dist < noDetectionDist) {
                        // Se leitura do sonar for mais proxima que a maxima aceitavel,
                        // atualize a leitura do sonar
                        if(dist < maxDetectionDist) {
                                dist = maxDetectionDist;	
                        }
                        // O vetor detect contem valores entre 0 e 1. Quanto mais proximo
                        // o objeto detectado, mais perto de 1 será o valor.
                        detect[i] = 1.0-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist));
                } else {
                        // Se o valor for zero, o objeto está fora da área de decisao do
                        // algoritmo.
                        detect[i] = 0.0;
                }
        }

        // Conta quantos sensores passaram do threshold
        int count_close_sonars = 0, too_close = 0;
        for(int i=1; i<(NUM_SONARS/2) - 1; i++) {
                if (detect[i] > 0.6)
                        count_close_sonars++;
                if (detect[i] >0.9 && i >= 3 && i <= 4)
                        too_close = 1;
        }

        // Decisao se deve dar ré
        int k = rand();
        if (count_close_sonars >= 4 || too_close) {
                cout << "MUITO PERTO! Invertendo rotacao das rodas...\n";
                rear = 1 - rear;
                inverting = 50;
                if (k%2) {
                        vLeft = 8.0;
                        vRight = 4.0;
                } else {
                        vLeft = 4.0;
                        vRight = 8.0;
                }
        }

        if(rear) {
                vLeft = -1 * vLeft;
                vRight = -1 * vRight;
                for(int i=0; i<NUM_SONARS/2; i++)
                        detect[i] = -1 * detect[i];
        }

        if (inverting) {
                sim->setJointTargetVelocity(motorHandle[0], vLeft);
                sim->setJointTargetVelocity(motorHandle[1], vRight);
                return;
        }

        // Para cada elemento de detect setado, acumula 
        for(int i=0; i<NUM_SONARS/2; i++) {
                if (!rear) {
                        vLeft=vLeft+((wL[i]*detect[i]));
                        vRight=vRight+((wR[i]*detect[i]));
                }
                else {
                        vLeft=vLeft-((wL[i]*detect[i]));
                        vRight=vRight-((wR[i]*detect[i]));
                }
        }

        // Adicionado pela Cibelle: fatores randomicos de locomocao
        if (!rear) {
                if(vLeft >= 1.6 && vRight >= 1.6 && (i % 200) == 0) { 
                        if (k%2) {
                                vLeft = 4.0;
                                vRight = -4.0;
                        } else {
                                vLeft = -4.0;
                                vRight = 4.0;
                        }
                        //cout << i<< "Ajuste velocidade para pegar mais dados da cena\n";
                }
        } else {
                if(vLeft <= -1.6 && vRight <= -1.6 && (i % 200) == 0) {
                        if (k%2) {
                                vLeft = 4.0;
                                vRight = -4.0;
                        } else {
                                vLeft = -4.0;
                                vRight = 4.0;
                        }
                        //cout << i<< "Ajuste velocidade para pegar mais dados da cena\n";
                }

        }

        if (i%200) {
                sim->setJointTargetVelocity(motorHandle[0], vLeft);
                sim->setJointTargetVelocity(motorHandle[1], vRight);
        }


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
                //cout << sonarReadings[i] << " ";
        }
        //cout << "\n";

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

        //cout << "Position = (" << robotPosition[0] << ","<<robotPosition[1] << "," << robotPosition[2] <<")/Orientation = (" << robotOrientation[0] << ","<<robotOrientation[1] << "," << robotOrientation[2] <<")\n";
}

/*	
        data=simPackFloats(points) -- points is a table that contains all scanned points (3 values per point)
        simSetStringSignal("ScannerData",data)
        */
void Robot::updateLaser() {
        simxGetStringSignal(sim->getId(),"ScannerData",&laserScannerData,&dataSize,simx_opmode_buffer);

        if(DEBUG) {
                cout << "-updateLaser" << robotPosition[0] << " - " << robotPosition[1] << robotOrientation[2] << "\n"; 
                for (int i=0;i<dataSize/(4*3);i++) { // if each point has 3 coordinates
                        float x = ((simxFloat*)(laserScannerData+4*3*i))[0];
                        float y = ((simxFloat*)(laserScannerData+4*3*i))[1];
                        //float z = ((simxFloat*)(laserScannerData+4*3*i))[2];
                        cout << "(" << x << "," << y << ")" << "\t";	
                }
                cout << "\n";

        }

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

        //float x1 = robotPosition[0] + laserReadingX;
        //float y1 = robotPosition[1] + laserReadingY;

        //float angle = robotOrientation[2];

        //coordY = x1*cos(angle*PI/180.0) - y1*sin(angle*PI/180.0);
        //coordX = x1*sin(angle*PI/180.0) + y1*cos(angle*PI/180.0);

        coordX = -laserReadingX;
        coordY = -laserReadingY;
}


void Robot::updateGridMap(int i) {
        if(DEBUG) {
                cout << "-updateGridMap\n";
        }

        bool different = false;
        for(int i = 0; (i < NUM_SONARS && !different); i++) {
                if(lastSonarReadings[i] != sonarReadings[i]) {
                        different = true;
                }
        }

if(i % 5 != 0 ) return;

int n = DIMENSION_SCENE/INTERVAL_GRID;

for(int i=0; i<dataSize/(4*3); i++) {

        float laserReadingX = ((simxFloat*)(laserScannerData+4*3*i))[0];
        float laserReadingY = ((simxFloat*)(laserScannerData+4*3*i))[1];


        float coordX, coordY;
        calcPositionObstacleLaser(laserReadingX, laserReadingY, coordX, coordY);

        int c = (-coordX  + MAX_X)/INTERVAL_GRID;
        int l = (-coordY + MAX_Y)/INTERVAL_GRID;

        if(c >= 0 && l >= 0 && c < n && l < n) {
                gridMapbyLaser[c][l]++;//gridMapbyLaser[c][l] = true;
        }
}

for(int i=0; i<dataSize2/(4*3); i++) {

        float laserReadingX = ((simxFloat*)(laserScannerData2+4*3*i))[0];
        float laserReadingY = ((simxFloat*)(laserScannerData2+4*3*i))[1];


        float coordX, coordY;
        calcPositionObstacleLaser(laserReadingX, laserReadingY, coordX, coordY);

        int c = (-coordX + MAX_X)/INTERVAL_GRID;
        int l = (-coordY + MAX_Y)/INTERVAL_GRID;

        if(c >= 0 && l >= 0 && c < n && l < n) {
                gridMapbyLaser[c][l]++;//gridMapbyLaser[c][l] = true;
        }
}
//}
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
                                                fprintf(data, "%.3f\t%.3f\t%d\n", x, y, gridMapbyLaser[i][j]);
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
                                for (int i=0; i<2; ++i)
                                        fprintf(data, "%.2f\t",robotPosition[i]);
                                fprintf(data, "%.2f\t", -xOd+initialPosition[0]);
                                fprintf(data, "%.2f\t", -yOd+initialPosition[1]);
                                /*
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
                                */
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
        stopping = 1;
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
