    /*visionHandle = sim->getHandle("Pioneer_p3dx_vision_sensor");
    if(visionHandle == -1) {
    	cout << "Error on connecting to vision sensor\n";
    } else {
    	cout << "Connected to vision sensor\n";
    }

    laserHandle = sim->getHandle("Pioneer_p3dx_laser");
	if(laserHandle == -1) {
    	cout << "Error on connecting to laser sensor\n";
    } else {
    	cout << "Connected to laser sensor\n";
    }*/


//QUESTIONAR PROFESSOR PORQUE PARA POSIÇÃO 2 pegou valor da orientação?
/*


if (sim_call_type==sim_childscriptcall_actuation) then 
    for i=1,16,1 do
        res,dist=simReadProximitySensor(usensors[i])
        if (res>0) and (dist<noDetectionDist) then
            if (dist<maxDetectionDist) then
                dist=maxDetectionDist
            end
            detect[i]=1-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist))
        else
            detect[i]=0
        end
    end
    
    vLeft=v0
    vRight=v0
    
    for i=1,16,1 do
        vLeft=vLeft+braitenbergL[i]*detect[i]
        vRight=vRight+braitenbergR[i]*detect[i]
    end
    
    simSetJointTargetVelocity(motorLeft,vLeft)
    simSetJointTargetVelocity(motorRight,vRight)
end 

*/