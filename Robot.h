 class Robot {
    private:
    float static const distanceBetweenWheels = 0.19; //currently not used may be used later
    int static const numberOfSamples = 100; //number of samples the line voltage array will hold
    int static const numberOfSensors = 6; //how many sensors you want to use
    float AF; //attenuation factor just a number you want to minimize max speed by, obviously dont go higher than 1
    Wheel* leftWheel; //a pointer to the left wheel
    Wheel* rightWheel;
    lineSensor* sensorArray[numberOfSensors];
    PID2 controller;
    float lineVoltages[numberOfSamples];
    int lvIndex;
    int sensorNumber;
    int brakeC;
    
    Ticker updater;
    Timeout timeToStop;
    
    float endOfLineDetection;
    float Reflec;
    
    float RoboticAngularVelocity;
    
    char state;
    
    public:
    
    Robot(Wheel* LW, Wheel* RW, lineSensor* SA[]) : controller(2.0f, 0.0f, 0.0f, 0.0006f) //this controller actually does nothing atm
    {
        updater.detach();
        timeToStop.detach();
        AF  = 0.7f; //change this value to change max speedzzzzzzzzzzzzzzzzz
        state = 'F';
        Reflec = 0;
          
        lvIndex = 0;
        sensorNumber = 0;
        sensorArray[0] = SA[0];
        sensorArray[1] = SA[1];
        sensorArray[2] = SA[2];
        sensorArray[3] = SA[3];
        sensorArray[4] = SA[4];
        sensorArray[5] = SA[5];
        leftWheel = LW;
        rightWheel= RW;
        
        lineVoltages[lvIndex] = 0.0f;
        
        controller.setInputLimits(-180.0f,180.0f);
        controller.setSetPoint(0.0f);
        //controller will output a value between +- max speed of wheels
        
        controller.assignLimitAddress(rightWheel->returnMinAVptr(),leftWheel->returnMaxAVptr());
    };
    
    float calculateTranslationalVelocity()
    {
        return ((leftWheel->returnAngularVelocity() + rightWheel->returnAngularVelocity())/2.0f);
    }
    
    //difference between angular velocities.
    void dW()
    {
        RoboticAngularVelocity = leftWheel->returnAngularVelocity() - rightWheel->returnAngularVelocity();
    }
    
    float returnRoboticAngularVelocity()
    {
        return RoboticAngularVelocity;   
    }
    
    //attempts to modify the angular velocity of the buggy
    void adjustRbtAngularVelocity(float W)
    {
            //negative is a right turn
            if (W < 0)
            {
                rightWheel->adjustAngularVelocity(rightWheel->returnMaxAngularVel()*AF);
                leftWheel->adjustAngularVelocity((rightWheel->returnMaxAngularVel()+W)*AF);
            }
            else
            {
                leftWheel->adjustAngularVelocity(leftWheel->returnMaxAngularVel()*AF);
                rightWheel->adjustAngularVelocity((leftWheel->returnMaxAngularVel()-W)*AF);
            }   
    }
    
    void robotUpdates(void) //sampling rate the ticker is attached I.E the wheel speed is updated everytime this function is called
        {
            switch (state)
            {
                case 'T':
                    turn180();
                    break;
                case 'S':
                    stopMovement();   
                    break;
                default:
                    followLine();
                    break;
            }
        }
    
    void stopMovement(void)
    {
        leftWheel->adjustAngularVelocity(0.0f);
        rightWheel->adjustAngularVelocity(0.0f);
    }
    
    float returnLineVoltage()
    {
        return lineVoltages[lvIndex%numberOfSamples];
    }
    
    void rbtInit()
    {
    sensorArray[0]->sample();
    sensorArray[0]->calcLineVoltage();
    Reflec = sensorArray[0]->returnLineVoltage();
    updater.attach(callback(this, &Robot::robotUpdates),0.0001f);
    }
    
    void spin()
    {
        rightWheel->adjustAngularVelocity(rightWheel->returnMaxAngularVel()*0.3f);
        leftWheel->adjustAngularVelocity((rightWheel->returnMaxAngularVel())*-0.3f);
    }
    
    void setState(char S)
    {
        state = S;
    }
    
    void followLine(void)
    {
        float ambientLight;
        float reading;
        ambientLight = sensorArray[sensorNumber]->calcLineVoltage();  
        sensorArray[sensorNumber]->sample();
        sensorArray[sensorNumber]->calcLineVoltage();
        reading = (sensorArray[sensorNumber]->returnLineVoltage()-ambientLight);
        switch (sensorNumber)
        {
        case 0:
            lineVoltages[lvIndex%numberOfSamples] = (reading * -260.0f); 
            if (reading <= Reflec*8){endOfLineDetection++;}
            break;
        case 1:
            lineVoltages[lvIndex%numberOfSamples] += (reading * -105.0f);
            if (reading <= Reflec*8) {endOfLineDetection++;}
            break;
        case 2:
            lineVoltages[lvIndex%numberOfSamples] += (reading * -32.5f);
            if (reading <= Reflec*8) {endOfLineDetection++;}
            break;
        case 3:
            lineVoltages[lvIndex%numberOfSamples] += (reading * 32.5f);
            if (reading <= Reflec*8) {endOfLineDetection++;}
            break;
        case 4: 
            lineVoltages[lvIndex%numberOfSamples] += (reading * 75.0f);
            if (reading <= Reflec*8) {endOfLineDetection++;}
            break;
        case 5:
            lineVoltages[lvIndex%numberOfSamples] += (reading * 175.0f);
            if (reading <= Reflec*8) {endOfLineDetection++;}
            break;
        }
                
        sensorNumber ++;
        if (sensorNumber >= numberOfSensors)
        {
            sensorNumber = 0;
            if (endOfLineDetection < 6)
            {
                //AF = scaler(-180.0f,180.0f, 0.2f, 1.0f, lineVoltages[lvIndex%numberOfSamples]);
                AF = 0.7f;
                adjustRbtAngularVelocity(lineVoltages[lvIndex%numberOfSamples]);
                lvIndex++;
            }
            else
            {
               stopMovement(); 
            }
        endOfLineDetection = 0;
        }
    }
    
    void turn180()
    {
        updater.detach();
        rightWheel->adjustAngularVelocity(rightWheel->returnMaxAngularVel()*0.3f);
        leftWheel->adjustAngularVelocity(rightWheel->returnMaxAngularVel()*-0.3f);
        timeToStop.attach(callback(this, &Robot::reAttach),1.1f);
        state = 'S';
    }
    
    void reAttach()
    {
        updater.attach(callback(this, &Robot::robotUpdates),0.0001f);
    }
    
    float scaler(float prevMin, float prevMax, float newMin, float newMax, float var)
    {
        if (var > prevMax) {var = prevMax;}
        if (var < prevMin) {var = prevMin;}
        return (((var-prevMin)/(prevMax - prevMin))*(newMax-newMin))+newMin;
    }

};

/*
Timeout timeToStop,

    
    void travelDistance(float d)//in metres
    {
        timeToStop.attach(callback(this, &Robot::stopMovement), d/(calculateTranslationalVelocity()*(float)PI*Wheel::wheelDiameter));
    }
    
    void robotUpdates(void) //sampling rate the ticker is attached I.E the wheel speed is updated everytime this function is called
        {
            sensorArray[sensorNumber]->sample();
            
            if (sensorNumber < (numberOfSensors/2))
            {
                lineVoltages[(lvIndex%numberOfSamples)] += sensorArray[sensorNumber]->returnLineVoltage()*(sensorNumber-3);
            }
            else
            }
                lineVoltages[(lvIndex%numberOfSamples)] += sensorArray[sensorNumber]->returnLineVoltage()*(sensorNumber-2);
            }
            
            sensorNumber++;
            if (sensorNumber % numberOfSensors == 0)
            {
            sensorNumber = 0;
            controller.setProcessValue(lineVoltages[(lvIndex%numberOfSamples)];
            adjustAngularVelocity(controller.compute());
            lvIndex++;
            }
        }
    
*/
//lineVoltages[lvIndex] += 0.5f;



/*if input value is greater than the maximum value the left wheel can go, go full right TURN
        if (W > leftWheel->returnMaxAngularVel()*AF)
        {
            leftWheel->adjustAngularVelocity(leftWheel->returnMaxAngularVel());
            rightWheel->adjustAngularVelocity(0); 
        }
        else if (W < (-1.0f*rightWheel->returnMaxAngularVel())) //if input value is less than the fastest the right wheel can go in reverse go full left TURN
        {
            rightWheel->adjustAngularVelocity(rightWheel->returnMaxAngularVel());
            leftWheel->adjustAngularVelocity(0); 
        }
        else if (W == 0)
        {
            rightWheel->adjustAngularVelocity(rightWheel->returnMaxAngularVel()*AF);
            leftWheel->adjustAngularVelocity(leftWheel->returnMaxAngularVel()*AF);  
        }
        else
        {

*/