class Wheel 
{
    private:
    
    float distance; //distance traversed by wheel
    float angularVelocity;
    int br;
    
        float const static gain = 1.1f; //closed loop gain, (amount to amplify the difference) you have to tune this value
    //but make sure its less than 1.5 otherwise you'll have a really sensitive motor
    
    PwmOut Mtr; //connect this pin to the motor driveboard pwm
    DigitalOut direction; //connected to the direction pin of motor drive board
    DigitalOut polarity; //connected to the bipolar of motor drive board. 0 = unipolar 1 = bipolar
    
    Ticker updater;

    Encoder* enc;
    
    PID controller;
    
    public:
    
    float  maxAngularVel;
    float  minAngularVel; 
    
    float static const wheelDiameter = 0.18; //used in calculation of Linear velocity i.e never
    
    Wheel (Encoder* E, PinName M, PinName D, PinName Mode) : Mtr(M), direction(D), polarity(Mode), controller(gain,4.0f,0.0f,0.0003f)
        {
        maxAngularVel = 0.0f;
        enc = E;
        polarity = 0;   
        direction = 0;
        distance = 0;
        Mtr.period_us(100); //frequency of 5KHz determine this constant value based on switching losses+frequency losses
        //higher freq -> more switching losses lower freq -> more "capacitive losses" need to find a balance
        Mtr.write(1); //start off on the turned off state
        
        updater.detach();
        
        float *maxAVptr = &maxAngularVel;
        float *minAVPtr = &minAngularVel;
        controller.assignLimitAddress(maxAVptr,minAVPtr);
        }
    
    void calculateAngularVelocity() //returns a float value which is the angular velocity of the WHEEL
    {
        float eTR = enc->encoderTickRate();
        angularVelocity = (eTR/256.0f)*2.0f*(float)PI;
    }
    
    void setFrequency(int freq) //if you want to adjust the frequency
    {
        Mtr.period(1/freq);
    }
    
    float returnAngularVelocity() //as it says
    {
        return angularVelocity;
    }
    
    //only called once during initialization to calculate max angular velocity 
    //dir = direction, do opposite for each wheel just so your buggy doesn't move FORWARD but rather rotates
    void init(int dir) 
    {
      enc->startTimer();
      Mtr.write(0); //max speed
      angularVelocity = 10.0f;
      if (dir == 0) {direction = 0;} else {direction = 1;}
      updater.attach(callback(this, &Wheel::init2),0.6f);
    }
    
    void init2(void) //used as a temporarily wait command for the wheel to spin to max
    {
        enc->startTimer();
        calculateAngularVelocity();
        maxAngularVel = 75.0f;
        minAngularVel = -1.0f*maxAngularVel;
        controller.setSetPoint(0.0f);
        updater.attach(callback(this, &Wheel::wheelUpdates),0.0003f);
    }
    
    void startController()
    {
       updater.attach(callback(this, &Wheel::wheelUpdates),0.01f); 
    }
    
    void stopController()
    {
        updater.detach();
    }
    
    void wheelUpdates(void) //sampling rate the ticker is attached I.E the wheel speed is updated everytime this function is called
        {
            calculateAngularVelocity();
            //distance += angularVelocity*(wheelDiameter/2)*0.0005f;
            float temp2 = controller.compute(angularVelocity); //another temporary value to store the computed angular velocity
            if (temp2 < 0) {direction = 0;} else {direction = 1;} //change direction according to the computed value
            Mtr.write((1.0f - abs(temp2))); //write the value as a pwm
        }
        
    void adjustAngularVelocity(float W) // W = angular velocity you want, obviously putting a |w| value that is > max angular velocity will set dutcy cycle to max
        {
        controller.setSetPoint(W);
        };
        
    float getDistance(void)
    {
    return distance; //distance traversed by wheel
    }
    
    void stop()
    {
        controller.setSetPoint(0);
    }
    
    float returnMaxAngularVel(void)
    {
        return  maxAngularVel;
    }
    
    float* returnMaxAVptr()
    {
        float *tempPTR = &maxAngularVel;
        return tempPTR;
    }
    
    float* returnMinAVptr()
    {
        float *tempPTR = &minAngularVel;
        return tempPTR;
    }
    
    void setBR (int p)
    {
        br = p;
    }
    
    void brake()
    {
        
    }
    
};