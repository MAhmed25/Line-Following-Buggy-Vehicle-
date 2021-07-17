class PID
{
    private:
    float Kp_, Ki_, Kd_, Ts, PrevErr, PrevPrevErr, prevControlAction, setPoint;
    
    float* inMin_;
    float* inMax_;
    
    float outMin_;
    float outMax_;
    
    public:
    PID (float Kp, float Ki, float Kd, float sampleTime)
    {
        PrevErr = 0;
        PrevPrevErr = 0;
        prevControlAction = 0;
        setPoint = 0;
        
        outMin_ = -1.0f;
        outMax_ = 1.0f;
        
        Ts = sampleTime;
        Kp_ = Kp;
        Ki_ = Ki;
        Kd_ = Kd;
    }
    
    float compute (float currVal)
    {
    float currErr = setPoint - currVal;
    float controlAction;
    
    //if (currVal > *inMax_) {*inMax_ = currVal; *inMin_ = -1.0f*currVal;}
    //if (currVal < *inMin_) {*inMin_ = currVal; *inMax_ = -1.0f*currVal;}

    controlAction = prevControlAction - (PrevErr*Kp_) + (Kp_*currErr)+ (Ki_*Ts*currErr) + ((Kd_/Ts)*(currErr - PrevErr - PrevErr + PrevPrevErr));
   
    prevControlAction = controlAction;
    PrevPrevErr = PrevErr;
    PrevErr = currErr;
    //scale the control Action to the correct output limits and output
    controlAction = ((((controlAction)- *inMin_)/(*inMax_ - *inMin_)) * (outMax_ - outMin_)) + outMin_;

    if (controlAction > outMax_) {return outMax_;} else if (controlAction < outMin_) {return outMin_;} else {return controlAction;}
    
    }
    
    void setSetPoint(float _sP)
    {
        setPoint = _sP;
    }
    
    void setOutputLimits(float outMin,float outMax)
    {
        if (outMin > outMax) {return;}
        
        outMin_ = outMin;
        outMax_ = outMax;
    }
    
    void assignLimitAddress(float *inMax, float *inMin)
    {
        inMax_ = inMax; 
        inMin_ = inMin;
    }
    
    float returnInMax() 
    {
        return  *inMax_;
    }
    
    float scaler(float prevMin, float prevMax, float newMin, float newMax, float var)
    {
        if (var > prevMax) {var = prevMax;}
        if (var < prevMin) {var = prevMin;}
        return (((var-prevMin)/(prevMax - prevMin))*(newMax-newMin))+newMin;
    }
    
    float returnPrevCA()
    {
        return   prevControlAction;
    }
    
};

//(((temp-inMin)/(inMax - inMin))*(outMax-outMin))+outMin