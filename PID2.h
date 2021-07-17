class PID2
{
    private:
    float Kp_, Ki_, Kd_, Ts, PrevErr, PrevPrevErr, prevControlAction, setPoint;
    
    float inMin_;
    float inMax_;
    
    float* outMin_;
    float* outMax_;
    
    public:
    PID2 (float Kp, float Ki, float Kd, float sampleTime)
    {
        PrevErr = 0;
        PrevPrevErr = 0;
        prevControlAction = 0;
        setPoint = 0;
        
        inMin_ = -1.0f;
        inMax_ = 1.0f;
        
        Ts = sampleTime;
        Kp_ = Kp;
        Ki_ = Ki;
        Kd_ = Kd;
    }
    
    float compute (float currVal)
    {
    float currErr = currVal - setPoint;
    float controlAction;
    
    if (currVal > inMax_) {inMax_ = currVal;}
    if (currVal < inMin_) {inMin_ = currVal;}
    
    controlAction = prevControlAction - (PrevErr*Kp_) + (Kp_*currErr)+ (Ki_*Ts*currErr) + ((Kd_/Ts)*(currErr - PrevErr - PrevErr + PrevPrevErr));
   
    prevControlAction = controlAction;
    PrevPrevErr = PrevErr;
    PrevErr = currErr;
    //scale the control Action to the correct output limits and output
    controlAction = ((((controlAction)- inMin_)/(inMax_ - inMin_)) * (*outMax_ - *outMin_)) + *outMin_;
    
    if (controlAction > *outMax_) {return *outMax_;} else if (controlAction < *outMin_) {return *outMin_;} else {return controlAction;}
    
    }
    
    void setSetPoint(float _sP)
    {
        setPoint = _sP;
    }
    
    void setInputLimits(float inMin,float inMax)
    {
        if (inMin > inMax) {return;}
        
        PrevErr = scaler(inMax_,inMin_,inMin,inMax,PrevErr);
        PrevPrevErr = scaler(inMax_,inMin_,inMin,inMax,PrevPrevErr);
        prevControlAction = scaler(inMax_,inMin_,inMin,inMax,prevControlAction);
        
        inMin_ = inMin;
        inMax_ = inMax;
    }
    
    void assignLimitAddress(float *outMax, float *outMin)
    {
        outMin_ = outMin; 
        outMax_ = outMax;
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
    
    float returnOutMax()
    {
        return *outMax_;
    }
    
};

//(((temp-inMin)/(inMax - inMin))*(outMax-outMin))+outMin