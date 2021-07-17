class lineSensor
{
    private:
    
    DigitalOut emitter;
    AnalogIn receiver;
    float static const vREF = 3.3f;
    float lineVoltage;
    Timeout sampler;
    
    public:
    float static const sampleTime = 0.00004f;
    
    lineSensor(PinName E, PinName R):emitter(E), receiver(R)
    {   
        sampler.detach();
    } 
    
    //Turn on the emitter i.e emit a light
    void turnOn(void)
    {
        emitter.write(1);   
    }
    
    //turn off the emitter i.e don't emit a light
    void turnOff(void)
    {
        emitter.write(0);
    }
    
    //toggle the emitter i.e if its on turn it off, if its off turn it on
    void toggleEmitter(void)
    {
        emitter.write(!emitter.read());
    }
    
    //return an int representing the state of the emitter i.e is it on or off where 1 represents on and 0 represents off
    int returnEmitterState(void)
    {
        return emitter.read();
    }
    
    void sample(void)
    {
        turnOn();
        lineVoltage = receiver.read()*vREF;
        sampler.attach(callback(this, &lineSensor::turnOff),sampleTime);
    }
    
    float calcLineVoltage(void)
    {
        lineVoltage = receiver.read()*vREF;
        return lineVoltage;
    }
    
    float returnLineVoltage(void)
    {
     return lineVoltage;
    }
    
};