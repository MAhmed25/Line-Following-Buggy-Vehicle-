//works only in x2 encoding (512 counts per second)
#include "QEI.h"
class Encoder: public QEI {  
    private:
    Timer dT; //to calculate rate of change of encoder ticks.
    int prevPulses;
    
    public:
    
    Encoder(PinName A, PinName B) : QEI(A,B,NC,256,X2_ENCODING)
    {
        prevPulses = 0;
        dT.stop();
        
    };
        
    float encoderTickRate() {
        int PP = prevPulses;
        int CP = QEI::getPulses();
        float deltaT = dT.read();
        prevPulses = CP;
        dT.reset();
        return ((float)(CP-PP)/deltaT);
        };
        
    void startTimer(void)
    {
     dT.start();   
    }

};
