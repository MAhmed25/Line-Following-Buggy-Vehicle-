#define PI 3.141

#include "math.h"
#include "mbed.h"
#include "Encoder.h"
#include "lineSensor.h"
#include "PID2.h"
#include "PID.h"
#include "Wheel.h"
#include "Robot.h"


//blue D8
//Green D9
//Red D5
int main() {
    Serial hm10(PA_11, PA_12);
    hm10.baud(9600);
    char prevC;
    
    DigitalIn mybutton(USER_BUTTON);
    
    DigitalOut enable(PA_13);
    enable.write(1);
    
    Encoder* RE = new Encoder(PB_3,PB_5);
    Encoder* LE = new Encoder(PB_10,PB_4);
    Wheel* leftWheel = new Wheel(LE,PC_8,PA_9, PA_14);
    Wheel* rightWheel = new Wheel(RE,PC_6,PA_8, PA_7);
    
    //an array of lineSensor pointers params: lineSensor(PinName emitter Pin, PinName reciever Pin, make sure it is from LEFT TO RIGHT
    lineSensor* sensorArray[6] = {new lineSensor(PB_9,A5),new lineSensor(PC_11,A4),new lineSensor(PD_2,A3),new lineSensor(PC_10,A2),new lineSensor(PB_8,A1),new lineSensor(PC_12,A0)};
    
    Robot rbt(leftWheel, rightWheel, sensorArray);
    
    leftWheel->init2();
    
    rightWheel->init2();
    
    rbt.rbtInit();
    prevC = 'C';
   
    while(1)
    {
       if(hm10.readable())
        {
            rbt.setState(hm10.getc());
        } 
    }
}

/*

        */
