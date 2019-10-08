/*





*/

#include <Servo.h>
#include <Stepper.h> 



typedef enum {
    Acquire = 0, ErrorCalc, PID, Actuators
} estados;

estados state  = Acquire;



#define LDR11 A0
#define LDR12 A1
#define LDR21 A2
#define LDR22 A3
#define SERVO 6



//Variables
const int stepsPerRevolution = 500; 
int kP = 4, kI = 0, kD = 1.5;
int P0 = 0, I0 = 0 , D0 = 0;
int P1 = 0, I1 = 0 , D1 = 0;
int Value11 = 0, Value12 = 0, Value21 = 0, Value22 = 0;
int Error0 = 0, Error1 = 0, LastError0 = 0, LastError1 = 0;
int PID0 = 0, PID1 = 0;
int pos = 90, val = 90;



Stepper stepper(stepsPerRevolution, 8,10,9,11); 
Servo servo;



void setup(){
    Serial.begin(9600);

    analogRead(LDR11);
    analogRead(LDR12);
    analogRead(LDR21);
    analogRead(LDR22);

    servo.attach(SERVO);
    servo.write(90);

    stepper.setSpeed(30);
}



void threadServo(){
                       
            pos = pos - PID0;
            servo.write(pos);
            //Serial.println(pos);

}


void threadMotor(){

    stepper.step(-PID1); 

}


void machine(){

    switch (state) {
        case Acquire:
            //Sensor Read
            Value11 = analogRead(LDR11);
            Value12 = analogRead(LDR12);
            Value21 = analogRead(LDR21);
            Value22 = analogRead(LDR22);
            
            state = ErrorCalc;
            break;


        case ErrorCalc:
            //Error calculation
            Error0 = ((Value11 + Value12) - (Value21 + Value22))/100;
            Error1 = ((Value11 + Value21) - (Value12 + Value22))/100;
            
            state = PID;
            break;


        case PID:
            //PID implementation
            P0 = Error0 * kP;
            P1 = Error1 * kP;

            I0 += Error0 * kI;
            I1 += Error1 * kI;

        
            D0 = (Error0 - LastError0) * kD;
            D1 = (Error1 - LastError1) * kD;

            PID0 = (P0 + I0 + D0)/2;
            PID1 = (P1 + I1 + D1)*12/3;

            LastError0 = Error0;
            LastError1 = Error1;

            state = Actuators;
            break;


        case Actuators:
            //Actuators
            threadMotor();
            threadServo();
            
            
            //Debugger
                      
            Serial.println("PID Value Begin");
            Serial.print(PID0);
              Serial.print(" , ");
            Serial.println(PID1);
            Serial.println("PID Value END");
            Serial.println();
            Serial.println();

            delay(100);

            state = Acquire;
            break;
      }
}



void loop(){
    machine();    
}
