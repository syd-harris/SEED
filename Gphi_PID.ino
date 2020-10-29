#include "Arduino.h"
#include <Encoder.h> //https://www.pjrc.com/teensy/td_libs_Encoder.html

// variables for computation
float counter = 0;
bool docounting = false;

// varaibles for serial communication
String InputString = ""; // a string to hold incoming data
bool StringComplete = false;

// PINS
int encoderM1_A = 2; //interrupt pins
int encoderM2_A = 3; //interrupt pins
int D2 = 4;     // disables both outputs of motor channels when LOW; toggling resets latched driver fault condition
int encoderM1_B = 5;
int encoderM2_B = 6;
int M1DIR = 7;  // Motor 1 direction output
int M2DIR = 8;  // Motor 2 direction output
int M1PWM = 9;  // Motor 1 PWM speed output
int M2PWM = 10; // Motor 2 PWM speed output
int SF = 12;    // Status flag indicator (LOW = fault)



// CONSTANTS
const int encoderCountsPerRev = 64 * 50;
const float WHEEL_RADIUS = 0.1476375/2; // m (5.8125 in diameter)
const float WHEEL_DISTANCE = 0.282575; // m (11.125 in)


int speed1 = 0; // Motor 1 speed percentage
int speed2 = 0; // Motor 2 speed percentage
int dir1 = 1;
int dir2 = 1;


// VARIABLES
int posM1raw_intPinOnly = 0;
int posM1raw = 0; //raw encoder counts
int encoderM1_AState = 0;
int encoderM1_BState = 0;
unsigned long encoderM1_ISR_time = 0;
unsigned long encoderM1_ISR_timeLast = 0;
double posM1 = 0; //position in Radians
double velocityM1 = 0; //rads / seconds
double posM1_old = 0;
double posM1Setpoint = 0; //position SP in radians
int posM2raw_intPinOnly = 0;
int posM2raw = 0; //raw encoder counts
int encoderM2_AState = 0;
int encoderM2_BState = 0;
unsigned long encoderM2_ISR_time = 0;
unsigned long encoderM2_ISR_timeLast = 0;
double posM2 = 0; //position in Radians
double velocityM2 = 0; //rads / seconds
double posM2_old = 0;
double posM2Setpoint = 0; //position SP in radians


double velocityRobotLinear = 0; // m/s
double velocityRobotRotational = 0; // rad/s



double errorM1 = 0; //error in radians
double errorM1_old = 0;


//Gphi PID

double velocityRobotLinearSP = 0;
double velocityRobotLinearError = 0;
double velocityRobotLinearError_past = 0;

double velocityRobotRotationalSP = 0;
double velocityRobotRotationalError = 0;
double velocityRobotRotationalError_past = 0;

const int phiPID_Kp = 2;
const int phiPID_Ki = 83;
const int phiPID_Kd = 2000;
double phiPID_D = 0;
double phiPID_I = 0;
double phiPID_e_past = 0;
double phiPID_u = 0;
const int phiPID_umax = 200;



double time_now = 0;
int period = 5; //milliseconds
double timeStart = 4294967296;
bool timeFlag = false;


void setup() {
  // put your setup code here, to run once:
  pinMode(encoderM1_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderM1_A), encoderM1_ISR, CHANGE);
  pinMode(encoderM1_B, INPUT_PULLUP);
  pinMode(encoderM2_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderM2_A), encoderM2_ISR, CHANGE);
  pinMode(encoderM2_B, INPUT_PULLUP);
  pinMode(D2, OUTPUT);
  pinMode(M1DIR, OUTPUT);
  pinMode(M2DIR, OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M2PWM, OUTPUT);
  pinMode(SF, INPUT);
  
  Serial.begin(115200);
  // reserve 200 bytes for the inputString:
  InputString.reserve(200);
  Serial.println("Ready!"); // Let anyone on the other end of the serial line know that Arduino is ready
}

void loop() {
  time_now = millis();
  // Change behavior based on serial input
  if (StringComplete) {
    switch (InputString.charAt(0)) {
      case 'S':
        docounting = true;
        timeFlag = true;
        break;
      case 'E':
        docounting = false;
        break;
    }
    StringComplete = false;
  }

  // Some main code
  if (docounting) {
    if (timeFlag){
          timeStart = millis();
          timeFlag = false;
    }
    counter = millis() - timeStart;
     if (counter > 1000)
     {
      velocityRobotRotationalSP = 2000;
    }
    if (counter >= 2000) {
      Serial.println("Finished");
      velocityRobotRotationalSP = 0;
      docounting = false;

    }
//    if (counter >= 2500){
//      speed1 = 0;
//      speed2 = 0;
//      docounting = false;
//    }
  }
  else {
    counter = 0;
  }

  
     posM1raw = -pos(encoderM1_A, encoderM1_B, posM1raw_intPinOnly, encoderM1_AState);
     posM1 = double(posM1raw)/encoderCountsPerRev*2*3.142;
     posM2raw = pos(encoderM2_A, encoderM2_B, posM2raw_intPinOnly, encoderM2_AState);
     posM2 = double(posM2raw)/encoderCountsPerRev*2*3.142;
     velocityRobotLinear = WHEEL_RADIUS * (velocityM1 + velocityM2) / 2;
     velocityRobotRotational = -WHEEL_RADIUS * (velocityM1 - velocityM2) / WHEEL_DISTANCE;

     velocityRobotLinearError = velocityRobotLinearSP - velocityRobotLinear;
     
     velocityRobotRotationalError = velocityRobotRotationalSP - velocityRobotRotational;
     phiPID_D = (velocityRobotRotationalError - velocityRobotRotationalError_past)/period;
     velocityRobotRotationalError_past = velocityRobotRotationalError;
     phiPID_I = phiPID_I + (period * velocityRobotRotationalError);
     phiPID_u = phiPID_Kp*velocityRobotRotational + phiPID_Ki*phiPID_I + phiPID_Kd*phiPID_D;
     if abs(phiPID_u > phiPID_umax){
      int sign_u = 0;
      if (phiPID_u > 0) sign_u = 1;
      else sign_u = -1;
      phiPID_u = sign_u * phiPID_umax;
      int sign_e = 0;
      if (velocityRobotRotationalError > 0) sign_e = 1;
      else sign_e = -1;
      velocityRobotRotationalError = sign_e * min(phiPID_umax/phiPID_Kp, abs(velocityRobotRotationalError));
      phiPID_I = (phiPID_u - phiPID_Kp*velocityRobotRotationalError - phiPID_Kd*phiPID_D) / phiPID_Ki;
     }

     if (abs(velocityRobotRotationalSP) < 0.0001){
      speed1 = 0;
      speed2 = 0;
     }
     else{
        speed1 = abs(phiPID_u /2);
        speed2 = abs(phiPID_u /2);
     }

     if (phiPID_u > 0) {
      dir1 = 0;
      dir2 = 1;
     }
     else{
      dir1 = 1;
      dir2 = 0;
     }
     
  digitalWrite(M1DIR, !dir1);
  digitalWrite(M2DIR, dir2);
  digitalWrite(D2, HIGH);
  analogWrite(M1PWM, int(speed1*2.55));
  analogWrite(M2PWM, int(speed2*2.55));
     
//

  Serial.print(counter);
  Serial.print("\t");
  Serial.print(velocityRobotRotational);
  Serial.println("");
//
//Serial.print(angularVelocityM1);
//Serial.print("\n");
  
  // wait 100 ms
  //delay(100);

  while(millis() < time_now + period);  // delay for length of period
}


/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    InputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      StringComplete = true;
    }
  }
}

void encoderM1_ISR(){
  //on pin A state change, encoder has advanced or returned 2 positions
    encoderM1_ISR_time = micros();
    encoderM1_AState = digitalRead(encoderM1_A);
    encoderM1_BState = digitalRead(encoderM1_B);
    if (encoderM1_AState != encoderM1_BState){
        posM1raw_intPinOnly += 2;
        velocityM1 = float(2)/encoderCountsPerRev*2*3.142/((encoderM1_ISR_time - encoderM1_ISR_timeLast)/float(1000000));
      }
    else{
      posM1raw_intPinOnly -= 2;
      velocityM1 = -float(2)/encoderCountsPerRev*2*3.142/((encoderM1_ISR_time - encoderM1_ISR_timeLast)/float(1000000));
    }
    encoderM1_ISR_timeLast = encoderM1_ISR_time;
}

void encoderM2_ISR(){
  //on pin A state change, encoder has advanced or returned 2 positions
    encoderM2_ISR_time = micros();
    encoderM2_AState = digitalRead(encoderM2_A);
    encoderM2_BState = digitalRead(encoderM2_B);
    if (encoderM2_AState != encoderM2_BState){
        posM2raw_intPinOnly += 2;
        velocityM2 = -float(2)/encoderCountsPerRev*2*3.142/((encoderM2_ISR_time - encoderM2_ISR_timeLast)/float(1000000));
      }
    else{
      posM2raw_intPinOnly -= 2;
      velocityM2 = float(2)/encoderCountsPerRev*2*3.142/((encoderM2_ISR_time - encoderM2_ISR_timeLast)/float(1000000));
    }
    encoderM2_ISR_timeLast = encoderM2_ISR_time;
}

int pos(int pinA, int pinB, int count, int AState){
  //adjust actual position by -1, 0, +1 depending on current values of pins
  int AStateCur = digitalRead(pinA);
  int BStateCur = digitalRead(pinB);
  if (AStateCur == BStateCur){ // if current AState = BState, the encoder is an indented position.  No need to adjust
     return count;
  }
  else if (AState == 1){
    if (AStateCur == 0){
     return count+1;
    }
    else{
      return count-1;
    }
  }
  else if (AState == 0){
     if (AStateCur == 1){
     return count+1;
    }
    else{
      return count-1;
    }
  }
}
