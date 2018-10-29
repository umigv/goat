#include "RoboClaw.h"
#include <math.h>

/* 
 *  HOW TO USE:
 *  1. Set the roboclaw address (probably 0x80 or 0x81)
 *  2. Open the serial console and send m<motor value> to set
 *  the motor speed in encoder pulses per second
 *  3. To set P,I, and D values, send <p/i/d><value>
 *  e.g. p1.4, i4.6, d1.0
 */

// BEGIN CONSTS
//TODO: MAKE SURE THIS ADDRESS MATCHES THAT OF THE CONTROLLER
const uint8_t ROBOCLAW = 0x80; // or 0x81 for the right side
const long ROBOCLAW_BAUD_RATE = 115200;
const long TIMEOUT_VALUE_MS = 10000;

// END CONSTS

// Use uno's Serial1 (same as sabertooth)
RoboClaw roboclaw(&Serial1,TIMEOUT_VALUE_MS);

void setup() {
  //Communicate at 38400bps
  Serial.begin(9600);
  roboclaw.begin(ROBOCLAW_BAUD_RATE);
}
float Kp = 0;
float Ki = 0;
float Kd = 0;
void loop(){
  if(Serial.available()){
    char command = Serial.read();
    if(command == 'p'){
      Kp = Serial.parseFloat();
      setPIDs();
    }else if(command == 'i'){
      Ki = Serial.parseFloat();
      setPIDs();
    }else if(command == 'd'){
      Kd = Serial.parseFloat();
      setPIDs();
    }else if(command == 'm'){
      int32_t s = Serial.parseInt();
      Serial.print("Set motor velocity to: ");
      Serial.println(s);
      roboclaw.SpeedM1(ROBOCLAW,s);
      roboclaw.SpeedM2(ROBOCLAW,s);
    }
  }
  if(millis() % 100 == 0){
    uint8_t stat0,stat1;
    bool valid0,valid1;
    int32_t enc = roboclaw.ReadEncM1(ROBOCLAW,&stat0,&valid0);
    int32_t s = roboclaw.ReadSpeedM1(ROBOCLAW,&stat1,&valid1);
    Serial.println(s);
//    uint16_t conf;
//    roboclaw.GetConfig(ROBOCLAW,conf);
//    Serial.println(conf,HEX);
  }
}
void setPIDs(){
  roboclaw.SetM1VelocityPID(ROBOCLAW,Kp,Ki,Kd,3100);
  roboclaw.SetM2VelocityPID(ROBOCLAW,Kp,Ki,Kd,3100);
  Serial.print("Set PID to: ");
  Serial.print(Kp);
  Serial.print(',');
  Serial.print(Ki);
  Serial.print(',');
  Serial.println(Kd);
}
