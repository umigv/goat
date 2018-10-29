#include "RoboClaw.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#define statusLight 2

// BEGIN CONSTS
//TODO: MAKE SURE THIS ADDRESS MATCHES THAT OF THE CONTROLLER
const uint8_t ROBOCLAW0 = 0x80;
const uint8_t ROBOCLAW1 = 0x81;
const bool REVERSE0 = false;
const bool REVERSE1 = true;
const long ROBOCLAW_BAUD_RATE = 38400;
const long ROBOCLAW_TIMEOUT = 10000; 
const long CONTROL_TIMEOUT = 1000; //ms to wait  before killing motors
const float PPRREV = 1440; // encoder pulses per rotation
const float HALFTRACK = 0.3048; // Robot thiccness/2 in METERS
const float DIAMETER = 0.3302; // in METERS
const float REVPM = 1.0 / (M_PI * DIAMETER); // revolutions per meter
const float PPM = PPRREV * REVPM; // encoder pulses per meter
// END CONSTS

int mode;
unsigned long statusTic;
int ledState = LOW;

// Use uno's Serial1 (same as sabertooth)
RoboClaw roboclaw(&Serial,ROBOCLAW_TIMEOUT);
unsigned long lastData = 0;

void cmdVelCallback(const geometry_msgs::Twist&);
ros::NodeHandle handle;
ros::Subscriber<geometry_msgs::Twist> subscriber("cmd_vel", &cmdVelCallback);

void cmdVelCallback(const geometry_msgs::Twist &twist) {
  // calculations are done with the following assumptions:
  // lin. vel (linear) is in m/s
  // ang. vel (spin) is in rad/s
  // ROBOT_RADIUS is in meters and is 1/2 the distance between the 2 wheels
  lastData = millis();
  const float linear = twist.linear.x;
  const float spin = twist.angular.z;
  // t is track
  // [ 1/2  1/2 ][vl] = [vx]
  // [-1/2t 1/2t][wz] = [wz]
  // by inversion...
  // [1 -t][vx] = [vl]
  // [1  t][wz] = [vr]
  float vLeft = linear - spin * HALFTRACK*2;
  float vRight = linear + spin * HALFTRACK*2;
  vLeft = REVERSE0 ? -vLeft : vLeft;
  vRight = REVERSE1 ? -vRight : vRight;
  
  roboclaw.SpeedM1M2(ROBOCLAW0, vLeft * PPM, vLeft * PPM);
  roboclaw.SpeedM1M2(ROBOCLAW1, vRight * PPM, vRight * PPM);
}

void setup() {
  //Communicate at 38400bps
  roboclaw.begin(ROBOCLAW_BAUD_RATE);
  handle.initNode();
  handle.subscribe(subscriber);

  statusTic = millis();
  pinMode(statusLight, OUTPUT);
}

void setMode(int val) {
  mode = val;
}

void loop(){
  handle.spinOnce();
  if(millis() - lastData >= CONTROL_TIMEOUT){
    roboclaw.SpeedM1M2(ROBOCLAW0, 0, 0);
    roboclaw.SpeedM1M2(ROBOCLAW1, 0, 0);
  }

  setMode(digitalRead(3));//temporary
  if(mode == 0)
  {
    digitalWrite(statusLight, HIGH);
  }
  else
  {
    if( millis()-statusTic > 250 )
    {
      ledState = !ledState;
      digitalWrite(statusLight, ledState);
      statusTic = millis();
    }
  }
}

