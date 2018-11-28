#include "Arduino.h"
#include "Hexapod.h"
#include <Wire.h>

// Initialize pin modes and servo shield
void Hexapod::init() {
  pwm1.begin();
  pwm2.begin();
  pinMode(relay, OUTPUT);
  digitalWrite(relay, HIGH);
  pwm1.setPWMFreq(60);
  pwm2.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}

// Move legs into the next configuration of a foward walking gait
void Hexapod::walk(float forward, float turn, int counter) {
  for(int leg=1; leg<7; leg++) {
    if(leg%2==0) { // right side
      moveLegToState(leg, counter%4, forward, turn);
    } else {
      moveLegToState(leg, (counter+2)%4, forward, turn);
    }
  }
}

// Lower the hexapod to the ground
void Hexapod::sit() {
  for(int leg=1; leg<7; leg++) {
    float dR = 5.3;
    float z0 = -0.6;
    float pos[3] = {(R+dR)*cos(leg*M_PI/3-M_PI/6),(R+dR)*sin(leg*M_PI/3-M_PI/6),z0};
    moveLegToPosition(pos[0], pos[1], pos[2], leg);
  }    
}

// Stand with all 6 legs on the ground
void Hexapod::stand() {
  for(int leg=1; leg<7; leg++) {
    moveLegToState(leg, 0, 0, 0);
  }
}

// Move a leg to a predefined state of the gait
void Hexapod::moveLegToState(int leg, int state, float forward, float turn) {
  float pos[3];
  getLegPosition(leg, state, forward, turn, pos);
  moveLegToPosition(pos[0], pos[1], pos[2], leg);
}

// Generate a position vector for a given state of a leg
// Legs numbered 1-6 CCW from front left
// States numbered 1-4 back-up-forward-down
// forward and turn specify movement direction between -1 and 1
// Resulting position is stored in output
void Hexapod::getLegPosition(int leg, int state, float forward, float turn, float* output) {
  float pos[] = {R*cos(leg*M_PI/3-M_PI/6),R*sin(leg*M_PI/3-M_PI/6),ground};
  if(state == 2) { // leg lifted
    pos[2] += clearance;
  }
  if (leg <=3) { // left side
    pos[1] += yoffset;
  } else {
    pos[1] -= yoffset;
  }
  float w = 0;
  if (state == 1) { // leg back
    pos[0] -= dx*forward;
    w = -turn*dtheta;
  }
  else if (state == 3) { // leg forward
    pos[0] += dx*forward;
    w = turn*dtheta;
  }
  // turn
  float x = pos[0]*cos(w) - pos[1]*sin(w);
  float y = pos[0]*sin(w) + pos[1]*cos(w);
  pos[0] = x;
  pos[1] = y;
  for(int i=0; i<3; i++) {
    output[i] = pos[i];
  }
}

// Move all 3 servos of a leg to position the end effector
void Hexapod::moveLegToPosition(float x, float y, float z, int leg) {
  int angles[3];
  getAngles(x, y, z, leg, angles);
  if(angles[0] != -1) {
    moveLeg(angles, leg);
  }
}

// Determine servo angles (deg) from position relative to hexapod center (in)
// x: forward, y: left, z: up, leg: CCW starting with front left leg
void Hexapod::getAngles(float x, float y, float z, int leg, int* angles) {
  float a = 0;
  float b = 0;
  float c = 0;
  x = x - R*cos(leg*M_PI/3 - M_PI/6);
  y = y - R*sin(leg*M_PI/3 - M_PI/6);
  a = atan2(y, x) + M_PI/6 - leg*M_PI/3;
  float r = sqrt(pow(x,2)+pow(y,2)) - L1;
  if (a < -M_PI) {
      a = a + 2*M_PI;
  }
  if (a < -M_PI/2) {
      a = a + M_PI;
      r = -sqrt(pow(x,2)+pow(y,2)) - L1;
  }
  if (a > M_PI/2) {
      a = a - M_PI;
      r = -sqrt(pow(x,2)+pow(y,2)) - L1;
  }
  if (r > L2 + L3 || pow(r,2)+pow(z,2) == 0) {
      Serial.println("Requested configuration out of reach");
      angles[0] = angles[1] = angles[2] = -1;
      return;
  }
  float r1 = (pow(r,3)+r*pow(z,2)+r*pow(L2,2)-r*pow(L3,2)-z*sqrt(-pow(r,4)-pow(z,4)-pow((pow(L2,2)-pow(L3,2)),2)+2*pow(z,2)*
      (pow(L2,2)+pow(L3,2))+2*pow(r,2)*(-pow(z,2)+pow(L2,2)+pow(L3,2))))/(2*(pow(r,2)+pow(z,2)));
  float z1 = (pow(z,3)+z*pow(r,2)+z*pow(L2,2)-z*pow(L3,2)+r*sqrt(-pow(r,4)-pow(z,4)-pow((pow(L2,2)-pow(L3,2)),2)+2*pow(z,2)*
      (pow(L2,2)+pow(L3,2))+2*pow(r,2)*(-pow(z,2)+pow(L2,2)+pow(L3,2))))/(2*(pow(r,2)+pow(z,2)));
  if (isnan(r1)|| isnan(z1)) {
      Serial.println("Requested configuration has no solution");
      angles[0] = angles[1] = angles[2] = -1;
      return;
  }
  b = -atan2(z1, r1);
  c = -atan2(z-z1, r-r1) - b;
  angles[0] = round(180/M_PI*a + 90);
  angles[1] = round(-180/M_PI*b + 90);
  angles[2] = round(180/M_PI*c);
  for(int i=0; i<3; i++) {
    if (angles[i] > maxLimits[i] || angles[i] < minLimits[i]) {
      Serial.print("Requested configuration outside servo range: Servo ");
      Serial.println(i);
      angles[0] = angles[1] = angles[2] = -1;
      return;
    }
  }
}

// Move all 3 servos of a leg to a given state
void Hexapod::moveLeg(int *angles, int leg) {
  if(VERBOSE) {
    Serial.print("Leg ");
    Serial.print(leg);
    Serial.print(" moving to ");
    Serial.print(angles[0]);
    Serial.print(", ");
    Serial.print(angles[1]);
    Serial.print(", ");
    Serial.println(angles[2]);
  }
  for(int i=0; i<3; i++) {
    moveServo(angles[i], leg, i);
  }
}

// Move a specified servo to a given angle in degrees
void Hexapod::moveServo(int value, int leg, int servo) {
  if(value >= minLimits[servo] && value <= maxLimits[servo]) {
    int pulse = pulseLength(value - offsets[leg-1][servo], leg, servo);
    if(servos[leg-1][servo] < 16) {
      pwm1.setPWM(servos[leg-1][servo], 0, pulse);
    } else {
      pwm2.setPWM(servos[leg-1][servo]-16, 0, pulse);
    }
  } else if(value!=-1) {
    Serial.print("Servo ");
    Serial.print(labels[servo]);
    Serial.print(" out of bounds: please enter a value between ");
    Serial.print(minLimits[servo]);
    Serial.print(" and ");
    Serial.println(maxLimits[servo]);
  }
}

// Convert angle in degrees to PWM pulse length
int Hexapod::pulseLength(int angle, int leg, int servo) {
  int len = map(angle, ANGLEMIN, ANGLEMAX, SERVOMIN, SERVOMAX);
  if(leg > 3 && servo > 0) {
    len = map(180-angle, ANGLEMIN, ANGLEMAX, SERVOMIN, SERVOMAX);
  }
  return(len);
}
