#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver();
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

// Gait parameters
float ground = -6; // Height of ground relative to body (in)
float clearance = 2; // Height of raised leg relative to ground (in)
float dx = 1.5; // Half of the linear step distance (in)
float dtheta = M_PI/12; // Half of the angular step angle (rad)
float yoffset = 3; // Horizontal distance of feet from body (in)
int stepDuration = 500; // Time duration of a step in (ms)

// Pins
int relay = 22;
int servos[6][3] = {{0, 1, 2},
                    {3, 4, 5},
                    {6, 7, 8},
                    {16, 17, 18},
                    {19, 20, 21},
                    {22, 23, 24}}; // [leg 1-6][servo A-C]

// Servo properties
String labels[] = {"A", "B", "C"};
const int N = 3; // number of servos
const int minLimits[] = {0, 40, 0};
const int maxLimits[] = {180, 180, 120};
const int offsets[6][3] = {{0, -3, -1},
                           {10, 20, 15},
                           {10, 5, 8},
                           {5, 3, 2},
                           {11, 0, 0},
                           {5, 2, 4}}; // (actual - desired) servo angle

// Servo PWM to angle conversion factors
const int ANGLEMIN = 0; // minimum servo angle in degrees
const int ANGLEMAX = 180; // maximum servo angle in degrees
const int SERVOMIN = 140; // minimum pwm pulse length count (out of 4096)
const int SERVOMAX = 520; // maximum pwm pulse length count (out of 4096)

// Frame geometry
const float R = 5.35; // Distance from center to Servo A (in)
const float L1 = 1.12; // Distance from Servo A to Servo B (in)
const float L2 = 2.24; // Distance from Servo B to Servo C (in)
const float L3 = 4.84; // Distance from Servo C to end effector (in)

enum State {NONE, STAND, SIT, WALK};

// State variables
long stepStartTime = 0;
int counter = 0;
State state = NONE;
float forward = 0;
float turn = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Starting up! Please enter desired linear and angular velocities.");
  pwm1.begin();
  pwm2.begin();
  pinMode(relay, OUTPUT);
  digitalWrite(relay, HIGH);
  pwm1.setPWMFreq(60);
  pwm2.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  delay(10);
}

void loop() {
  // Operator input
  if(Serial.available() > 0) {
    if(Serial.peek() == ' ') { // Stop
      if(state==STAND) {
        state = SIT;
        Serial.println("Sit");
      } else {
        state = STAND;
        Serial.println("Stand");
      }
      counter = 0;
      Serial.read();
    } else {
      forward = Serial.parseFloat();
      turn = Serial.parseFloat();
      if(abs(forward)+abs(turn) <= 1) { // Walk
        Serial.print("Entering walk mode: ");
        Serial.print(forward);
        Serial.print(", ");
        Serial.println(turn);
        state = WALK;
      } else { // Invalid input
        Serial.println("Invalid velocity");
      }
    }
  }
  
  // Act
  if(state==WALK && (millis() - stepStartTime > stepDuration)) {
    stepStartTime = millis();
    walk(forward, turn, counter);
    counter++;
  } else if(state==STAND) {
    stand();
  } else if(state==SIT) {
    sit();
  }
}

// Move legs into the next configuration of a foward walking gait
void walk(float forward, float turn, int counter) {
  for(int leg=1; leg<7; leg++) {
    if(leg%2==0) { // right side
      moveLegToState(leg, counter%4, forward, turn);
    } else {
      moveLegToState(leg, (counter+2)%4, forward, turn);
    }
  }
}

// Lower the hexapod to the ground
void sit() {
  for(int leg=1; leg<7; leg++) {
    float dR = 5.3;
    float z0 = -0.6;
    float pos[3] = {(R+dR)*cos(leg*M_PI/3-M_PI/6),(R+dR)*sin(leg*M_PI/3-M_PI/6),z0};
    moveLegToPosition(pos[0], pos[1], pos[2], leg);
  }    
}

// Stand with all 6 legs on the ground
void stand() {
  for(int leg=1; leg<7; leg++) {
    moveLegToState(leg, 0, 0, 0);
  }
}

// Move a leg to a predefined state of the gait
void moveLegToState(int leg, int state, float forward, float turn) {
  float pos[3];
  getLegPosition(leg, state, forward, turn, pos);
  moveLegToPosition(pos[0], pos[1], pos[2], leg);
}

// Generate a position vector for a given state of a leg
// Legs numbered 1-6 CCW from front left
// States numbered 1-4 back-up-forward-down
// forward and turn specify movement direction between -1 and 1
// Resulting position is stored in output
void getLegPosition(int leg, int state, float forward, float turn, float* output) {
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
void moveLegToPosition(float x, float y, float z, int leg) {
  int angles[3];
  getAngles(x, y, z, leg, angles);
  if(angles[0] != -1) {
    moveLeg(angles, leg);
  }
}

// Determine servo angles (deg) from position relative to hexapod center (in)
// x: forward, y: left, z: up, leg: CCW starting with front left leg
void getAngles(float x, float y, float z, int leg, int* angles) {
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
      Serial.print(i);
      Serial.print(", Value ");
      Serial.println(angles[i]);
      angles[0] = angles[1] = angles[2] = -1;
      return;
    }
  }
}

// Move all 3 servos of a leg to a given state
void moveLeg(int *angles, int leg) {
  Serial.print("Leg ");
  Serial.print(leg);
  Serial.print(" moving to ");
  Serial.print(angles[0]);
  Serial.print(", ");
  Serial.print(angles[1]);
  Serial.print(", ");
  Serial.println(angles[2]);
  for(int i=0; i<3; i++) {
    moveServo(angles[i], leg, i);
  }
}

// Move a specified servo to a given angle in degrees
void moveServo(int value, int leg, int servo) {
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
int pulseLength(int angle, int leg, int servo) {
  int len = map(angle, ANGLEMIN, ANGLEMAX, SERVOMIN, SERVOMAX);
  if(leg > 3 && servo > 0) {
    len = map(180-angle, ANGLEMIN, ANGLEMAX, SERVOMIN, SERVOMAX);
  }
  return(len);
}
