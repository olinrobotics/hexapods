#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Gait parameters
int ground = -5; // Height of ground relative to body (in)
int clearance = 2; // Height of raised leg relative to body (in)
int dx = 2.5; // Half of the total step distance (in)
int yoffset = 5; // Horizontal distance of feet from body (in)
int stepDuration = 250; // Time duration of a step in (ms)

// Pins
int relay = 22;
int servos[6][3] = {{9, 10, 11},
                    {0, 1, 2},
                    {3, 4, 5},
                    {3, 4, 5},
                    {3, 4, 5},
                    {3, 4, 5}}; // [leg 1-6][servo A-C]

// Servo properties
String labels[] = {"A", "B", "C"};
const int N = 3; // number of servos
const int minLimits[] = {0, 40, 0};
const int maxLimits[] = {180, 180, 120};
const int offsets[6][3] = {{0, -3, -1},
                           {10, 20, 15},
                           {10, 5, 8},
                           {5, 3, 2},
                           {11, 12, 0},
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

// State variables
long stepStartTime = 0;
int counter = 0;
bool enabled = false;

void setup() {
  Serial.begin(9600);
  Serial.println("Starting up! Type \"walk\" to begin walking.");
  pwm.begin();  
  pinMode(relay, OUTPUT);
  digitalWrite(relay, HIGH);
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  delay(10);
}

void loop() {
  // Parse input
  if (Serial.available() > 0) {
    String input = Serial.readString();
    if(input == "walk") {
      Serial.println("Entering walk mode");
      enabled = true;
    } else {
      enabled = false;
      Serial.println("Stopping");
    }
  }
  
  // Walk
  if(enabled && (millis() - stepStartTime > stepDuration)) {
    stepStartTime = millis();
    counter++;
    stepForward(counter);
  }
}

// Move legs into the next configuration of a foward walking gait
void stepForward(int counter) {
  for(int leg=1; leg<7; leg++) {
    if(leg%2==0) { // right side
      moveLegToState(leg, counter%4);
    } else {
      moveLegToState(leg, (counter+2)%4);
    }
  }
}

// Move a leg to a predefined state of the gait
void moveLegToState(int leg, int state) {
  float pos[3];
  getLegPosition(leg, state, pos);
  moveLegToPosition(pos[0], pos[1], pos[2], leg);
}

// Generate a position vector for a given state of a leg
// Legs numbered 1-6 CCW from front left
// States numbered 1-4 back-up-forward-down
void getLegPosition(int leg, int state, float* output) {
    float pos[] = {R*cos(leg*M_PI/3-M_PI/6),R*sin(leg*M_PI/3-M_PI/6),ground};
    if(state == 2) { // leg lifted
        pos[2] += clearance;
    }
    if (leg <=3) { // left side
        pos[1] += yoffset;
    } else {
        pos[1] -= yoffset;
    }
    if (state == 1) { // leg back
        pos[0] -= dx;
    }
    else if (state == 3) { // leg forward
        pos[0] += dx;
    }
    for(int i=0; i<3; i++) {
      output[i] = pos[i];
    }
}

// Move all 3 servos of a leg to position the end effector
void moveLegToPosition(float x, float y, float z, int leg) {
  int *angles = getAngles(x, y, z, leg);
  if(angles[0] != -1) {
    moveLeg(angles, leg);
  }
}

// Determine servo angles (deg) from position relative to hexapod center (in)
// x: forward, y: left, z: up, leg: CCW starting with front left leg
int getAngles(float x, float y, float z, int leg) {
    int error[] = {-1, -1, -1};
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
        return(error);
    }
    float r1 = (pow(r,3)+r*pow(z,2)+r*pow(L2,2)-r*pow(L3,2)-z*sqrt(-pow(r,4)-pow(z,4)-pow((pow(L2,2)-pow(L3,2)),2)+2*pow(z,2)*
        (pow(L2,2)+pow(L3,2))+2*pow(r,2)*(-pow(z,2)+pow(L2,2)+pow(L3,2))))/(2*(pow(r,2)+pow(z,2)));
    float z1 = (pow(z,3)+z*pow(r,2)+z*pow(L2,2)-z*pow(L3,2)+r*sqrt(-pow(r,4)-pow(z,4)-pow((pow(L2,2)-pow(L3,2)),2)+2*pow(z,2)*
        (pow(L2,2)+pow(L3,2))+2*pow(r,2)*(-pow(z,2)+pow(L2,2)+pow(L3,2))))/(2*(pow(r,2)+pow(z,2)));
    if (isnan(r1)|| isnan(z1)) {
        Serial.println("Requested configuration has no solution");
        return(error);
    }
    b = -atan2(z1, r1);
    c = -atan2(z-z1, r-r1) - b;
    a = 180/M_PI*a + 90;
    b = -180/M_PI*b + 90;
    c = 180/M_PI*c;
    int angles[] = {round(a), round(b), round(c)};
    for(int i=0; i<3; i++) {
      if (angles[i] > maxLimits[i] || angles[i] < minLimits[i]) {
        Serial.print("Requested configuration outside servo range: Servo ");
        Serial.println(i);
        return(error);
      }
    }
    return(angles);
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
    pwm.setPWM(servos[leg-1][servo], 0, pulse);
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
