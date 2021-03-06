#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Pins
int relay = 22;
int servos[] = {9, 10, 11};

// Servo properties
String labels[] = {"A", "B", "C"};
const int N = 3; // number of servos
const int minLimits[] = {0, 40, 0};
const int maxLimits[] = {180, 180, 120};
const int centers[] = {90, 90, 90};

// Frame geometry
const float R = 5.35; // Distance from center to Servo A in inches
const float L1 = 1.12; // Distance from Servo A to Servo B in inches
const float L2 = 2.24; // Distance from Servo B to Servo C in inches
const float L3 = 4.84; // Distance from Servo C to end effector in inches

// State variables
String state = "stop ";   //create a string for the state of the robot
float new_pos[] = {-1, -1, -1}; //for user input to move the servo around
String which_servo = "a"; //variable for determining which servo to move
boolean realTimeStop = true; //real time control loop flag

// Servo PWM to angle conversion factors
const int ANGLEMIN = 0; // minimum servo angle in degrees
const int ANGLEMAX = 180; // maximum servo angle in degrees
const int SERVOMIN = 140; // minimum pwm pulse length count (out of 4096)
const int SERVOMAX = 520; // maximum pwm pulse length count (out of 4096)

void setup() {
  Serial.begin(9600);
  Serial.println("Starting up!");
  pwm.begin();  
  pinMode(relay, OUTPUT);
  digitalWrite(relay, HIGH);
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  delay(10);
}

void loop() {

  //----------------------------------------OCU-----------------------------------------
  state = getOperatorInput();
  if (state == "stop") realTimeStop = false;
  else realTimeStop = true;


 //-------------------------------------real time loop ----------------------------------
  while(realTimeStop == true) {
    if (Serial.available() > 0) {
      realTimeStop = false;
      state = Serial.readString();
      break;
    }
    else {realTimeStop = true;}


    //----------------------------------- state machine ---------------------------------
    if (state == "stop") {
      Serial.println("Stop Robot");
      realTimeStop = true;
    }
    else if (state == "test") {
      Serial.println("Testing Robot!");
      delay(100);
      robotPlay();
      realTimeStop = true;
    }
    else {
      Serial.println("Nope, that's not a state! Please try again.");
      realTimeStop = false;
    }
  }
  //send state to OCU -------------------- Send OCU update -------------------------------
  Serial.println("Robot control loop stopping to wait for new command");
}
//------------------------------------- TEST functions -----------------------------------
void robotPlay() {
  //Allows user to set different positions for the servos.
  Serial.println("Hi! I've reached the robotPlay function.");
  int i;
  for (i = 0; i < 4; i++) {
    Serial.println("Please enter the next position.");
    while (Serial.available() == 0) {};
    
    switch(Serial.peek()) {
//      case 'a': moveServos(A_low); Serial.readString(); return;
//      case 'b': moveServos(B_low); Serial.readString(); return;
//      case 'c': moveServos(C_low); Serial.readString(); return;
//      case 'x': moveServos(A_high); Serial.readString(); return;
//      case 'y': moveServos(B_high); Serial.readString(); return;
//      case 'z': moveServos(C_high); Serial.readString(); return;
//      case 'o': moveServos(centers); Serial.readString(); return;
    }
    new_pos[i] = Serial.parseFloat();
    Serial.print("You entered ... ");
    Serial.println(new_pos[i]);
  }
  moveToPosition(new_pos[0], new_pos[1], new_pos[2], new_pos[3]);
//  for (i = 0; i < 3; i++) {
//    moveServo(i, new_pos[i]);
//  }
}

void moveServos(int *positions, int leg) {
  // Move all 3 servos to a new position
  Serial.print("Moving to ");
  Serial.print(positions[0]);
  Serial.print(", ");
  Serial.print(positions[1]);
  Serial.print(", ");
  Serial.println(positions[2]);
  for(int i=0; i<3; i++) {
    moveServo(positions[i], leg, i);
  }
}

void moveServo(int value, int leg, int servo) {
  // Move a specified servo to the given position
  if(value >= minLimits[servo] && value <= maxLimits[servo]) {
    Serial.println("Moving to position.");
    pwm.setPWM(servos[servo], 0, pulseLength(value, leg, servo));
    delay(100);
  } else if(value!=-1) {
    Serial.print("Servo ");
    Serial.print(labels[servo]);
    Serial.print(" out of bounds: please enter a value between ");
    Serial.print(minLimits[servo]);
    Serial.print(" and ");
    Serial.println(maxLimits[servo]);
  }
}

void moveToPosition(float x, float y, float z, int leg) {
  int *angles = getAngles(x, y, z, leg);
  if(angles[0] != -1) {
    moveServos(angles, leg);
  }
}

// Determine servo angles given a position in inches relative to hexapod center
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
    Serial.println(a);
    Serial.println(b);
    Serial.println(c);
    a = 180/M_PI*a + 90;
    b = -180/M_PI*b + 90;
    c = 180/M_PI*c;
    Serial.println(a);
    Serial.println(b);
    Serial.println(c);
    int angles[] = {round(a), round(b), round(c)};
    Serial.println(angles[0]);
    for(int i=0; i<3; i++) {
      if (angles[i] > maxLimits[i] || angles[i] < minLimits[i]) {
        Serial.print("Requested configuration outside servo range: Servo ");
        Serial.println(i);
        return(error);
      }
    }
    return(angles);
}

// Convert angle in degrees to PWM pulse length
int pulseLength(int angle, int leg, int servo) {
  int len = map(angle, ANGLEMIN, ANGLEMAX, SERVOMIN, SERVOMAX);
  if(leg > 3 && servo > 0) {
    len = map(180-angle, ANGLEMIN, ANGLEMAX, SERVOMIN, SERVOMAX);
  }
  return(len);
}

// -------------------------------- OCU FUNCTIONS ----------------------------------------

String getOperatorInput(){
  Serial.print("Current state of the robot is ");
  Serial.println(state);
  Serial.println("Please enter new state.");

  while (Serial.available() == 0) {};
  state = Serial.readString();
  Serial.print("New robot behavior is: ");
  Serial.println(state);

  return state;
}
