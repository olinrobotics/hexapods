#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver();
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

// Pins
int relay = 22;

// State variables
String state = "test";   //create a string for the state of the robot
int new_pos = -1; //for user input to move the servo around
boolean realTimeStop = true; //real time control loop flag

// Servo PWM to angle conversion factors
const int ANGLEMIN = 0; // minimum servo angle in degrees
const int ANGLEMAX = 180; // maximum servo angle in degrees
const int SERVOMIN = 140; // minimum pwm pulse length count (out of 4096)
const int SERVOMAX = 520; // maximum pwm pulse length count (out of 4096)

void setup() {
  Serial.begin(9600);
  Serial.println("Starting up!");
  pwm1.begin();
  pwm2.begin();
  pinMode(relay, OUTPUT);
  digitalWrite(relay, HIGH);
  pwm1.setPWMFreq(60);
  pwm2.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  delay(10);
}

void loop() {
  Serial.println("Testing Robot!");
  delay(100);
  robotPlay();
  realTimeStop = true;
}
//------------------------------------- TEST functions -----------------------------------
void robotPlay() {
  //Allows user to set different positions for the servos.
  Serial.println("Hi! I've reached the robotPlay function.");
  int i;
  Serial.println("Please enter the next position.");
  while (Serial.available() == 0) {};
  int pin = Serial.parseInt();
  new_pos = Serial.parseInt();
  Serial.print("You entered ... ");
  Serial.print(pin);
  Serial.print(", ");
  Serial.println(new_pos);
  moveServo(pin, new_pos);
}

void moveServo(int pin, int value) {
  if(pin==0) return;
  // Move a specified servo to the given position
  Serial.println("Moving to position.");
  Serial.println(pin);
  Serial.println(value);
  if(pin < 16) {
    pwm1.setPWM(pin, 0, pulseLength(value));
  } else {
    pwm2.setPWM(pin-16, 0, pulseLength(value));
  }
  delay(100);
}

// Convert angle in degrees to PWM pulse length
int pulseLength(int angle) {
  int len = map(angle, ANGLEMIN, ANGLEMAX, SERVOMIN, SERVOMAX);
  return len;
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
