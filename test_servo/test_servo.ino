#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Pins
int relay = 22;
int servo = 10;

// State variables
String state = "stop ";   //create a string for the state of the robot
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
  Serial.println("Please enter the next position.");
  while (Serial.available() == 0) {};
  new_pos = Serial.parseInt();
  Serial.print("You entered ... ");
  Serial.println(new_pos);
  moveServo(new_pos);
}

void moveServo(int value) {
  // Move a specified servo to the given position
  Serial.println("Moving to position.");
  pwm.setPWM(servo, 0, pulseLength(value));
  delay(100);
}

// Convert angle in degrees to PWM pulse length
int pulseLength(int angle) {
  int len = map(angle, ANGLEMIN, ANGLEMAX, SERVOMIN, SERVOMAX);
  return len;
}

// -------------------------------- OCU FUNCTIONS ----------------------------------------

String getOperatorInput(){
  //
  Serial.print("Current state of the robot is ");
  Serial.println(state);
  Serial.println("Please enter new state.");

  while (Serial.available() == 0) {};
  state = Serial.readString();
  Serial.print("New robot behavior is: ");
  Serial.println(state);

  return state;
}
