/*
 * Code to test the functionality and limits of the Hexapod servos.
 * 
 * In test mode:
 * Enter 3 integers separated by spaces to move to a specific position
 * a = min servo A position, x = max servo A position
 * b = min servo B position, y = max servo B position
 * c = min servo C position, z = max servo C position
 * o = neutral position for all 3 servos
 * 
 * Last Updated 9/28/18
 * Hannah.kolano@students.olin.edu
 */

//TODO
//Include software e-stops on limits
//Software e-stops in "testing"

#include <Servo.h>

int pins[] = {9, 10, 11};
String labels[] = {"A", "B", "C"};
const int N = 3; // number of servos

Servo servos[N];

// Servo max and min values for software e-stop
const int minLimits[] = {0, 40, 0};
const int maxLimits[] = {180, 180, 130};
const int centers[] = {90, 80, 90};

// State variables
String state = "stop ";   //create a string for the state of the robot
int new_pos[] = {-1, -1, -1}; //for user input to move the servo around
String which_servo = "a"; //variable for determining which servo to move
boolean realTimeStop = true; //real time control loop flag

// Preprogrammed states for quick testing
const int A_low[] = {minLimits[0], centers[1], centers[2]};
const int A_high[] = {maxLimits[0], centers[1], centers[2]};
const int B_low[] = {centers[0], minLimits[1], centers[2]};
const int B_high[] = {centers[0], maxLimits[1], centers[2]};
const int C_low[] = {centers[0], centers[1], minLimits[2]};
const int C_high[] = {centers[0], centers[1], maxLimits[2]};
const int centered[] = {centers[0], centers[1], centers[2]};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Starting up!");
  for(int i=0; i<N; i++) {
    servos[i].attach(pins[i]);
  }
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
//------------------------------------- end of loop --------------------------------------

//------------------------------------- TEST functions -----------------------------------
void robotPlay() {
  //Allows user to set different positions for the servos.
  Serial.println("Hi! I've reached the robotPlay function.");
  int i;
  for (i = 0; i < 3; i++) {
    Serial.println("Please enter the next position.");
    while (Serial.available() == 0) {};
    
    switch(Serial.peek()) {
      case 'a': moveServos(A_low); Serial.readString(); return;
      case 'b': moveServos(B_low); Serial.readString(); return;
      case 'c': moveServos(C_low); Serial.readString(); return;
      case 'x': moveServos(A_high); Serial.readString(); return;
      case 'y': moveServos(B_high); Serial.readString(); return;
      case 'z': moveServos(C_high); Serial.readString(); return;
      case 'o': moveServos(centers); Serial.readString(); return;
    }
    new_pos[i] = Serial.parseInt();
    Serial.print("You entered ... ");
    Serial.println(new_pos[i]);
  }
  for (i = 0; i < 3; i++) {
    moveServo(i, new_pos[i]);
  }
}

void moveServos(int *positions) {
  // Move all 3 servos to a new position
  Serial.print("Moving to ");
  Serial.print(positions[0]);
  Serial.print(", ");
  Serial.print(positions[1]);
  Serial.print(", ");
  Serial.println(positions[2]);
  for(int i=0; i<3; i++) {
    moveServo(i, positions[i]);
  }
}

void moveServo(int servo, int value) {
  // Move a specified servo to the given position
  if(value >= minLimits[servo] && value <= maxLimits[servo]) {
    Serial.println("Moving to position.");
    servos[servo].write(value);
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

