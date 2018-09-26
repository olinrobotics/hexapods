/*
 * Code to test the functionality and limits of the Hexapod servos.
 * 
 * Last Updated 9/19/18 by Hannah Kolano
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

String state = "stop ";   //create a string for the state of the robot
int new_pos[] = {-1, -1, -1}; //for user input to move the servo around
String which_servo = "a"; //variable for determining which servo to move
boolean realTimeStop = true; //real time control loop flag

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
    new_pos[i] = Serial.parseInt();
    Serial.print("You entered ... ");
    Serial.println(new_pos[i]);
  }
  for (i = 0; i < 3; i++) {
    moveServo(i, new_pos[i]);
  }
}

void moveServo(int servo, int value) {
  // Move a servo to the given position
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

