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

Servo Cservo;
const int cpin = 11; //attach C servo to pin 11
Servo Bservo;
const int bpin = 10; //attach B servo to pin 10
Servo Aservo;
const int apin = 9; //attach A servo to pin 9

// Servo max and min values for software e-stop
const int amin = 0;
const int amax = 180;
const int bmin = 40;
const int bmax = 180;
const int cmin = 0;
const int cmax = 130;

String state = "stop ";   //create a string for the state of the robot
int new_pos[] = {90, 90, 90}; //for user input to move the servo around
String which_servo = "a"; //variable for determining which servo to move
boolean realTimeStop = true; //real time control loop flag

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Starting up!");
  Aservo.attach(apin);
  Bservo.attach(bpin);
  Cservo.attach(cpin);
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
  Serial.println("Moving to position.");
  if(new_pos[0] >= amin && new_pos[0] <= amax) {
    Aservo.write(new_pos[0]);
    delay(100);
  } else {
    Serial.print("Servo A out of bounds: please enter a value between ");
    Serial.print(amin);
    Serial.print(" and ");
    Serial.println(amax);
  }
  if(new_pos[1] >= bmin && new_pos[1] <= bmax) {
    Bservo.write(new_pos[1]);
    delay(100);
  } else {
    Serial.print("Servo B out of bounds: please enter a value between ");
    Serial.print(bmin);
    Serial.print(" and ");
    Serial.println(bmax);
  }
  if(new_pos[2] >= cmin && new_pos[2] <= cmax) {
    Cservo.write(new_pos[2]);
    delay(100);
  } else {
    Serial.print("Servo C out of bounds: please enter a value between ");
    Serial.print(cmin);
    Serial.print(" and ");
    Serial.println(cmax);
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

