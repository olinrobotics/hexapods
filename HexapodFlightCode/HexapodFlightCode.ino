#include "Hexapod.h"

Hexapod hex = Hexapod();
enum State {NONE, STAND, SIT, WALK, TEST, PACE, WANDER, UPDATE};
State state = NONE;
float forward = 0;
float left = 0;
float turn = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Starting up! Please enter desired linear and angular velocities.");
  Serial.print("Max linear speed: "); Serial.println(linSpeed);
  Serial.print("Max angular speed: "); Serial.println(angSpeed);
  Serial.println("Input values are scaled between -1 and 1");
  delay(10);
  hex.init();
}

void loop() {
  // Operator input
  if (Serial.available() > 0) {
    if (STATE_VERBOSE) {Serial.println("Found a Serial Command!");}
    if (Serial.peek() == ' ') { // Stop
      if (STATE_VERBOSE) {Serial.println("Command: Stop");}
      if (state == STAND) {
        state = SIT;
        Serial.println("Sit");
      } else {
        state = STAND;
        Serial.println("Stand");
      }
      Serial.read();
    } else if (Serial.peek() == 't') { // Test calibration
      state = TEST;
      Serial.println("Test");
      Serial.read();
    } else if (Serial.peek() == 'p') { // Pace back and forth
      state = PACE;
      hex.resetPosition();
      hex.clearWaypoints();
      hex.addDestination(12, 0);
      Serial.println("Pace");
      Serial.read();
//    } else if (Serial.peek() == 'r') { // Reset position and waypoints
//      hex.resetPosition();
//      hex.clearWaypoints();
//      Serial.read();
//    } else if (Serial.peek() == 'w') { //"Wander" mode; takes off at random, sensing cliffs and walls)
//      if (STATE_VERBOSE) {Serial.println("Command: Wander");}
//      state = WANDER;
//      hex.addWalkSteps(1, 0, -1);
//      Serial.read();

    } else if (Serial.peek() == 'a') { // Move left
      if (STATE_VERBOSE) {Serial.println("Move: 0 1");}
      state = WALK;
      forward = 0;
      turn = 0;
      left = 1;
      Serial.read(); 
    } else if (Serial.peek() == 's') { // Move backward
      if (STATE_VERBOSE) {Serial.println("Move: -1 0");}
      state = WALK;
      forward = -1;
      turn = 0;
      left = 0;
      Serial.read(); 
    } else if (Serial.peek() == 'd') { // Move right
      if (STATE_VERBOSE) {Serial.println("Move: 0 -1");}
      state = WALK;
      forward = 0;
      turn = 0;
      left = -1;
      Serial.read(); 
    } else if (Serial.peek() == 'w') { // Move forward
      if (STATE_VERBOSE) {Serial.println("Move: 1 0");}
      state = WALK;
      forward = 1;
      turn = 0;
      left = 0;
      Serial.read(); 
    } else if (Serial.peek() == 'q') { // Turn left
      if (STATE_VERBOSE) {Serial.println("Turn: 1");}
      state = WALK;
      forward = 0;
      turn = 1;
      left = 0;
      Serial.read(); 
    }  else if (Serial.peek() == 'e') { // Turn right
      if (STATE_VERBOSE) {Serial.println("Turn: -1");}
      state = WALK;
      forward = 0;
      turn = -1;
      left = 0;
      Serial.read(); 
    } else if (Serial.peek() == 'j') { // Twist left
      if (STATE_VERBOSE) {Serial.println("Twist: left");}
      state = UPDATE;
      hex.rotateBody(0,0,-.05);
      Serial.read(); 
    } else if (Serial.peek() == 'l') { // Twist right
      if (STATE_VERBOSE) {Serial.println("Twist: right");}
      state = UPDATE;
      hex.rotateBody(0,0,.05);
      Serial.read(); 
    } else if (Serial.peek() == 'i') { // Pitch up
      if (STATE_VERBOSE) {Serial.println("Pitch: up");}
      state = UPDATE;
      hex.rotateBody(0,-.05,0);
      Serial.read(); 
    } else if (Serial.peek() == 'k') { // Pitch down
      if (STATE_VERBOSE) {Serial.println("Pitch: down");}
      state = UPDATE;
      hex.rotateBody(0,.05,0);
      Serial.read();
    } else if (Serial.peek() == 'u') { // Roll left
      if (STATE_VERBOSE) {Serial.println("Roll: left");}
      state = UPDATE;
      hex.rotateBody(.05,0,0);
      Serial.read();
    } else if (Serial.peek() == 'o') { // Roll right
      if (STATE_VERBOSE) {Serial.println("Roll: right");}
      state = UPDATE;
      hex.rotateBody(-.05,0,0);
      Serial.read();
    } /*else if (Serial.peek() == 'q') { // This is just a test
      state = FOLLOW;
      hex.resetPosition();
      hex.clearWaypoints();
      hex.addWalkSteps(1, 0, 5);
      hex.addDelay(3);
      hex.addWalkSteps(-1, 0, 5);
      Serial.read();
    } */ else {
      if (STATE_VERBOSE) {Serial.println("I'm going to try to walk.");}
      forward = Serial.parseFloat();
      turn = Serial.parseFloat();
      left = 0;
      if (abs(forward) + abs(turn) <= 100000) { // Walk
        Serial.print("Walk: ");
        Serial.print(forward);
        Serial.print(", ");
        Serial.println(turn);
        state = WALK;
      } else { // Invalid input
        Serial.println("Invalid velocity");
      }
    }
    Serial.read();
  }

  // hex.walk(forward, turn)) {
        //Serial.println("Finding new path");
        //hex.trynewpath(2, 2);
      //Act
  if (state == WALK) {
    if(hex.goTo(forward, left, turn) == 1) {
      Serial.println("Stopping");
//      hex.stand();
    }
  } else if (state == STAND) {
    hex.stand();
  } else if (state == PACE) {
    if(hex.followWaypoint()) {
      hex.addDestination(12-hex.x, 0);
    }
  } else if (state == SIT) {
    hex.sit();
  } else if (state == TEST) {
    hex.testCalibration();
  } else if (state == WANDER) {
      if(hex.followWaypoint() == -1) {
        hex.clearWaypoints();
        hex.addWalkSteps(-1, 0, 3);
        hex.addWalkSteps(0, 1, random(4, 10));
        hex.addWalkSteps(1, 0, -1);
      }
  }  else if (state == UPDATE) {
    hex.updateServos();
  } /*else if (state == FOLLOW) {
    hex.followWaypoint();
  } */
  delay(1);
}
