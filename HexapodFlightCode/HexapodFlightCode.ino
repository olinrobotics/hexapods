#include "Hexapod.h"

Hexapod hex = Hexapod();
enum State {NONE, STAND, SIT, WALK, TEST, PACE, WANDER};
State state = NONE;
float forward = 0;
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
    } else if (Serial.peek() == 'r') { // Reset position and waypoints
      hex.resetPosition();
      hex.clearWaypoints();
      Serial.read();
    } else if (Serial.peek() == 'w') { //"Wander" mode; takes off at random, sensing cliffs and walls)
      if (STATE_VERBOSE) {Serial.println("Command: Wander");}
      state = WANDER;
      hex.addWalkSteps(1, 0, -1);
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
      if (abs(forward) + abs(turn) <= 1) { // Walk
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
    if(hex.walk(forward, turn) == -1) {
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
  } /*else if (state == FOLLOW) {
    hex.followWaypoint();
  } */
  delay(1);
}
