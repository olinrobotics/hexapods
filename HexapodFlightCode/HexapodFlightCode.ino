#include "Hexapod.h"

Hexapod hex = Hexapod();
enum State {NONE, STAND, SIT, WALK};
long stepStartTime = 0;
int counter = 0;
State state = NONE;
float forward = 0;
float turn = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Starting up! Please enter desired linear and angular velocities.");
  delay(10);
  hex.init();
}

void loop() {
  // Operator input
  if (Serial.available() > 0) {
    if (Serial.peek() == ' ') { // Stop
      if (state == STAND) {
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
      if (abs(forward) + abs(turn) <= 1) { // Walk
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
  if (state == WALK && (millis() - stepStartTime > stepDuration)) {
    stepStartTime = millis();
    hex.walk(forward, turn, counter);
    counter++;
  } else if (state == STAND) {
    hex.stand();
  } else if (state == SIT) {
    hex.sit();
  }
  delay(1);
}
