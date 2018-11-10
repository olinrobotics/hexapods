#include "Hexapod.h"

Hexapod hex = Hexapod();
long stepStartTime = 0;
int counter = 0;
bool enabled = false;
float forward = 0;
float turn = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Starting up! Please enter desired linear and angular velocities.");
  delay(10);
}

void loop() {
  // Parse input
  if(Serial.available() > 0) {
    if(Serial.peek() == ' ') { // Stop
      enabled = false;
      Serial.println("Stop");
      Serial.read();
    } else {
      forward = Serial.parseFloat();
      turn = Serial.parseFloat();
      if(abs(forward)+abs(turn) <= 1) { // Walk
        Serial.print("Entering walk mode: ");
        Serial.print(forward);
        Serial.print(", ");
        Serial.println(turn);
        enabled = true;
      } else { // Invalid input
        enabled = false;
        Serial.println("Invalid velocity");
      }
    }
  }
  
  // Walk
  if(enabled && (millis() - stepStartTime > stepDuration)) {
    stepStartTime = millis();
    counter++;
    hex.walk(forward, turn, counter);
  }
}
