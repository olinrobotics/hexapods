#include <Adafruit_PWMServoDriver.h>
#include "Constants.h"
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver();

int minval = 40;
int maxval = 120;
float val = minval;
float d = 1;
int leg = 1;
int servo = 1;
bool increasing = true;

void setup() {
  // put your setup code here, to run once:
  pwm1.begin();
  pinMode(foot1, INPUT);
  pinMode(relay, OUTPUT);
  digitalWrite(relay, HIGH);
  pwm1.setPWMFreq(60);
  Serial.begin(9600);
  pinMode(foot6, INPUT);
}

void loop() {
  
  if(!digitalRead(foot6)) {
    return;
  }
  // put your main code here, to run repeatedly:
  if(increasing) {
    val+=d;
    if(val > maxval) {
      val = maxval;
      increasing = false;
    }
  } else {
    val-=d;
    if(val < minval) {
      val = minval;
      increasing = true;
    }
  }
  moveServo(val, leg, servo);
  Serial.println(val);
  delay(1);
}

// Move a specified servo to a given angle in degrees
void moveServo(float value, int leg, int servo) {
  if (value >= minLimits[servo] && value <= maxLimits[servo]) {
    float pulse = pulseLength(value - offsets[leg - 1][servo], leg, servo);
    pwm1.setPWM(servos[leg - 1][servo], 0, pulse);
  } else if (value != -1) {
    Serial.print("Servo ");
    Serial.print(labels[servo]);
    Serial.print(" out of bounds: please enter a value between ");
    Serial.print(minLimits[servo]);
    Serial.print(" and ");
    Serial.println(maxLimits[servo]);
  }
}


// Convert angle in degrees to PWM pulse length
float pulseLength(float angle, int leg, int servo) {
  float len = map(angle, ANGLEMIN, ANGLEMAX, SERVOMIN, SERVOMAX);
  if (leg > 3 && servo > 0) {
    len = map(180 - angle, ANGLEMIN, ANGLEMAX, SERVOMIN, SERVOMAX);
  }
  return (len);
}
