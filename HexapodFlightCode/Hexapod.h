#ifndef Hexapod_h
#define Hexapod_h

#include "Arduino.h"
#include "Constants.h"
#include <Adafruit_PWMServoDriver.h>

class Hexapod {
  public:
    // Initialize pin modes and servo shield
    void init();
    
    // Move legs into the next configuration of a foward walking gait
    void walk(float forward, float turn, int counter);
    
    // Lower the hexapod to the ground
    void sit();
    
    // Stand with all 6 legs on the ground
    void stand();
    
    // Move a leg to a predefined state of the gait
    void moveLegToState(int leg, int state, float forward, float turn);
    
    // Generate a position vector for a given state of a leg
    void getLegPosition(int leg, int state, float forward, float turn, float* output);
    
    // Move all 3 servos of a leg to position the end effector
    void moveLegToPosition(float x, float y, float z, int leg);
    
    // Determine servo angles (deg) from position relative to hexapod center (in)
    void getAngles(float x, float y, float z, int leg, int* angles);
    
    // Move all 3 servos of a leg to a given state
    void moveLeg(int *angles, int leg);
    
    // Move a specified servo to a given angle in degrees
    void moveServo(int value, int leg, int servo);
    
    // Convert angle in degrees to PWM pulse length
    int pulseLength(int angle, int leg, int servo);

    Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver();
    Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);
};

#endif
