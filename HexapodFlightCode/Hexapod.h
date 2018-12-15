#ifndef Hexapod_h
#define Hexapod_h

#include "Arduino.h"
#include "Constants.h"
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>
#include <SharpIR.h>


class Hexapod {
  public:
    // Initialize pin modes and servo shield
    void init();

    // Add a new desired waypoint to the end of the list
    int addWaypoint(float x, float y);

    // Remove all waypoints
    void clearWaypoints();

    // Walk toward the next waypoint
    bool followWaypoint();
    
    // Set current position to new origin
    void resetPosition();
    
    // Called iteratively to walk with given linear and angular velocities
    bool walk(float forward, float turn);
    
    // Move legs into the next configuration of a walking gait
    void step(float forward, float turn, int counter);
    
    // Lower the hexapod to the ground
    void sit();
    
    // Stand with all 6 legs on the ground
    void stand();

    // Move all servos to 90 degrees
    void testCalibration();
    
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

    // Determine {x,y,z} acceleration in m/s^2
    void Hexapod::getAccel(float *acceleration);

    // Hexapod coordinates foward (in), left(in), and CCW (rad)
    float x = 0, y = 0, theta = 0;

    //sample IR sensors
    int sampleIR();

    // If hits a wall, back up and turn somewhere else. 
    void trynewpath(int stepsback, int turns);

    Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver();
    Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);
    Adafruit_LIS3DH accel = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
    SharpIR IR_r = SharpIR(SharpIR::GP2Y0A21YK0F, irRpin);
};

#endif
