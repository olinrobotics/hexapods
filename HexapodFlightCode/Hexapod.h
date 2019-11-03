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

    // Add a walk command to the end of the waypoint list
    int addWalkSteps(float forward, float turn, int steps);

    // Add a destination to the end of the waypoint list
    int addDestination(float x, float y);

    // Add a delay command in seconds to the end of the waypoint list
    int addDelay(float duration);

    // Add a new desired waypoint to the end of the list
    int addWaypoint(float x, float y, int type);

    // Remove all waypoints
    void clearWaypoints();

    // Walk toward the next waypoint
    // Returns 1 if waypoint reached, -1 if obstacle encountered, 0 otherwise
    int followWaypoint();

    // Set current position to new origin
    void resetPosition();

/// BEGIN NEW GAIT CODE //////////////////////////////////////////////////////////

    // Called iteratively to move the hexapod to the given coordinates
    // Returns true if target position reached
    bool goTo(float x2, float y2, float theta2);

    // Incrementally move a foot triangle by given displacement
    bool moveTripod(float dx, float dy, float dz, float dtheta, bool even, bool ignoreLimits);

    // Levels the hexapod body relative to the ground
    // Returns false if position is unreachable
    bool levelBody(float pitch, float roll);
    
    // Determine midpoint of tripod
    void getCentroid(bool even, float* centroid);

    // Adjust radius and angle offset of a raised foot triangle
    // Positive angle = space legs closer horizontally
    bool resizeTripod(float radius, float angle, bool even);

/// END NEW GAIT CODE /////////////////////////////////////////////////////////////

    // Called iteratively to walk with given x, y, and angular velocities (normalized)
    // Returns 1 if step taken, -1 if obstacle encountered, 0 otherwise
    int walk(float forward, float left, float turn);

    // Called iteratively to lower feet to the ground and level out hexapod body
    // Returns true if complete
    bool balance();

    // Move legs into the next configuration of a walking gait
    void step(float forward, float left, float turn, int counter);

    // Lower the hexapod to the ground
    void sit();

    // Stand with all 6 legs on the ground
    void stand();

    // Move all servos to 90 degrees
    void testCalibration();

    // Move a leg to a predefined state of the gait
    void moveLegToState(int leg, int state, float forward, float left, float turn);

    // Generate a position vector for a given state of a leg
    void getLegPosition(int leg, int state, float forward, float turn, float left, float* output);

    // States numbered 1-6 (up, forward, down, down, back, back)
    int getLegState(int leg, int counter);

    // Displaces the hexapod body by a set amount relative to the ground
    void translateBody(float dx, float dy, float dz);

    // Rotates the hexapod body by a set amount relative to the ground
    void rotateBody(float droll, float dpitch, float dyaw);

    // Move all 3 servos of a leg to position the end effector
    // Return false if given position is out of range
    bool moveLegToPosition(float x, float y, float z, int leg);

    // Incrementally move all 3 servos of a leg to position the end effector
    void moveLegToPositionSmooth(float x, float y, float z, int leg);

    // Determine current leg position (in) relative to hexapod center
    void getCurrentPosition(int leg, float* pos);

    // Determine whether leg can move to a given position
    bool isPositionValid(float x, float y, float z, int leg);
    
    // Determine position (in) relative to hexapod center from servo angles (deg)
    void getPosition(float a, float b, float c, int leg, float* pos);
    
    // Determine servo angles (deg) from position relative to hexapod center (in)
    void getAngles(float x, float y, float z, int leg, int* angles);

    // Move all 3 servos of a leg to a given state
    void moveLeg(int *angles, int leg);

    // Move a specified servo to a given angle in degrees
    void moveServo(int value, int leg, int servo);

    // Incrementally move a specified servo to a given angle in degrees
    void moveServoSmooth(int value, int leg, int servo);

    // Sets leg target position to its current position
    void stopLeg(int leg);

    // Directly move all servos to their target angles
    void moveServosDirect();
    
    // Increments servos toward desired positions
    bool updateServos();

    // Convert angle in degrees to PWM pulse length
    int pulseLength(int angle, int leg, int servo);

    // Determine {x,y,z} acceleration in m/s^2
    void getAccel(float *acceleration);

    //sample IR sensors
    int sampleIR();

    // Hexapod coordinates foward (in), left(in), and CCW (rad)
    float x = 0, y = 0, theta = 0;

    Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver();
    Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);
    Adafruit_LIS3DH accel = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
//    Adafruit_LIS3DH accel = Adafruit_LIS3DH();
    SharpIR IR_r = SharpIR(SharpIR::GP2Y0A21YK0F, irRpin);
};

#endif
