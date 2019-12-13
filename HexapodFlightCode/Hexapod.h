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

    // Called iteratively to move the hexapod in the given direction
    bool walk(float vx, float vy, float vtheta);

    // Called iteratively to move the hexapod to the given coordinates
    // Returns true if target position reached
    bool goTo(float x2, float y2, float theta2);

    // Convert from hexapod to world frames
    float hexToWorld(float xval, float yval, bool returnx);

    // Convert from world to hexapod frames
    float worldToHex(float xval, float yval, bool returnx);

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

    // Resets a tripod to default stance, centered at (x0, y0, z0)
    void resetTripod(float x0, float y0, float z0, bool even);

    // Mimic accelerometer motion
    void dance();

    // Lower the hexapod to the ground
    void sit();

    // Stand with all 6 legs on the ground
    void stand();

    // Move all servos to 90 degrees
    void testCalibration();

    // Move all 3 servos of a leg to position the end effector
    // Return false if given position is out of range
    bool moveLegToPosition(float x, float y, float z, int leg);
    
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

    // Convert angle in degrees to PWM pulse length
    int pulseLength(int angle, int leg, int servo);

    // Rotate lidar servo to angle in degrees below horizontal
    int tiltLidar(int angle);

    // Determine {x,y,z} acceleration in m/s^2
    void getAccel(float *acceleration);

    //sample IR sensors
    int sampleIR();

    // Hexapod coordinates foward (in), left(in), and CCW (rad)
    float x = 0, y = 0, theta = 0;

    Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver();
    Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);
    Adafruit_LIS3DH accel = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
//    Adafruit_LIS3DH accel = Adafruit_LIS3DH(); // I2C
    SharpIR IR_r = SharpIR(SharpIR::GP2Y0A21YK0F, irRpin);
};

#endif
