#ifndef Constants_h
#define Constants_h

// Settings
const bool LEGS_VERBOSE = false; // Use for debugging leg positions
const bool LOCATION_VERBOSE = true; // Use for debugging hexapod location
const bool IR_VERBOSE = false; // Use for debugging IR sensor data
const bool ACCEL_VERBOSE = false; // Use for debugging accelerometer data
const bool STATE_VERBOSE = true; // Debug main states in flight code
const bool DETECT_WALLS = true; // Use IRs to avoid walls
const bool DETECT_CLIFFS = true; // Use foot sensor to avoid falling

// Waypoint types
const int WAYPOINT_STAND = 0; // Stand
const int WAYPOINT_SIT = -1; // Sit
const int WAYPOINT_INFINITE = -2; // Walk indefinitely with given gait
const int WAYPOINT_DESTINATION = -3; // Walk to given coordinates
const int WAYPOINT_DELAY = -4; // Wait for a given time interval

// Gait parameters
const float ground = -6; // Height of ground relative to body (in)
const float clearance = 2; // Height of raised leg relative to ground (in)
const float dx = 1.5/2; // Half of the linear step distance (in)
const float dtheta = M_PI/12/2; // Half of the angular step angle (rad)
const float yoffset = 3; // Horizontal distance of feet from body (in)
const int stepDuration = 1000; // Time duration of a step (ms)
const int TILT_THRESHOLD = 3; // Max allowed horizontal acceleration (m/s^2)
const int FILTER_LENGTH = 10; // Number of accelerometer values to average
const int IR_THRESHOLD = 28; // Distance seen by IR sensor to make it stop (cm)

// Derived gait parameters
const float linSpeed = dx*1000/stepDuration; // Max linear speed (in/s)
const float angSpeed = dtheta*1000/stepDuration; // Max angular speed (rad/s)

// Pins
const int relay = 22;
const int servos[6][3] = {{0, 1, 2},
                          {3, 4, 5},
                          {6, 7, 8},
                          {16, 17, 18},
                          {19, 20, 21},
                          {22, 23, 24}}; // [leg 1-6][servo A-C]

const int LIS3DH_CLK = 13;
const int LIS3DH_MISO = 12;
const int LIS3DH_MOSI = 11;
const int LIS3DH_CS = 10;
const int irRpin = A15;
const int foot1 = 45;
const int foot6 = 44;

// Servo properties
const String labels[] = {"A", "B", "C"};
const int N = 3; // number of servos
const int minLimits[] = {0, 40, 0};
const int maxLimits[] = {180, 180, 120};
const int offsets[6][3] = {{0, -3, -1},
                           {10, 13, 13},
                           {10, 5, 8},
                           {-5, 10, 9},
                           {0, 0, 10},
                           {-2, 8, 6}}; // (actual - desired) servo angle

// Servo PWM to angle conversion factors
const int ANGLEMIN = 0; // minimum servo angle in degrees
const int ANGLEMAX = 180; // maximum servo angle in degrees
const int SERVOMIN = 140; // minimum pwm pulse length count (out of 4096)
const int SERVOMAX = 520; // maximum pwm pulse length count (out of 4096)

// Frame geometry
const float R = 5.35; // Distance from center to Servo A (in)
const float L1 = 1.12; // Distance from Servo A to Servo B (in)
const float L2 = 2.24; // Distance from Servo B to Servo C (in)
const float L3 = 4.84; // Distance from Servo C to end effector (in)

#endif
