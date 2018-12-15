#include "Arduino.h"
#include "Hexapod.h"

long stepStartTime = 0; // Tracks elapsed time
int counter = 0; // Tracks current walking state
bool accelPresent = false; // Accelerometer initialized successfully
float pastAccel[FILTER_LENGTH][3]; // Accelerometer moving average values
float *waypointsX; // Linear speed (-1 to 1), target x (in), delay (s)
float *waypointsY; // Angular speed (-1 to 1), target y (in), delay flag
int *waypointsN; // Waypoint type (N<=0), steps remaining (N>0)
int waypointLen = 0; // Number of waypoints
int waypointIndex = 0; // Current waypoint

// Initialize pin modes and servo shield
void Hexapod::init() {
  pwm1.begin();
  pwm2.begin();
  if (accel.begin(0x18)) {
    accelPresent = true;
    accel.setRange(LIS3DH_RANGE_4_G);
  }
  pinMode(relay, OUTPUT);
  pinMode(foot1, INPUT);
  digitalWrite(relay, HIGH);
  pwm1.setPWMFreq(60);
  pwm2.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}

// Add a walk command to the end of the waypoint list
int Hexapod::addWalkSteps(float forward, float turn, int steps) {
  if(steps<0) {
    return(addWaypoint(forward, turn, WAYPOINT_INFINITE));
  } else if(steps==0) {
    return(addWaypoint(forward, turn, WAYPOINT_STAND));
  } else {
    return(addWaypoint(forward, turn, steps*4));
  }
}

// Add a destination to the end of the waypoint list
int Hexapod::addDestination(float x, float y) {
  return(addWaypoint(x, y, WAYPOINT_DESTINATION));
}

// Add a delay command in seconds to the end of the waypoint list
int Hexapod::addDelay(float duration) {
  return(addWaypoint(duration, 0, WAYPOINT_DELAY));
}

// Add a new desired waypoint to the end of the list
int Hexapod::addWaypoint(float targetX, float targetY, int type) {
  float *tempX = malloc(sizeof(float)*(waypointLen+1));
  float *tempY = malloc(sizeof(float)*(waypointLen+1));  
  int *tempN = malloc(sizeof(int)*(waypointLen+1));  
  for(int i=0; i<waypointLen; i++) {
    tempX[i] = waypointsX[i];
    tempY[i] = waypointsY[i];
    tempN[i] = waypointsN[i];
  }
  tempX[waypointLen] = targetX;
  tempY[waypointLen] = targetY;
  tempN[waypointLen] = type;
  waypointLen++;
  free(waypointsX);
  free(waypointsY);
  free(waypointsN);
  waypointsX = tempX;
  waypointsY = tempY;
  waypointsN = tempN;
  return(waypointLen);
}

// Remove all waypoints
void Hexapod::clearWaypoints() {
  waypointLen = 0;
  waypointIndex = 0;
}

// Set current position to new origin
void Hexapod::resetPosition() {
  x = 0;
  y = 0;
  theta = 0;
}

// Walk toward the next waypoint
// Returns 1 if waypoint reached, -1 if obstacle encountered, 0 otherwise
int Hexapod::followWaypoint() {
  if(waypointIndex >= waypointLen) {
    return 1;
  }
  if(waypointsN[waypointIndex]==WAYPOINT_STAND) {
    stand();
    waypointIndex++;
    return 1;
  } else if(waypointsN[waypointIndex]==WAYPOINT_SIT) {
    sit();
    waypointIndex++;
    return 1;
  } else if(waypointsN[waypointIndex]==WAYPOINT_INFINITE) {
    if(walk(waypointsX[waypointIndex], waypointsY[waypointIndex])) {
      return -1;
    }
    return 0;
  } else if(waypointsN[waypointIndex]==WAYPOINT_DESTINATION) {
    float deltaX = waypointsX[waypointIndex]-x;
    float deltaY = waypointsY[waypointIndex]-y;
    float deltaTheta = fmod(atan2(deltaY, deltaX)-theta+M_PI, 2*M_PI)-M_PI;
    if(abs(sqrt(deltaX*deltaX+deltaY*deltaY))<=dx/2) {
      waypointIndex++;
      return 1;
    }
    if(abs(deltaTheta) >= dtheta/2) {
      return(walk(0, deltaTheta>0 ? 1:-1) ? 0 : -1);
    } else {
      return(walk(1, 0) ? 0 : -1);
    }
  } else if(waypointsN[waypointIndex]==WAYPOINT_DELAY) {
    if(waypointsY[waypointIndex] == 0) {
      waypointsY[waypointIndex] = 1; // Flag for timer start
      stepStartTime = millis();
    }
    if(millis() - stepStartTime > waypointsX[waypointIndex]) {
      waypointIndex++;
      return 1;
    }
    return 0;
  } else if(waypointsN[waypointIndex]>0) {
    if(walk(waypointsX[waypointIndex], waypointsY[waypointIndex])) {
      waypointsN[waypointIndex]++;
      return -1;
    }
    waypointsN[waypointIndex]--;
    if(waypointsN[waypointIndex] == 0) {
      waypointIndex++;
      return 1;
    } else {
      return 0;
    }
  }
}

// Called iteratively to walk with given linear and angular velocities
bool Hexapod::walk(float forward, float turn) {
  if (accelPresent) {
    float a[3];
    getAccel(a);
  }
  if (millis() - stepStartTime > stepDuration) {
    stepStartTime = millis();
    if (sampleIR() < 12) {
      Serial.println("Object is too close! Stopping!");
      return false;
    }
    if (counter%4!=1 && digitalRead(foot1)!=LOW) {
      step(forward, turn, counter-2);
      x -= 2*forward*dx*cos(theta);
      y -= 2*forward*dx*sin(theta);
      theta -= 2*turn*dtheta;
      counter--;
      return false;
    }
    step(forward, turn, counter);
    counter++;
  }
  return true;
}

// Move legs into the next configuration of a walking gait
void Hexapod::step(float forward, float turn, int counter) {
  x += forward*dx*cos(theta);
  y += forward*dx*sin(theta);
  theta += turn*dtheta;
  if(LOCATION_VERBOSE) {
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.println(theta);
  }
  for (int leg = 1; leg < 7; leg++) {
    if (leg % 2 == 0) { // right side
      moveLegToState(leg, counter % 4, forward, turn);
    } else {
      moveLegToState(leg, (counter + 2) % 4, forward, turn);
    }
  }
}

// Lower the hexapod to the ground
void Hexapod::sit() {
  for (int leg = 1; leg < 7; leg++) {
    float dR = 5.3;
    float z0 = -0.6;
    float pos[3] = {(R + dR)*cos(leg*M_PI / 3 - M_PI / 6), (R + dR)*sin(leg*M_PI / 3 - M_PI / 6), z0};
    moveLegToPosition(pos[0], pos[1], pos[2], leg);
  }
  counter = 0;
}

// Stand with all 6 legs on the ground
void Hexapod::stand() {
  for (int leg = 1; leg < 7; leg++) {
    moveLegToState(leg, 0, 0, 0);
  }
  counter = 0;
}

// Move all servos to 90 degrees
void Hexapod::testCalibration() {
  int angles[] = {90, 90, 90};
  for (int leg = 1; leg < 7; leg++) {
    moveLeg(angles, leg);
  }
  counter = 0;
}

// Move a leg to a predefined state of the gait
void Hexapod::moveLegToState(int leg, int state, float forward, float turn) {
  float pos[3];
  getLegPosition(leg, state, forward, turn, pos);
  moveLegToPosition(pos[0], pos[1], pos[2], leg);
}

// Generate a position vector for a given state of a leg
// Legs numbered 1-6 CCW from front left
// States numbered 1-4 back-up-forward-down
// forward and turn specify movement direction between -1 and 1
// Resulting position is stored in output
void Hexapod::getLegPosition(int leg, int state, float forward, float turn, float* output) {
  float pos[] = {R * cos(leg*M_PI / 3 - M_PI / 6), R * sin(leg*M_PI / 3 - M_PI / 6), ground};
  if (state == 2) { // leg lifted
    pos[2] += clearance;
  }
  if (leg <= 3) { // left side
    pos[1] += yoffset;
  } else {
    pos[1] -= yoffset;
  }
  float w = 0;
  if (state == 1) { // leg back
    pos[0] -= dx * forward;
    w = -turn * dtheta;
  }
  else if (state == 3) { // leg forward
    pos[0] += dx * forward;
    w = turn * dtheta;
  }
  // turn
  float x = pos[0] * cos(w) - pos[1] * sin(w);
  float y = pos[0] * sin(w) + pos[1] * cos(w);
  pos[0] = x;
  pos[1] = y;
  for (int i = 0; i < 3; i++) {
    output[i] = pos[i];
  }
}

// Move all 3 servos of a leg to position the end effector
void Hexapod::moveLegToPosition(float x, float y, float z, int leg) {
  int angles[3];
  getAngles(x, y, z, leg, angles);
  if (angles[0] != -1) {
    moveLeg(angles, leg);
  }
}

// Determine servo angles (deg) from position relative to hexapod center (in)
// x: forward, y: left, z: up, leg: CCW starting with front left leg
void Hexapod::getAngles(float x, float y, float z, int leg, int* angles) {
  float a = 0;
  float b = 0;
  float c = 0;
  x = x - R * cos(leg * M_PI / 3 - M_PI / 6);
  y = y - R * sin(leg * M_PI / 3 - M_PI / 6);
  a = atan2(y, x) + M_PI / 6 - leg * M_PI / 3;
  float r = sqrt(pow(x, 2) + pow(y, 2)) - L1;
  if (a < -M_PI) {
    a = a + 2 * M_PI;
  }
  if (a < -M_PI / 2) {
    a = a + M_PI;
    r = -sqrt(pow(x, 2) + pow(y, 2)) - L1;
  }
  if (a > M_PI / 2) {
    a = a - M_PI;
    r = -sqrt(pow(x, 2) + pow(y, 2)) - L1;
  }
  if (r > L2 + L3 || pow(r, 2) + pow(z, 2) == 0) {
    Serial.println("Requested configuration out of reach");
    angles[0] = angles[1] = angles[2] = -1;
    return;
  }
  float r1 = (pow(r, 3) + r * pow(z, 2) + r * pow(L2, 2) - r * pow(L3, 2) - z * sqrt(-pow(r, 4) - pow(z, 4) - pow((pow(L2, 2) - pow(L3, 2)), 2) + 2 * pow(z, 2) *
              (pow(L2, 2) + pow(L3, 2)) + 2 * pow(r, 2) * (-pow(z, 2) + pow(L2, 2) + pow(L3, 2)))) / (2 * (pow(r, 2) + pow(z, 2)));
  float z1 = (pow(z, 3) + z * pow(r, 2) + z * pow(L2, 2) - z * pow(L3, 2) + r * sqrt(-pow(r, 4) - pow(z, 4) - pow((pow(L2, 2) - pow(L3, 2)), 2) + 2 * pow(z, 2) *
              (pow(L2, 2) + pow(L3, 2)) + 2 * pow(r, 2) * (-pow(z, 2) + pow(L2, 2) + pow(L3, 2)))) / (2 * (pow(r, 2) + pow(z, 2)));
  if (isnan(r1) || isnan(z1)) {
    Serial.println("Requested configuration has no solution");
    angles[0] = angles[1] = angles[2] = -1;
    return;
  }
  b = -atan2(z1, r1);
  c = -atan2(z - z1, r - r1) - b;
  angles[0] = round(180 / M_PI * a + 90);
  angles[1] = round(-180 / M_PI * b + 90);
  angles[2] = round(180 / M_PI * c);
  for (int i = 0; i < 3; i++) {
    if (angles[i] > maxLimits[i] || angles[i] < minLimits[i]) {
      Serial.print("Requested configuration outside servo range: Servo ");
      Serial.println(i);
      angles[0] = angles[1] = angles[2] = -1;
      return;
    }
  }
}

// Move all 3 servos of a leg to a given state
void Hexapod::moveLeg(int *angles, int leg) {
  if (LEGS_VERBOSE) {
    Serial.print("Leg ");
    Serial.print(leg);
    Serial.print(" moving to ");
    Serial.print(angles[0]);
    Serial.print(", ");
    Serial.print(angles[1]);
    Serial.print(", ");
    Serial.println(angles[2]);
  }
  for (int i = 0; i < 3; i++) {
    moveServo(angles[i], leg, i);
  }
}

// Move a specified servo to a given angle in degrees
void Hexapod::moveServo(int value, int leg, int servo) {
  if (value >= minLimits[servo] && value <= maxLimits[servo]) {
    int pulse = pulseLength(value - offsets[leg - 1][servo], leg, servo);
    if (servos[leg - 1][servo] < 16) {
      pwm1.setPWM(servos[leg - 1][servo], 0, pulse);
    } else {
      pwm2.setPWM(servos[leg - 1][servo] - 16, 0, pulse);
    }
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
int Hexapod::pulseLength(int angle, int leg, int servo) {
  int len = map(angle, ANGLEMIN, ANGLEMAX, SERVOMIN, SERVOMAX);
  if (leg > 3 && servo > 0) {
    len = map(180 - angle, ANGLEMIN, ANGLEMAX, SERVOMIN, SERVOMAX);
  }
  return (len);
}

// Determine {x,y,z} acceleration in m/s^2
void Hexapod::getAccel(float *acceleration) {
  if (!accelPresent) return;
  accel.read();
  sensors_event_t event;
  accel.getEvent(&event);
  acceleration[0] = event.acceleration.x;
  acceleration[1] = event.acceleration.y;
  acceleration[2] = event.acceleration.z;
  float avg_accel[] = {0,0,0};
  for(int i=0; i<3; i++) {
    for(int j=FILTER_LENGTH-1; j>=0; j--) {
      avg_accel[i] += pastAccel[j][i];
      if(j>0) pastAccel[j][i] = pastAccel[j-1][i];
      else pastAccel[0][i] = acceleration[i];
    }
    acceleration[i] = (avg_accel[i]+acceleration[i])/(FILTER_LENGTH+1);
  }
  if(ACCEL_VERBOSE) {
    Serial.print(acceleration[0]);
    Serial.print(", ");
    Serial.print(acceleration[1]);
    Serial.print(", ");
    Serial.println(acceleration[2]);
  }
}


//================================================================================== SENSE FUNCTIONS =====================================================================//

//Read the distance from the IR sensor
int Hexapod::sampleIR() {
  int goodvaluecount = 0;
  int goodvaluesum = 0;
  int tooclosecount = 0;
  int toofarcount = 0;
  

  //Take sensor values. "Good" values fall within IR range (10-80cm)
  for (int i=0; i<5; i++) {           //sample five times
    int distR = IR_r.getDistance();    //read right IR sensor
    if (distR >=80) {toofarcount++;}            //if the object is too far to read
    else if (distR <= 10) {tooclosecount++;}    //if the object is too close to read
    else {                                    //if the object is in the right range
      goodvaluesum += distR;
      goodvaluecount++;
      }
  }
  if (IR_VERBOSE) {
    Serial.print("The IR got "); Serial.print(goodvaluecount); Serial.println(" good readings (of 10)");
    Serial.print("The IR got "); Serial.print(tooclosecount); Serial.println(" of TOO CLOSE");
    Serial.print("The IR got "); Serial.print(toofarcount); Serial.println(" of TOO FAR"); 
  }

  // Decide what to output. If it shows "too close/far" more than twice, send that value. 
  if (tooclosecount > 2) {return 9;}
  else if (toofarcount > 2) {return 80;}
  else {
    int avgRval = goodvaluesum/goodvaluecount; //average the good values
    if (IR_VERBOSE) {Serial.print("The sensor value is:"); Serial.println(avgRval); }
    return avgRval;
  }
}
