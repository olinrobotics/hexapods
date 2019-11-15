#include "Arduino.h"
#include "Hexapod.h"

long stepStartTime = 0; // Tracks elapsed time for walking
bool accelPresent = false; // Accelerometer initialized successfully
float pastAccel[FILTER_LENGTH][3]; // Accelerometer moving average values
float footVals[6][3]; // Current x-y-z position of each foot
float footHeights[6]; // Current height of ground under each foot (in)
float *waypointsX; // Linear speed (-1 to 1), target x (in), delay (s)
float *waypointsY; // Angular speed (-1 to 1), target y (in), delay flag
int *waypointsN; // Waypoint type (N<=0), steps remaining (N>0)
int waypointLen = 0; // Number of waypoints
int waypointIndex = 0; // Current waypoint
float evenTripodYaw = 0; // Current yaw of even leg tripod relative to body
float oddTripodYaw = 0; // Current yaw of odd leg tripod relative to body
float netSlopeOffset = 0;
float ground = ground0;
float clearance = clearance0;

bool evenStep = true; // Whether the even-numbered legs are raised this step
long lastUpdate = 0;

// Initialize pin modes and servo shield
void Hexapod::init() {
  pwm1.begin();
  pwm2.begin();
  if (accel.begin(0x18)) {
    Serial.println("INIT");
    accelPresent = true;
    accel.setRange(LIS3DH_RANGE_4_G);
  }
  pinMode(relay, OUTPUT);
  for (int leg = 0; leg < 6; leg++) {
    pinMode(feet[leg], INPUT);
  }
  digitalWrite(relay, HIGH);
  pwm1.setPWMFreq(60);
  pwm2.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}

// Add a walk command to the end of the waypoint list
int Hexapod::addWalkSteps(float forward, float turn, int steps) {
  if (steps < 0) {
    return (addWaypoint(forward, turn, WAYPOINT_INFINITE));
  } else if (steps == 0) {
    return (addWaypoint(forward, turn, WAYPOINT_STAND));
  } else {
    return (addWaypoint(forward, turn, steps * 2));
  }
}

// Add a destination to the end of the waypoint list
int Hexapod::addDestination(float x, float y) {
  return (addWaypoint(x, y, WAYPOINT_DESTINATION));
}

// Add a delay command in seconds to the end of the waypoint list
int Hexapod::addDelay(float duration) {
  return (addWaypoint(duration * 1000, 0, WAYPOINT_DELAY));
}

// Add a new desired waypoint to the end of the list
int Hexapod::addWaypoint(float targetX, float targetY, int type) {
  float *tempX = malloc(sizeof(float) * (waypointLen + 1));
  float *tempY = malloc(sizeof(float) * (waypointLen + 1));
  int *tempN = malloc(sizeof(int) * (waypointLen + 1));
  for (int i = 0; i < waypointLen; i++) {
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
  return (waypointLen);
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
  if (waypointIndex >= waypointLen) {
    return 1;
  }
  if (waypointsN[waypointIndex] == WAYPOINT_STAND) {
    stand();
    waypointIndex++;
    return 1;
  } else if (waypointsN[waypointIndex] == WAYPOINT_SIT) {
    sit();
    waypointIndex++;
    return 1;
  } else if (waypointsN[waypointIndex] == WAYPOINT_INFINITE) {
    if (goTo(waypointsX[waypointIndex], 0, waypointsY[waypointIndex]) == -1) {
      return -1;
    }
    return 0;
  } else if (waypointsN[waypointIndex] == WAYPOINT_DESTINATION) {
    float deltaX = waypointsX[waypointIndex] - x;
    float deltaY = waypointsY[waypointIndex] - y;
    float deltaTheta = fmod(atan2(deltaY, deltaX) - theta + M_PI, 2 * M_PI) - M_PI;
    if (abs(sqrt(deltaX * deltaX + deltaY * deltaY)) <= DX / 2) {
      waypointIndex++;
      return 1;
    }
    //    if (abs(deltaTheta) >= DTHETA / 2) {
    //      return (walk(0, 0, deltaTheta > 0 ? 1 : -1) == -1 ? -1 : 0);
    //    } else {
    //      return (walk(1, 0, 0) == -1 ? -1 : 0);
    //    }
    return 0;
  } else if (waypointsN[waypointIndex] == WAYPOINT_DELAY) {
    if (waypointsY[waypointIndex] == 0) {
      waypointsY[waypointIndex] = 1; // Flag for timer start
      stepStartTime = millis();
    }
    if (millis() - stepStartTime > waypointsX[waypointIndex]) {
      waypointIndex++;
      return 1;
    }
    return 0;
  } else if (waypointsN[waypointIndex] > 0) {
    switch (goTo(waypointsX[waypointIndex], 0, waypointsY[waypointIndex])) {
      case -1: waypointsN[waypointIndex]++; return -1;
      case 0: return 0;
    }
    waypointsN[waypointIndex]--;
    if (waypointsN[waypointIndex] == 0) {
      waypointIndex++;
      return 1;
    } else {
      return 0;
    }
  }
}

// Increment a variable towards a setpoint, returning the displacement
float increment(float targetVal, float currentVal, float delta) {
  if (targetVal >= currentVal + delta) {
    return delta;
  } else if (targetVal <= currentVal - delta) {
    return -delta;
  } else {
    return targetVal - currentVal;
  }
}

// Called iteratively to move the hexapod in the given direction
bool Hexapod::walk(float vx, float vy, float vtheta) {
  // TODO: vary speed
  return goTo(x + vx, y + vy, theta + vtheta);
}

// Called iteratively to move the hexapod to the given coordinates
// Returns true if target position reached
bool Hexapod::goTo(float x2, float y2, float theta2) {
  // TODO: update waypoint code
  // TODO: change position on stand command
  
  Serial.print(x2);
  Serial.print(", ");
  Serial.print(y2);
  Serial.print(", ");
  Serial.print(theta2);
  Serial.print(";    ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.println(theta);
  // Determine time increment
  long dt = millis() - lastUpdate;
  lastUpdate = millis();
  if (dt > 200) { // time hasn't been updated recently
    return false;
  }
  if (FEET_VERBOSE) {
    Serial.print("Feet: ");
    for (int i = 0; i < 6; i++) {
      Serial.print(digitalRead(feet[i]));
    }
    Serial.println();
  }

  // Determine absolute orientation
  float a[3];
  float roll = 0;
  float pitch = 0;
  if (accelPresent) {
    getAccel(a);
    roll = -asin(a[1] / 9.81);
    pitch = asin(a[0] / 9.81);
    if (abs(pitch) > 15 * M_PI / 180) {
      pitch *= 15 * M_PI / 180 / abs(pitch);
    }
  }
  if(!DETECT_SLOPES) {
    pitch = 0;
    roll = 0;
  }
  // DANCE MODE (accelerometer directly controls orientation):
  //levelBody(-pitch/3, -roll/3); evenStep = !evenStep; levelBody(-pitch/3, -roll/3); return;

  // Determine current state of feet
  bool grounded = !digitalRead(feet[1]) && !digitalRead(feet[3]) && !digitalRead(feet[5]);
  if (digitalRead(feet[0]) && digitalRead(feet[2]) && digitalRead(feet[4])) {
    grounded = true;
  }
  bool lowered = (footHeights[1] <= ground || !digitalRead(feet[1])) &&
                 (footHeights[3] <= ground || !digitalRead(feet[3])) &&
                 (footHeights[5] <= ground || !digitalRead(feet[5]));
  bool raised = min(min(footHeights[0], footHeights[2]), footHeights[4]) >= ground + clearance;
  if (evenStep) {
    grounded = !digitalRead(feet[0]) && !digitalRead(feet[2]) && !digitalRead(feet[4]);
    if (digitalRead(feet[1]) && digitalRead(feet[3]) && digitalRead(feet[5])) {
      grounded = true;
    }
    lowered = (footHeights[0] <= ground || !digitalRead(feet[0])) &&
              (footHeights[2] <= ground || !digitalRead(feet[2])) &&
              (footHeights[4] <= ground || !digitalRead(feet[4]));
    raised = min(min(footHeights[1], footHeights[3]), footHeights[5]) >= ground + clearance;
  }
  grounded = true; // TODO: remove when bump sensors improve

  // Move feet during step transitions
  if (!lowered) { // Lowering legs
    // Adjust body height on steep slopes
    if (abs(pitch) > 10 * M_PI / 180) { // TODO: linear variation in clearance with angle
      ground = ground0 + 0.5;
      clearance = clearance0 - 0.5;
    } else {
      ground = ground0;
      clearance = clearance0;
    }
    if (!moveTripod(0, 0, -SPEED * dt, 0, !evenStep, true)) {
      return false;
    }
  }
  if (!grounded) { // Limit switches not triggered
    moveTripod(0, 0, SPEED * dt, 0, evenStep, false);
    moveTripod(0, 0, SPEED * dt, 0, !evenStep, true);
    return false;
  }
  if (!raised) { // Raising legs
    if (!moveTripod(0, 0, SPEED * dt, 0, evenStep, false)) {
      return false;
    }
  }
  if (!levelBody(0, 0)) { // Leveling body
    return false;
  }

  // Compute body displacement toward target state
  float slopeOffset = tan(pitch) * ground - netSlopeOffset;
  if (abs(slopeOffset) > SPEED * dt) {
    slopeOffset = copysignf(SPEED * dt, slopeOffset);
  }
  netSlopeOffset += slopeOffset;
  float dx = increment(x2, x, SPEED * dt);
  float dy = increment(y2, y, SPEED * dt);
  //  fmod(theta2 - theta + M_PI, 2 * M_PI) - M_PI; //TODO: theta modulus
  float dyaw = increment(theta2, theta, SPEED * dt / (R + roffset));
  x += dx * cos(theta) - dy * sin(theta);
  y += dx * sin(theta) + dy * cos(theta);
  theta += dyaw;
  if (LOCATION_VERBOSE) {
    Serial.print("x: ");
    Serial.print(x);
    Serial.print(", y: ");
    Serial.print(y);
    Serial.print(", theta: ");
    Serial.println(theta);
  }
  // Move feet during step
  bool swap = false;
  // Move grounded feet reverse of body motion
  swap = swap || moveTripod(-dx - slopeOffset, -dy, 0, -dyaw, !evenStep, false);
  // Move raised feet to mirror of grounded feet
  swap = swap || moveTripod(dx - slopeOffset, dy, 0, dyaw, evenStep, false);

  // Determine if step transition is required
  float cRaised[3];
  float cLowered[3];
  getCentroid(evenStep, cRaised);
  getCentroid(!evenStep, cLowered);
  float xMargin = (cRaised[0] - cLowered[0] + dx) / (2 * roffset * sqrt(3)); // forward stability margin
  float yMargin = (cRaised[1] - cLowered[1] + dy) / (2 * roffset * 0.5); // sideways stability margin
  bool xStable = (dx == 0) || (xMargin < STABILITY_MARGIN && dx > 0) || (-xMargin < STABILITY_MARGIN && dx < 0);
  bool yStable = (dy == 0) || (yMargin < STABILITY_MARGIN && dy > 0) || (-yMargin < STABILITY_MARGIN && dy < 0);
  // TODO: adjust y margin for even vs odd steps
  swap = swap || !xStable || !yStable; // exceed stability threshold
  swap = swap || evenStep && evenTripodYaw * copysignf(1.0, dyaw) > DTHETA || !evenStep && oddTripodYaw * copysignf(1.0, dyaw) > DTHETA;
  if (swap) { // switch feet
    evenStep = !evenStep;
    delay(100); // TODO: this is awful, set a delay variable somewhere instead
  }

  // Check for success
  if(abs(dx) < SPEED * dt && abs(dy) < SPEED * dt && abs(dyaw) < SPEED * dt / (R + roffset)) {
    return true;
  }
  return false;
}

// Incrementally move a foot triangle by given displacement, true if limit reached
// grounded prevents lowering a triggered foot or raising an untriggered foot
bool Hexapod::moveTripod(float dx, float dy, float dz, float dtheta, bool even, bool grounded) {
  if (even) evenTripodYaw += dtheta;
  else oddTripodYaw += dtheta;
  float pos[3];
  for (int leg = even ? 2 : 1; leg <= 6; leg += 2) {
    getCurrentPosition(leg, pos);
    pos[0] += dx;
    pos[1] += dy;
    if (!grounded || dz > 0 || digitalRead(feet[leg - 1])) { // leg not contacting ground
      if (!grounded || dz < 0 || !digitalRead(feet[leg - 1])) { // leg is in contact with ground
        pos[2] += dz;
      }
    }
    if (pos[2] > ground + clearance) { // leg raised too high
      pos[2] = ground + clearance;
    }
    if (pos[2] < ground) { // leg lowered too far
      pos[2] = ground;
    }
    pos[0] = -pos[1] * sin(dtheta) + pos[0] * cos(dtheta);
    pos[1] = pos[1] * cos(dtheta) + pos[0] * sin(dtheta);
    if (!isPositionValid(pos[0], pos[1], ground, leg)) { // check shadow is valid
      return true;
    }
    if (!moveLegToPosition(pos[0], pos[1], pos[2], leg)) {
      return true;
    }
    footHeights[leg - 1] = pos[2];
  }
  return false;
}

// Levels the hexapod body relative to the ground
// Returns true if level or position is unreachable
bool Hexapod::levelBody(float pitch, float roll) {
  float dx = footVals[0][0] - footVals[2][0]; // leg 1 to 3
  float dy = (footVals[0][1] + footVals[2][1]) / 2 - footVals[4][1]; // legs 1&3 to 5
  float p = atan2(footVals[0][2] - footVals[2][2], dx);
  float r = atan2((footVals[0][2] + footVals[2][2]) / 2 - footVals[4][2], dy);
  if (!evenStep) {
    dx = footVals[5][0] - footVals[3][0]; // leg 6 to 4
    dy = footVals[1][1] - (footVals[5][1] + footVals[3][1]) / 2; // leg 6&4 to 2
    p = atan2(footVals[5][2] - footVals[3][2], dx);
    r = atan2(footVals[1][2] - (footVals[5][2] + footVals[3][2]) / 2, dy);
  }
  float dzp = (tan(pitch) - tan(p)) * dx / 2;
  float dzr = (tan(roll) - tan(r)) * dy / 2;
  float dz[3];
  if (evenStep) { // odd legs grounded
    dz[0] = footVals[0][2] + dzp + dzr; // foot 1
    dz[1] = footVals[2][2] - dzp + dzr; // foot 3
    dz[2] = footVals[4][2] - dzr; // foot 5
  } else { // even legs grounded
    dz[0] = footVals[1][2] + dzr; // foot 2
    dz[1] = footVals[3][2] - dzp - dzr; // foot 4
    dz[2] = footVals[5][2] + dzp - dzr; // foot 6
  }
  float z0 = ground - min(min(dz[0], dz[1]), dz[2]);

  if (abs(dzp) + abs(dzr) < 0.1) {
    return true;
  }
  for (int i = 0; i < 3; i++) {
    int leg = !evenStep ? 2 * i + 2 : 2 * i + 1;
    if (!moveLegToPosition(footVals[leg - 1][0], footVals[leg - 1][1], z0 + dz[i], leg)) {
      Serial.println("Failed to level body");
      return true;
    }
  }
  delay(100); // TODO: this is awful, set a delay variable somewhere instead
  return false;
}

// Determine centroid of tripod
void Hexapod::getCentroid(bool even, float* centroid) {
  centroid[0] = centroid[1] = centroid[2] = 0;
  float pos[3];
  for (int leg = even ? 2 : 1; leg <= 6; leg += 2) {
    getCurrentPosition(leg, pos);
    for (int i = 0; i < 3; i++) {
      centroid[i] += pos[i] / 3;
    }
  }
}

// Adjust radius and angle offset of a raised foot triangle, 0 radius for no change
bool Hexapod::resizeTripod(float radius, float angle, bool even) {
  float centroid[3];
  getCentroid(even, centroid);
  float pos[3];
  for (int leg = even ? 2 : 1; leg <= 6; leg += 2) {
    getCurrentPosition(leg, pos);
    // Scale radius of tripod
    float r = sqrt((pos[0] - centroid[0]) * (pos[0] - centroid[0]) + (pos[1] - centroid[1]) * (pos[1] - centroid[1]));
    if (radius != 0) {
      pos[0] = (pos[0] - centroid[0]) * radius / r + centroid[0];
      pos[1] = (pos[1] - centroid[1]) * radius / r + centroid[1];
    }
    // Adjust angular spacing of tripod
    float theta = atan2(pos[1] - centroid[1], pos[0] - centroid[0]);
    float target = leg * M_PI / 3 - M_PI / 6;
    if (leg % 3 == 0) { // Legs 3 and 6 CCW
      target += angle;
    } else if (leg % 3 == 1) { // Legs 1 and 4 CW
      target -= angle;
    }
    float dtheta = target - theta;
    pos[0] = -pos[1] * sin(dtheta) + pos[0] * cos(dtheta);
    pos[1] = pos[1] * cos(dtheta) + pos[0] * sin(dtheta);

    // Move leg to new position
    if (!moveLegToPosition(pos[0], pos[1], pos[2], leg)) {
      return false;
    }
  }
}

// Resets a tripod to default stance, centered at (x0, y0, z0)
void Hexapod::resetTripod(float x0, float y0, float z0, bool even) {
  float r0 = R + roffset;
  for (int leg = even ? 2 : 1; leg <= 6; leg += 2) {
    float pos[] = {r0 * cos(leg*M_PI / 3 - M_PI / 6) + x0, r0 * sin(leg*M_PI / 3 - M_PI / 6) + y0, z0};
    moveLegToPosition(pos[0], pos[1], pos[2], leg);
  }
}

int mirrorLeg(int leg) {
  return (leg + 2) % 6 + 1;
}

// Lower the hexapod to the ground
void Hexapod::sit() {
  float dR = 7.2;
  float z0 = -1.26;
  for (int leg = 1; leg < 7; leg++) {
    float pos[] = {(R + dR)*cos(leg*M_PI / 3 - M_PI / 6), (R + dR)*sin(leg*M_PI / 3 - M_PI / 6), z0};
    moveLegToPosition(pos[0], pos[1], pos[2], leg);
  }
}

// Stand with all 6 legs on the ground
void Hexapod::stand() {
  resetTripod(0, 0, ground, false);
  resetTripod(0, 0, ground, true);
  netSlopeOffset = 0;
}

// Move all servos to 90 degrees
void Hexapod::testCalibration() {
  int angles[] = {90, 90, 90};
  for (int leg = 1; leg < 7; leg++) {
    moveLeg(angles, leg);
  }
}

// Move all 3 servos of a leg to position the end effector
bool Hexapod::moveLegToPosition(float x, float y, float z, int leg) {
  int angles[3];
  getAngles(x, y, z, leg, angles);
  if (angles[0] == -1) {
    return false;
  }
  moveLeg(angles, leg);
  footVals[leg - 1][0] = x;
  footVals[leg - 1][1] = y;
  footVals[leg - 1][2] = z;
  return true;
}

// Determine current leg position (in) relative to hexapod center
void Hexapod::getCurrentPosition(int leg, float* pos) {
  for (int i = 0; i < 3; i++) {
    pos[i] = footVals[leg - 1][i];
  }
}

// Determine whether leg can move to a given position
bool Hexapod::isPositionValid(float x, float y, float z, int leg) {
  int angles[3];
  getAngles(x, y, z, leg, angles);
  for (int i = 0; i < 3; i++) {
    if (angles[i] < minLimits[i] || angles[i] > maxLimits[i]) {
      return false;
    }
  }
  return true;
}

// Determine position (in) relative to hexapod center from servo angles (deg)
void Hexapod::getPosition(float a, float b, float c, int leg, float* pos) {
  a = M_PI / 180 * (a - 90);
  b = -M_PI / 180 * (b - 90);
  c = M_PI / 180 * c;
  pos[0] = L1 + L2 * cos(b) + L3 * cos(b + c);
  pos[2] = -L2 * sin(b) - L3 * sin(b + c);
  pos[1] = pos[0] * sin(leg * M_PI / 3 - M_PI / 6 + a) + R * sin(leg * M_PI / 3 - M_PI / 6);
  pos[0] = pos[0] * cos(leg * M_PI / 3 - M_PI / 6 + a) + R * cos(leg * M_PI / 3 - M_PI / 6);
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
  float avg_accel[] = {0, 0, 0};
  for (int i = 0; i < 3; i++) {
    for (int j = FILTER_LENGTH - 1; j >= 0; j--) {
      avg_accel[i] += pastAccel[j][i];
      if (j > 0) pastAccel[j][i] = pastAccel[j - 1][i];
      else pastAccel[0][i] = acceleration[i];
    }
    acceleration[i] = (avg_accel[i] + acceleration[i]) / (FILTER_LENGTH + 1);
  }
  if (ACCEL_VERBOSE) {
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
  for (int i = 0; i < 5; i++) {       //sample five times
    int distR = IR_r.getDistance();    //read right IR sensor
    if (distR >= 80) {
      toofarcount++; //if the object is too far to read
    }
    else if (distR <= 10) {
      tooclosecount++; //if the object is too close to read
    }
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
  if (tooclosecount > 2) {
    return 9;
  }
  else if (toofarcount > 2) {
    return 80;
  }
  else {
    int avgRval = goodvaluesum / goodvaluecount; //average the good values
    if (IR_VERBOSE) {
      Serial.print("The sensor value is:");
      Serial.println(avgRval);
    }
    return avgRval;
  }
}
