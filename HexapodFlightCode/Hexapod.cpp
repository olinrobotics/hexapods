#include "Arduino.h"
#include "Hexapod.h"

long stepStartTime = 0; // Tracks elapsed time for walking
long lastServoUpdate = 0; // Tracks elapsed time for servo updates
int counter = 0; // Tracks current walking state
bool feetDown = true; // Tracks if a step is currently in progress
bool accelPresent = false; // Accelerometer initialized successfully
float pastAccel[FILTER_LENGTH][3]; // Accelerometer moving average values
float footVals[6][3]; // Current x-y-z position of each foot
float footTargets[6][3]; // Current target position of each foot
float footHeights[6]; // Current height of foot above the ground (in)
float *waypointsX; // Linear speed (-1 to 1), target x (in), delay (s)
float *waypointsY; // Angular speed (-1 to 1), target y (in), delay flag
int *waypointsN; // Waypoint type (N<=0), steps remaining (N>0)
int waypointLen = 0; // Number of waypoints
int waypointIndex = 0; // Current waypoint
float evenTripodYaw = 0; // Current yaw of even leg tripod relative to body
float oddTripodYaw = 0; // Current yaw of odd leg tripod relative to body

bool evenStep = true; // Whether the even-numbered legs are raised this step
long lastUpdate = 0;

// Initialize pin modes and servo shield
void Hexapod::init() {
  pwm1.begin();
  pwm2.begin();
  if (accel.begin(0x18)) {
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
    if (walk(waypointsX[waypointIndex], 0, waypointsY[waypointIndex]) == -1) {
      return -1;
    }
    return 0;
  } else if (waypointsN[waypointIndex] == WAYPOINT_DESTINATION) {
    float deltaX = waypointsX[waypointIndex] - x;
    float deltaY = waypointsY[waypointIndex] - y;
    float deltaTheta = fmod(atan2(deltaY, deltaX) - theta + M_PI, 2 * M_PI) - M_PI;
    if (abs(sqrt(deltaX * deltaX + deltaY * deltaY)) <= dx / 2) {
      waypointIndex++;
      return 1;
    }
    if (abs(deltaTheta) >= dtheta / 2) {
      return (walk(0, 0, deltaTheta > 0 ? 1 : -1) == -1 ? -1 : 0);
    } else {
      return (walk(1, 0, 0) == -1 ? -1 : 0);
    }
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
    switch (walk(waypointsX[waypointIndex], 0, waypointsY[waypointIndex])) {
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

// Called iteratively to walk with given linear and angular velocities
// Returns 1 if step taken, -1 if obstacle encountered, 0 otherwise
int Hexapod::walk(float forward, float left, float turn) {
  bool complete = true;
  if (accelPresent) {
    float a[3];
    getAccel(a);
  }
  if (ROUGH_TERRAIN) {
    if (!feetDown) { // Stop legs if they hit the ground
      for (int leg = 1; leg <= 6; leg++) {
        if (digitalRead(feet[leg - 1]) == LOW && getLegState(leg, counter) == 3) {
          float pos[3];
          getCurrentPosition(leg, pos);
          footHeights[leg - 1] = pos[2];
          moveLegToPositionSmooth(footVals[leg - 1][0], footVals[leg - 1][1], footHeights[leg - 1], leg);
        }
      }
      feetDown = updateServos();
      Serial.println("Lowering feet");
      return 0;
    } else if (counter % 3 == 0) { // Shift body up or down
      if (!balance()) {
        Serial.println("Balancing");
        return false;
      }
    }
  } else {
    updateServos();
  }
  if (millis() - stepStartTime > stepDuration) { // Take new step
    stepStartTime = millis();
    if (DETECT_WALLS && sampleIR() < 12 && forward > 0) {
      Serial.println("Object is too close! Stopping!");
      return -1;
    }
    if (DETECT_CLIFFS && ((getLegState(1, counter) == 4 && digitalRead(feet[0]) != LOW) ||
                          (getLegState(6, counter) == 4 && digitalRead(feet[5]) != LOW))) {
      step(forward, left, turn, counter - 2);
      x -= 2 * forward * dx * cos(theta);
      y -= 2 * forward * dx * sin(theta);
      theta -= 2 * turn * dtheta;
      counter--;
      if (counter < 0) counter += 6;
      Serial.println("My feet aren't on the ground!");
      Serial.print("Foot 1 is: "); Serial.println(digitalRead(feet[0]));
      Serial.print("Foot 6 is: "); Serial.println(digitalRead(feet[5]));
      return -1;
    }
    counter++;
    step(forward, left, turn, counter);
    return 1;
  }
  return 0;
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

// Called iteratively to move the hexapod to the given coordinates
// Returns true if target position reached
bool Hexapod::goTo(float x2, float y2, float theta2) {
  long dt = millis() - lastUpdate;
  lastUpdate = millis();
  if (dt > 200) { // time hasn't been updated recently
    return false;
  }
    Serial.print("Feet: ");
    for(int i=0; i<6; i++) {
      Serial.print(digitalRead(feet[i]));
    }
  // Raise or lower feet
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
  if (!lowered) { // Lowering legs
    moveTripod(0, 0, -SPEED * dt, 0, !evenStep, true);
    Serial.println(", lower");
    return false;
  } else if (!grounded) { // Limit switches not triggered
    moveTripod(0, 0, SPEED * dt, 0, evenStep, false);
    moveTripod(0, 0, SPEED * dt, 0, !evenStep, true);
    Serial.println(", ground");
    return false;
  } else if (!raised) { // Raising legs
    moveTripod(0, 0, SPEED * dt, 0, evenStep, false);
    Serial.println(", raise");
    return false;
  } else if (!levelBody(0, 0)) { // Leveling body
    return false;
  }
  // Move body coordinates toward target coordinates
  // TODO: adjust position increment for body pitch
  float dx = increment(x2, x, SPEED * dt);
  float dy = increment(y2, y, SPEED * dt);
  float dyaw = increment(theta2, theta, SPEED * dt / (R + roffset));
  x += dx * cos(theta);
  y += dy * sin(theta);
  theta += dyaw;

  bool swap = false;
  // Move grounded feet reverse of body motion
  swap = swap || moveTripod(-dx, -dy, 0, -dyaw, !evenStep, false);
  // Move raised feet to mirror of grounded feet
  swap = swap || moveTripod(dx, dy, 0, dyaw, evenStep, false);
  Serial.println(swap);
  float c1[3];
  float c2[3];
  getCentroid(evenStep, c1); // raised
  getCentroid(!evenStep, c2); // grounded
  float xMargin = (c1[0] - c2[0] + dx*2) / (2 * roffset * sqrt(3)); // forward stability margin
  float yMargin = (c1[1] - c2[1] + dy*2) / (2 * roffset * 0.5); // sideways stability margin
  swap = swap || max(abs(xMargin), abs(yMargin)) > STABILITY_MARGIN; // exceed stability threshold
  // TODO: adjust for slope, change y margin every other step
  swap = swap || evenStep && evenTripodYaw * copysignf(1.0, dyaw) > dtheta || !evenStep && oddTripodYaw * copysignf(1.0, dyaw) > dtheta;
  if (swap) { // switch feet
    Serial.println("SWAP");
    evenStep = !evenStep;
    delay(100); // TODO: this is awful, set a delay variable somewhere instead
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
      if(!grounded || dz < 0 || !digitalRead(feet[leg-1])) { // leg is in contact with ground
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
    if (!moveLegToPosition(pos[0], pos[1], pos[2], leg)) {
      return true;
    }
    footHeights[leg - 1] = pos[2];
  }
  return false;
}

// Levels the hexapod body relative to the ground
// Returns false if position is unreachable
bool Hexapod::levelBody(float pitch, float roll) {
//  return true; // TODO: test
  float dx = footVals[0][0] - footVals[2][0]; // leg 1 to 3
  float dy = (footVals[0][1] + footVals[2][1]) / 2 - footVals[4][1]; // legs 1&3 to 5
  float p = atan2(footVals[0][2] - footVals[2][2], dx);
  float r = atan2((footVals[0][2] + footVals[2][2]) / 2 - footVals[4][2], dy);
  if(evenStep){
    Serial.print(", dx: ");
    Serial.print(dx);
    Serial.print(", dy: ");
    Serial.print(dy);
    Serial.print(", dz p: ");
    Serial.print(footVals[0][2] - footVals[2][2]);
    Serial.print(", dz r: ");
    Serial.println((footVals[0][2] + footVals[2][2]) / 2 - footVals[4][2]);
  }
  if (!evenStep) {
    dx = footVals[5][0] - footVals[3][0]; // leg 6 to 4
    dy = footVals[1][1] - (footVals[5][1] + footVals[3][1]) / 2; // leg 6&4 to 2
    p = atan2(footVals[5][2] - footVals[3][2], dx);
    r = atan2(footVals[1][2] - (footVals[5][2] + footVals[3][2]) / 2, dy);
    Serial.print(", dx: ");
    Serial.print(dx);
    Serial.print(", dy: ");
    Serial.print(dy);
    Serial.print(", dz p: ");
    Serial.print(footVals[5][2] - footVals[3][2]);
    Serial.print(", dz r: ");
    Serial.println(footVals[1][2] - (footVals[5][2] + footVals[3][2]) / 2);
  }
  float dzp = (tan(pitch) - tan(p)) * dx/2;
  float dzr = (tan(roll) - tan(r)) * dy/2;
  Serial.print(", p: ");
  Serial.print(p);
  Serial.print(", r: ");
  Serial.println(r);
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
  Serial.print(z0);
  Serial.print(dzp);
  Serial.print(dzr);
  if (abs(dzp) + abs(dzr) < 0.1) {
    Serial.print(", already level; ");
    return true;
  }
  for (int i = 0; i < 3; i++) {
    int leg = !evenStep ? 2 * i + 2 : 2 * i + 1;
    if (!moveLegToPosition(footVals[leg - 1][0], footVals[leg - 1][1], z0 + dz[i], leg)) {
      Serial.println("Failed to level body");
      return false;
    }
  }
  Serial.print(", leveling");
  delay(100); // TODO: this is awful, set a delay variable somewhere instead
  return true;
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

// Adjust radius and angle offset of a raised foot triangle
bool Hexapod::resizeTripod(float radius, float angle, bool even) {
  float centroid[3];
  getCentroid(even, centroid);
  float pos[3];
  for (int leg = even ? 2 : 1; leg <= 6; leg += 2) {
    getCurrentPosition(leg, pos);

    // Scale radius of tripod
    float r = (pos[0] - centroid[0]) * (pos[0] - centroid[0]) + (pos[1] - centroid[1]) * (pos[1] - centroid[1]);
    pos[0] = (pos[0] - centroid[0]) * radius / r + centroid[0];
    pos[1] = (pos[1] - centroid[1]) * radius / r + centroid[1];

    // Adjust angular spacing of tripod
    float theta = atan2(pos[1] - centroid[1], pos[0] - centroid[0]);
    float target = leg * M_PI / 3 - M_PI / 6;
    if (leg % 3 == 0) { // Legs 3 and 6 CCW
      target += angle;
    } else if (leg % 3 == 1) { // Legs 1 and 5 CW
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

int mirrorLeg(int leg) {
  return (leg + 2) % 6 + 1;
}

// Called iteratively to lower feet to the ground and level out hexapod body
// Returns true if complete
bool Hexapod::balance() {
  int contacts = 0;
  for (int leg = 1; leg <= 6; leg++) {
    if (digitalRead(feet[leg - 1]) == LOW && getLegState(leg, counter) == 3) {
      contacts++;
    }
  }
  if (contacts == 3) { // Raise body
    translateBody(0, 0, 0.1);
    float front[3];
    float back[3];
    float mid[3];
    if ((counter % 4) / 2 == 1) {
      getCurrentPosition(1, front);
      getCurrentPosition(5, mid);
      getCurrentPosition(3, back);
    } else {
      getCurrentPosition(6, front);
      getCurrentPosition(2, mid);
      getCurrentPosition(4, back);
    }
    float pitch = -atan2(front[2] - back[2], front[0] - back[0]) - .1;
    float roll = atan2(back[2] + front[2] - 2 * mid[2], back[1] + front[1] - 2 * mid[1]);
    if ((counter % 4) / 2 == 0) roll *= -1;
    Serial.println(pitch);
    Serial.println(roll);
    if (pitch > 0.02) rotateBody(0, -0.01, 0);
    else if (pitch < -0.02) rotateBody(0, 0.01, 0);
    if (roll > 0.02) rotateBody(-0.01, 0, 0);
    else if (roll < -0.02) rotateBody(0.01, 0, 0);
  } else { // Lower body
    translateBody(0, 0, -0.1);
  }
  return updateServos();
}

// Move legs into the next configuration of a walking gait
void Hexapod::step(float forward, float left, float turn, int counter) {
  x += forward * dx * cos(theta) - left * dy * sin(theta);
  y += forward * dx * sin(theta) + left * dy * cos(theta);
  theta += turn * dtheta;
  feetDown = false;
  if (LOCATION_VERBOSE) {
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.println(theta);
  }
  for (int leg = 1; leg < 7; leg++) {
    if (leg % 2 == 0) { // right side
      moveLegToState(leg, counter % 4, forward, left, turn);
    } else {
      moveLegToState(leg, (counter + 2) % 4, forward, left, turn);
    }
  }
}

// Lower the hexapod to the ground
void Hexapod::sit() {
  for (int leg = 1; leg < 7; leg++) {
    float dR = 5.3;
    float z0 = -0.6;
    float pos[3] = {(R + dR)*cos(leg*M_PI / 3 - M_PI / 6), (R + dR)*sin(leg*M_PI / 3 - M_PI / 6), z0};
    moveLegToPositionSmooth(pos[0], pos[1], pos[2], leg);
  }
  moveServosDirect();
  counter = 0;
}

// Stand with all 6 legs on the ground
void Hexapod::stand() {
  for (int leg = 1; leg < 7; leg++) {
    moveLegToState(leg, 0, 0, 0, 0);
  }
  moveServosDirect();
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
void Hexapod::moveLegToState(int leg, int state, float forward, float left, float turn) {
  float pos[3];
  getLegPosition(leg, state, forward, left, turn, pos);
  moveLegToPositionSmooth(pos[0], pos[1], pos[2], leg);
}

// Generate a position vector for a given state of a leg
// Legs numbered 1-6 CCW from front left
// States numbered 1-4 back-up-forward-down
// forward and turn specify movement direction between -1 and 1
// Resulting position is stored in output
void Hexapod::getLegPosition(int leg, int state, float forward, float left, float turn, float* output) {
  float r0 = R + roffset;
  float pos[] = {r0 * cos(leg*M_PI / 3 - M_PI / 6), r0 * sin(leg*M_PI / 3 - M_PI / 6), ground};
  if (leg % 2 == 0) {
    state = state + 3;
  }
  state = state % 6;
  if (state == 1 || state == 2) { // leg lifted
    pos[2] += clearance;
  }
  if (ROUGH_TERRAIN) {
    if (state == 3) { // leg down
      footHeights[leg - 1] = pos[2];
    } else if (state != 1 && state != 2 && footHeights[leg - 1] != 0) { // leg on ground
      pos[2] = footHeights[leg - 1];
    }
  }
  //  if (leg <= 3) { // left side
  //    pos[1] += yoffset;
  //  } else {
  //    pos[1] -= yoffset;
  //  }
  float w = 0;
  if (state == 1) { // leg back
    pos[0] -= dx * forward;
    pos[1] -= dy * left;
    w = -turn * dtheta;
  }
  else if (state == 3) { // leg forward
    pos[0] += dx * forward;
    pos[1] += dy * left;
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

// States numbered 1-6 (up, forward, down, down, back, back)
int Hexapod::getLegState(int leg, int counter) {
  if (leg % 2 == 1) { // Odd
    return (counter % 6);
  } else {
    return ((counter + 3) % 6);
  }
}

// Displaces the hexapod body by a set amount relative to the ground
void Hexapod::translateBody(float dx, float dy, float dz) {
  float minHeight = ground + clearance;
  float maxHeight = ground;
  for (int leg = 1; leg <= 6; leg++) {
    if (getLegState(leg, counter) == 3) {
      minHeight = min(footHeights[leg - 1], minHeight);
      maxHeight = max(footHeights[leg - 1], maxHeight);
      if (footHeights[leg - 1] == 0) {
        return;
      }
    }
  }
  dz = min(minHeight - ground, dz); // Apply lower limit
  dz = max(maxHeight - (ground + clearance / 2), dz); // Apply upper limit
  float pos[3];
  for (int leg = 1; leg <= 6; leg++) {
    if (digitalRead(feet[leg - 1]) == LOW && counter % 3 == 0) {
      getCurrentPosition(leg, pos);
      moveLegToPositionSmooth(pos[0] - dx, pos[1] - dy, pos[2] - dz, leg);
      footHeights[leg - 1] = pos[2] - dz;
    }
  }
}

// Rotates the hexapod body by a set amount relative to the ground
void Hexapod::rotateBody(float droll, float dpitch, float dyaw) {
  float pos[3];
  for (int leg = 1; leg <= 6; leg++) {
    if (!ROUGH_TERRAIN || digitalRead(feet[leg - 1]) == LOW) {
      getCurrentPosition(leg, pos);
      pos[1] = -pos[2] * sin(droll) + pos[1] * cos(droll);
      pos[2] = pos[2] * cos(droll) + pos[1] * sin(droll);
      pos[2] = -pos[0] * sin(dpitch) + pos[2] * cos(dpitch);
      pos[0] = pos[0] * cos(dpitch) + pos[2] * sin(dpitch);
      pos[0] = -pos[1] * sin(dyaw) + pos[0] * cos(dyaw);
      pos[1] = pos[1] * cos(dyaw) + pos[0] * sin(dyaw);
      moveLegToPositionSmooth(pos[0], pos[1], pos[2], leg);
      footHeights[leg - 1] = pos[2];
    }
  }
}

// Move all 3 servos of a leg to position the end effector
void Hexapod::moveLegToPositionSmooth(float x, float y, float z, int leg) {
  footTargets[leg - 1][0] = x;
  footTargets[leg - 1][1] = y;
  footTargets[leg - 1][2] = z;
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

// Incrementally move a specified servo to a given angle in degrees
void Hexapod::moveServoSmooth(int value, int leg, int servo) {
  //  if (value >= minLimits[servo] && value <= maxLimits[servo]) {
  //    if (footVals[leg - 1][servo] == 0) servoVals[leg - 1][servo] = value;
  //    footTargets[leg - 1][servo] = value;
  //  } else if (value != -1) {
  //    Serial.print("Servo ");
  //    Serial.print(labels[servo]);
  //    Serial.print(" out of bounds: please enter a value between ");
  //    Serial.print(minLimits[servo]);
  //    Serial.print(" and ");
  //    Serial.println(maxLimits[servo]);
  //  }
}

// Sets leg target position to its current position
void Hexapod::stopLeg(int leg) {
  for (int i = 0; i < 3; i++) {
    footTargets[leg - 1][i] = footVals[leg - 1][i];
  }
}

// Directly move all servos to their target angles
void Hexapod::moveServosDirect() {
  for (int leg = 0; leg < 6; leg++) {
    for (int i = 0; i < 3; i++) {
      if (footTargets[leg][i] != 0) { // target exists
        footVals[leg][i] = footTargets[leg][i];
      }
    }
    moveLegToPosition(footVals[leg][0], footVals[leg][1], footVals[leg][2], leg + 1);
  }
}

// Increments servos toward desired positions
bool Hexapod::updateServos() {
  long dt = millis() - lastServoUpdate;
  bool complete = true;
  lastServoUpdate += dt;
  for (int leg = 0; leg < 6; leg++) {
    for (int i = 0; i < 3; i++) {
      float error = footTargets[leg][i] - footVals[leg][i];
      if (footVals[leg][i] == 0) {
        footVals[leg][i] = footTargets[leg][i];
      }
      if (footTargets[leg][i] != 0) { // target exists
        if (abs(error) < servoSpeed * dt / 1000) { // reached target
          footVals[leg][i] = footTargets[leg][i];
        } else {
          complete = false;
          if (error > 0) {
            footVals[leg][i] += servoSpeed * dt / 1000;
          } else {
            footVals[leg][i] -= servoSpeed * dt / 1000;
          }
        }
      }
    }
    moveLegToPosition(footVals[leg][0], footVals[leg][1], footVals[leg][2], leg + 1);
  }
  return complete;
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
