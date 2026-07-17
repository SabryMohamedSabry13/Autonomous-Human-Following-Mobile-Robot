#include "HUSKYLENS.h"
#include "Wire.h"

HUSKYLENS huskylens;

// ── Sensor pins ──
#define TRIG_1 6    // Front Left
#define ECHO_1 7
#define TRIG_2 4    // Front Center
#define ECHO_2 5
#define TRIG_3 15   // Front Right
#define ECHO_3 16
#define TRIG_4 17   // Left Side
#define ECHO_4 18
#define TRIG_5 46   // Right Side
#define ECHO_5 3

// ── Throttle pins → PWMV modules ──
#define THROTTLE_R  10
#define THROTTLE_L  12

// ── PWM config ──
#define PWM_FREQ       500
#define PWM_RESOLUTION 8
#define PWM_MAX        255

// ── Obstacle thresholds ──
#define FRONT_THRESH  60   // cm — front sensors
#define SIDE_THRESH   40   // cm — side sensors

// ── Speed settings ──
#define MIN_SPIN     30    // 1.18V — minimum to spin
#define FWD_SPEED    60    // 2.94V — cruise (shared by both modes)
#define TURN_SPEED   45    // 2.35V — pivot speed
#define SPD_SLOW     35    // 1.76V — slow approach (HuskyLens medium)
#define SPD_FAST     60    // 3.14V — fast approach (HuskyLens far)

// ── Obstacle avoidance differential factors ──
#define TURN_30      0.25
#define TURN_60      0.45
#define SIDE_NUDGE   0.12

// ── HuskyLens settings ──
#define SCREEN_CX    160   // horizontal center of 320px frame
#define AREA_FAR     3000  // below = far from color
#define AREA_CLOSE   11000 // above = too close to color
#define KP           0.01   // steering proportional gain

// ── State tracking ──
int currentPWM_R = 0;
int currentPWM_L = 0;

// ── Mode indicator ──
enum RobotMode { MODE_OBSTACLE, MODE_TRACKING, MODE_LOST };
RobotMode currentMode = MODE_LOST;

// ============================================
// SENSOR — ULTRASONIC
// ============================================

float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 999;
  return duration * 0.034 / 2.0;
}

void readSensors(float &s1, float &s2, float &s3,
                 float &s4, float &s5) {
  s1 = getDistance(TRIG_1, ECHO_1); delay(15);
  s2 = getDistance(TRIG_2, ECHO_2); delay(15);
  s3 = getDistance(TRIG_3, ECHO_3); delay(15);
  s4 = getDistance(TRIG_4, ECHO_4); delay(15);
  s5 = getDistance(TRIG_5, ECHO_5); delay(15);
}

// ============================================
// MOTOR CONTROL
// ============================================

void setMotors(int pwmR, int pwmL) {
  if (pwmR > 0) pwmR = constrain(pwmR, MIN_SPIN, PWM_MAX);
  if (pwmL > 0) pwmL = constrain(pwmL, MIN_SPIN, PWM_MAX);
  currentPWM_R = pwmR;
  currentPWM_L = pwmL;
  ledcWrite(THROTTLE_R, pwmR);
  ledcWrite(THROTTLE_L, pwmL);
}

void stopMotors() {
  currentPWM_R = 0;
  currentPWM_L = 0;
  ledcWrite(THROTTLE_R, 0);
  ledcWrite(THROTTLE_L, 0);
}

void moveForward(int speed) {
  setMotors(speed, speed);
}

void differentialTurn(float factor) {
  int r, l;
  if (factor > 0) {
    l = FWD_SPEED;
    r = FWD_SPEED * (1.0 - factor);
  } else {
    r = FWD_SPEED;
    l = FWD_SPEED * (1.0 + factor);
  }
  setMotors(r, l);
}

void rotateRight(int speed) {
  int s = constrain(speed, MIN_SPIN, PWM_MAX);
  currentPWM_R = 0; currentPWM_L = s;
  ledcWrite(THROTTLE_R, 0);
  ledcWrite(THROTTLE_L, s);
}

void rotateLeft(int speed) {
  int s = constrain(speed, MIN_SPIN, PWM_MAX);
  currentPWM_R = s; currentPWM_L = 0;
  ledcWrite(THROTTLE_R, s);
  ledcWrite(THROTTLE_L, 0);
}

void printPWM() {
  float vR = (currentPWM_R / 255.0) * 10.0;
  float vL = (currentPWM_L / 255.0) * 10.0;
  Serial.printf("  PWM R:%d(%.2fV) L:%d(%.2fV)\n",
                currentPWM_R, vR, currentPWM_L, vL);
}

// ============================================
// OBSTACLE AVOIDANCE — returns true if active
// ============================================

bool handleObstacles(float s1, float s2, float s3,
                     float s4, float s5) {

  bool b1 = s1 < FRONT_THRESH;
  bool b2 = s2 < FRONT_THRESH;
  bool b3 = s3 < FRONT_THRESH;
  bool b4 = s4 < SIDE_THRESH;
  bool b5 = s5 < SIDE_THRESH;

  // No obstacles — let tracking take over
  if (!b1 && !b2 && !b3 && !b4 && !b5) return false;

  currentMode = MODE_OBSTACLE;
  Serial.printf("S │ FL:%.0f FC:%.0f FR:%.0f L:%.0f R:%.0f\n",
                s1, s2, s3, s4, s5);

  // ── LEVEL 1 — CRITICAL ──

  if (b1 && b2 && b3 && b4 && b5) {
    stopMotors();
    Serial.println("C14: ALL BLOCKED — Stop");
    return true;
  }

  if (b1 && b2 && b3 && b4 && !b5) {
    Serial.println("C12: Pivot RIGHT");
    while (true) {
      rotateRight(TURN_SPEED);
      float f1,f2,f3,f4,f5;
      readSensors(f1,f2,f3,f4,f5);
      if (f1>FRONT_THRESH && f2>FRONT_THRESH && f3>FRONT_THRESH) break;
      if (f1<FRONT_THRESH && f2<FRONT_THRESH && f3<FRONT_THRESH
          && f4<SIDE_THRESH && f5<SIDE_THRESH) break;
    }
    stopMotors(); return true;
  }

  if (b1 && b2 && b3 && !b4 && b5) {
    Serial.println("C13: Pivot LEFT");
    while (true) {
      rotateLeft(TURN_SPEED);
      float f1,f2,f3,f4,f5;
      readSensors(f1,f2,f3,f4,f5);
      if (f1>FRONT_THRESH && f2>FRONT_THRESH && f3>FRONT_THRESH) break;
      if (f1<FRONT_THRESH && f2<FRONT_THRESH && f3<FRONT_THRESH
          && f4<SIDE_THRESH && f5<SIDE_THRESH) break;
    }
    stopMotors(); return true;
  }

  if (b1 && b2 && b3) {
    stopMotors();
    Serial.println("C5: FRONT BLOCKED");
    return true;
  }

  // ── LEVEL 2 — COMPLEX ──

  if (b1 && b2 && !b3 && b5) {
    stopMotors();
    Serial.println("C10: Waiting 3s...");
    delay(3000);
    float f1,f2,f3,f4,f5;
    readSensors(f1,f2,f3,f4,f5);
    if (f1<FRONT_THRESH && f2<FRONT_THRESH && f5<SIDE_THRESH) {
      if (f5 > 10) differentialTurn(TURN_30);
      else         differentialTurn(-TURN_30);
      delay(400); stopMotors();
    }
    return true;
  }

  if (!b1 && b2 && b3 && b4) {
    stopMotors();
    Serial.println("C11: Waiting 3s...");
    delay(3000);
    float f1,f2,f3,f4,f5;
    readSensors(f1,f2,f3,f4,f5);
    if (f2<FRONT_THRESH && f3<FRONT_THRESH && f4<SIDE_THRESH) {
      if (f4 > 10) differentialTurn(-TURN_30);
      else         differentialTurn(TURN_30);
      delay(400); stopMotors();
    }
    return true;
  }

  // ── LEVEL 3 — AVOIDANCE ──

  if (b1 && b2 && !b3)  { differentialTurn(TURN_60);   Serial.println("C2: RIGHT 60°");  printPWM(); return true; }
  if (!b1 && b2 && b3)  { differentialTurn(-TURN_60);  Serial.println("C4: LEFT 60°");   printPWM(); return true; }

  if (b1 && !b2 && b3) {
    if (s1 <= s3) differentialTurn(TURN_60);
    else          differentialTurn(-TURN_60);
    Serial.println("C6: space compare"); printPWM(); return true;
  }

  if (!b1 && b2 && !b3) {
    float lSpace = s1+s4, rSpace = s3+s5;
    if (rSpace >= lSpace) differentialTurn(TURN_60);
    else                  differentialTurn(-TURN_60);
    Serial.println("C7: FC steer"); printPWM(); return true;
  }

  if (b1 && !b2 && !b3 && b5)  { differentialTurn(SIDE_NUDGE);  Serial.println("C8: nudge R"); printPWM(); return true; }
  if (!b1 && !b2 && b3 && b4)  { differentialTurn(-SIDE_NUDGE); Serial.println("C9: nudge L"); printPWM(); return true; }
  if (b1 && !b2 && !b3 && b4)  { differentialTurn(TURN_30);     Serial.println("1+4: R");      printPWM(); return true; }
  if (!b1 && !b2 && b3 && b5)  { differentialTurn(-TURN_30);    Serial.println("3+5: L");      printPWM(); return true; }

  // ── LEVEL 4 — EARLY WARNING ──

  if (b1 && !b2 && !b3) { differentialTurn(TURN_30);  Serial.println("C1: FL warn R"); printPWM(); return true; }
  if (!b1 && !b2 && b3) { differentialTurn(-TURN_30); Serial.println("C3: FR warn L"); printPWM(); return true; }

  return false;  // no condition matched
}

// ============================================
// HUSKYLENS COLOR TRACKING
// ============================================

void handleTracking() {

  if (!huskylens.request()) {
    Serial.println("[Husky] No response");
    stopMotors();
    currentMode = MODE_LOST;
    return;
  }

  if (!huskylens.available()) {
    Serial.println("[Husky] Color not in frame");
    stopMotors();
    currentMode = MODE_LOST;
    return;
  }

  HUSKYLENSResult result = huskylens.read();
  int x    = result.xCenter;
  int y    = result.yCenter;
  int area = result.width * result.height;

  int   error = x - SCREEN_CX;
  int   turn  = (int)(error * KP);

  currentMode = MODE_TRACKING;
  Serial.printf("[Husky] X:%d Y:%d Area:%d\n", x, y, area);

  if (area < AREA_FAR) {
    // Far — move faster
    int L = constrain(SPD_FAST - turn, MIN_SPIN, PWM_MAX);
    int R = constrain(SPD_FAST + turn, MIN_SPIN, PWM_MAX);
    setMotors(R, L);
    Serial.println("[Husky] FAR → fast approach");
    printPWM();
  }
  else if (area > AREA_CLOSE) {
    // Too close — hold
    stopMotors();
    Serial.println("[Husky] TOO CLOSE → hold");
  }
  else {
    // Medium — slow approach
    int L = constrain(SPD_SLOW - turn, MIN_SPIN, PWM_MAX);
    int R = constrain(SPD_SLOW + turn, MIN_SPIN, PWM_MAX);
    setMotors(R, L);
    Serial.println("[Husky] MEDIUM → slow approach");
    printPWM();
  }
}

// ============================================
// SETUP
// ============================================

void setup() {
  Serial.begin(115200);

  // Ultrasonic sensor pins
  pinMode(TRIG_1,OUTPUT); pinMode(ECHO_1,INPUT);
  pinMode(TRIG_2,OUTPUT); pinMode(ECHO_2,INPUT);
  pinMode(TRIG_3,OUTPUT); pinMode(ECHO_3,INPUT);
  pinMode(TRIG_4,OUTPUT); pinMode(ECHO_4,INPUT);
  pinMode(TRIG_5,OUTPUT); pinMode(ECHO_5,INPUT);

  // PWMV throttle pins
  ledcAttach(THROTTLE_R, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(THROTTLE_L, PWM_FREQ, PWM_RESOLUTION);

  // HuskyLens I2C
  Wire.begin(8, 9);

  // Safe start
  stopMotors();

  Serial.println("=== Merged System Starting ===");

  if (!huskylens.begin(Wire)) {
    Serial.println("HuskyLens NOT found — check wiring");
    while (1);
  }
  huskylens.writeAlgorithm(ALGORITHM_COLOR_RECOGNITION);

  Serial.println("=== System Ready ===");
  Serial.println("Priority: Obstacle Avoidance > Color Tracking");
}

// ============================================
// MAIN LOOP
// ============================================

void loop() {

  // ── Step 1: read all 5 ultrasonic sensors ──
  float s1, s2, s3, s4, s5;
  readSensors(s1, s2, s3, s4, s5);

  // ── Step 2: obstacle check — highest priority ──
  if (handleObstacles(s1, s2, s3, s4, s5)) {
    // obstacle avoidance acted — skip tracking this cycle
    delay(30);
    return;
  }

  // ── Step 3: path is clear — do color tracking ──
  handleTracking();

  delay(40);
}