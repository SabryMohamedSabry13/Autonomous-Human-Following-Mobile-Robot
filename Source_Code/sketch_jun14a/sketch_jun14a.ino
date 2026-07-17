// ============================================
// AUTONOMOUS CART — MERGED FINAL
// Mode 1: BLE Manual Control
// Mode 2: HuskyLens + Ultrasonic Auto
// E-Bike Controller
// ============================================

#include "HUSKYLENS.h"
#include "Wire.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

HUSKYLENS huskylens;

// ── BLE UUIDs ──
#define SERVICE_UUID   "12345678-1234-1234-1234-1234567890ab"
#define CHAR_UUID_RX   "12345678-1234-1234-1234-1234567890ac"

// ── Sensor pins ──
#define TRIG_1 6    // Left side
#define ECHO_1 7
#define TRIG_2 4    // Front Left
#define ECHO_2 5
#define TRIG_3 15   // Front Center
#define ECHO_3 16
#define TRIG_4 17   // Front Right
#define ECHO_4 18
#define TRIG_5 46   // Right side
#define ECHO_5 3

// ── Motor pins ──
#define PWM_R 10
#define PWM_L 12

// ── Thresholds ──
#define CRIT_DIST     20
#define STOP_FRONT    30
#define TURN_THRESH   50
#define SLOW_THRESH   70
#define SIDE_BLOCK    10

// ── Speeds ──
#define SPD_FAST      26
#define SPD_SLOW      25
#define SPD_MIN       25
#define RAMP_STEP     4

// ── Camera ──
#define CAM_CENTER_X  160
#define CAM_DEAD_L    140
#define CAM_DEAD_R    180
#define AREA_CLOSE    11000
#define AREA_FAR      2000
#define SEARCH_TIME   5000

// ── Mode ──
enum Mode { MODE_MANUAL, MODE_AUTO };
volatile Mode currentMode = MODE_MANUAL;

// ── Last seen direction ──
enum LastDir { DIR_NONE, DIR_LEFT, DIR_RIGHT, DIR_CENTER };
LastDir lastDir  = DIR_NONE;
unsigned long lostTime = 0;
bool personLost  = false;

// ── Serial print throttle ──
String lastPrint = "";
void printOnce(const char* msg) {
  if (String(msg) != lastPrint) {
    Serial.println(msg);
    lastPrint = String(msg);
  }
}

// ============================================
// MOTOR
// ============================================

int currentR = 0, currentL = 0;
int targetR  = 0, targetL  = 0;

void rampMotors() {
  if (currentR < targetR) currentR = min(currentR + RAMP_STEP, targetR);
  else if (currentR > targetR) currentR = max(currentR - RAMP_STEP, targetR);
  if (currentL < targetL) currentL = min(currentL + RAMP_STEP, targetL);
  else if (currentL > targetL) currentL = max(currentL - RAMP_STEP, targetL);
  ledcWrite(PWM_R, currentR);
  ledcWrite(PWM_L, currentL);
}

void setMotors(int r, int l) {
  targetR = (r <= 0) ? 0 : constrain(r, SPD_MIN, SPD_FAST);
  targetL = (l <= 0) ? 0 : constrain(l, SPD_MIN, SPD_FAST);
}

void stopMotors() {
  targetR = 0; targetL = 0;
  currentR = 0; currentL = 0;
  ledcWrite(PWM_R, 0);
  ledcWrite(PWM_L, 0);
}

// ============================================
// SENSOR
// ============================================

float getDistance(int trigPin, int echoPin) {
  float readings[3];
  for (int i = 0; i < 3; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(4);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH, 25000);
    readings[i] = (duration == 0) ? 300.0 : duration * 0.034 / 2.0;
    delayMicroseconds(300);
  }
  if (readings[0]>readings[1]){float t=readings[0];readings[0]=readings[1];readings[1]=t;}
  if (readings[1]>readings[2]){float t=readings[1];readings[1]=readings[2];readings[2]=t;}
  if (readings[0]>readings[1]){float t=readings[0];readings[0]=readings[1];readings[1]=t;}
  return readings[1];
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
// BLE CALLBACK
// ============================================

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String v = c->getValue();
    if (v.length() == 0) return;
    char cmd = v[0];

    switch (cmd) {

      // ── Mode switching ──
      case 'A':
        currentMode = MODE_AUTO;
        Serial.println(">>> MODE: AUTO");
        break;

      case 'M':
        currentMode = MODE_MANUAL;
        stopMotors();
        Serial.println(">>> MODE: MANUAL");
        break;

      // ── Manual commands (only in MANUAL mode) ──
      case '0':
        if (currentMode == MODE_MANUAL) {
          stopMotors();
          Serial.println(">>> STOP");
        }
        break;

      case '1':
        if (currentMode == MODE_MANUAL) {
          setMotors(SPD_FAST, SPD_FAST);
          Serial.println(">>> FORWARD");
        }
        break;

      case '3':
        // Right → PWM_R faster, PWM_L slower
        if (currentMode == MODE_MANUAL) {
          setMotors(SPD_FAST, 0);
          Serial.println(">>> RIGHT");
        }
        break;

      case '2':
        // Left → PWM_L faster, PWM_R slower
        if (currentMode == MODE_MANUAL) {
          setMotors(0, SPD_FAST);
          Serial.println(">>> LEFT");
        }
        break;

      default:
        Serial.printf(">>> Unknown cmd: %c\n", cmd);
        break;
    }
  }
};

// ============================================
// OBSTACLE AVOIDANCE
// ============================================

bool handleObstacles(float s1, float s2, float s3,
                     float s4, float s5) {

  if (s1<CRIT_DIST||s2<CRIT_DIST||s3<CRIT_DIST||
      s4<CRIT_DIST||s5<CRIT_DIST) {
    stopMotors(); printOnce("CRITICAL STOP"); return true;
  }

  if ((s2<STOP_FRONT&&s4<STOP_FRONT)||s3<STOP_FRONT) {
    stopMotors(); printOnce("FRONT STOP"); return true;
  }

  bool fl = s2 < TURN_THRESH;
  bool fc = s3 < TURN_THRESH;
  bool fr = s4 < TURN_THRESH;
  bool lb = s1 < SIDE_BLOCK;
  bool rb = s5 < SIDE_BLOCK;

  if (!fl && !fc && !fr) return false;

  if (fl && fc && !fr) {
    if (rb) { stopMotors(); printOnce("STOP: want R s5 critical"); }
    else if (s5<STOP_FRONT) { setMotors(25,0); }
    else { setMotors(30,0); }
    return true;
  }
  if (fl && !fc && !fr) {
    if (rb) { stopMotors(); }
    else if (s5<STOP_FRONT) { setMotors(30,28); }
    else { setMotors(30,25); }
    return true;
  }
  if (!fl && fc && fr) {
    if (lb) { stopMotors(); printOnce("STOP: want L s1 critical"); }
    else if (s1<STOP_FRONT) { setMotors(0,25); }
    else { setMotors(0,30); }
    return true;
  }
  if (!fl && !fc && fr) {
    if (lb) { stopMotors(); }
    else if (s1<STOP_FRONT) { setMotors(28,30); }
    else { setMotors(25,30); }
    return true;
  }
  if (fl && !fc && fr) {
    if (s2<=s4) { if(rb)stopMotors(); else setMotors(30,25); }
    else        { if(lb)stopMotors(); else setMotors(25,30); }
    return true;
  }
  if (!fl && fc && !fr) {
    float ls=s1+s2, rs=s4+s5;
    if (rs>=ls) { if(rb)stopMotors(); else setMotors(30,25); }
    else        { if(lb)stopMotors(); else setMotors(25,30); }
    return true;
  }

  stopMotors(); printOnce("ALL FRONT BLOCKED"); return true;
}

// ============================================
// CAMERA FOLLOWING
// Mirrors fixed: person right → cart turns right
// ============================================

void handleCamera(float s1, float s2, float s3,
                  float s4, float s5) {

  float frontMin = min({s2, s3, s4});
  int baseSpeed  = (frontMin < SLOW_THRESH) ? SPD_SLOW : SPD_FAST;

  if (!huskylens.request()) {
    printOnce("CAM: no response");
    stopMotors(); return;
  }

  if (!huskylens.available()) {
    if (!personLost) {
      personLost = true;
      lostTime   = millis();
      Serial.println("CAM: person lost");
    }
    unsigned long elapsed = millis() - lostTime;
    if (elapsed > SEARCH_TIME) {
      stopMotors(); printOnce("CAM: timeout STOP"); return;
    }
    // Search toward last seen direction
    if (lastDir == DIR_LEFT)       { setMotors(25, 30); }
    else if (lastDir == DIR_RIGHT) { setMotors(30, 25); }
    else                           { stopMotors(); }
    rampMotors(); return;
  }

  // Person detected
  personLost = false;
  HUSKYLENSResult result = huskylens.read();
  int x    = result.xCenter;
  int area = result.width * result.height;

  // Update last direction
  if (x < CAM_DEAD_L)      lastDir = DIR_LEFT;
  else if (x > CAM_DEAD_R) lastDir = DIR_RIGHT;
  else                      lastDir = DIR_CENTER;

  // Too close → stop
  if (area > AREA_CLOSE) {
    stopMotors(); printOnce("CAM: too close"); return;
  }

  // ── FIXED MIRROR: positive error → person is RIGHT → turn RIGHT ──
  // Previously: R = base + turn (wrong direction)
  // Now:        R = base - turn, L = base + turn → correct
  float Kp  = 0.08;
  int error = x - CAM_CENTER_X;          // positive = person right of center
  int turn  = (int)(error * Kp * baseSpeed);

  // person right → turn right → R slower, L faster
  int r = constrain(baseSpeed - turn, 0, SPD_FAST);
  int l = constrain(baseSpeed + turn, 0, SPD_FAST);

  if (area < AREA_FAR) {
    r = SPD_FAST; l = SPD_FAST;
  }

  setMotors(r, l);
  rampMotors();
}

// ============================================
// SETUP
// ============================================

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_1, OUTPUT); pinMode(ECHO_1, INPUT);
  pinMode(TRIG_2, OUTPUT); pinMode(ECHO_2, INPUT);
  pinMode(TRIG_3, OUTPUT); pinMode(ECHO_3, INPUT);
  pinMode(TRIG_4, OUTPUT); pinMode(ECHO_4, INPUT);
  pinMode(TRIG_5, OUTPUT); pinMode(ECHO_5, INPUT);

  ledcAttach(PWM_R, 5000, 8);
  ledcAttach(PWM_L, 5000, 8);
  stopMotors();

  Wire.begin(8, 9);
  delay(500);
  if (!huskylens.begin(Wire)) {
    Serial.println("HuskyLens not found!");
    while (1);
  }

  // BLE
  BLEDevice::init("ESP32-CART");
  BLEServer*     server  = BLEDevice::createServer();
  BLEService*    service = server->createService(SERVICE_UUID);
  BLECharacteristic* rxChar = service->createCharacteristic(
    CHAR_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR
  );
  rxChar->setCallbacks(new CommandCallbacks());
  service->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->start();

  Serial.println("=== CART READY ===");
  Serial.println("BLE: ESP32-CART");
  Serial.println("Send M=Manual  A=Auto");
  Serial.println("Manual: 0=Stop 1=Fwd 2=Right 4=Left");
}

// ============================================
// MAIN LOOP
// ============================================

void loop() {

  if (currentMode == MODE_MANUAL) {
    // In manual mode — BLE callback handles motors
    // Just apply ramp every loop
    rampMotors();
    return;
  }

  // AUTO mode
  float s1, s2, s3, s4, s5;
  readSensors(s1, s2, s3, s4, s5);

  bool handled = handleObstacles(s1, s2, s3, s4, s5);

  if (!handled) {
    handleCamera(s1, s2, s3, s4, s5);
  } else {
    rampMotors();
  }
}