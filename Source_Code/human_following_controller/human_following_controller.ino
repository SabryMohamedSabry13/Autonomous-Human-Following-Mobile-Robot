// ============================================
// AUTONOMOUS HUMAN-FOLLOWING MOBILE ROBOT
// Differential Drive — Two-Mode Controller
// Mode 1: Linear Motion  (straight forward)
// Mode 2: Rotational     (vision-based steering)
// ──────────────────────────────────────────
// DUAL E-BIKE CONTROLLER — ZWK3648-6HS3 × 2
// PWMV MODULE: PWM 0–255 → 0–10V Throttle
// GPIO10 = Right motor throttle
// GPIO12 = Left  motor throttle
// ESP32 Arduino Core v3.0+
// ============================================

#include "HUSKYLENS.h"
#include "Wire.h"

HUSKYLENS huskylens;

// ============================================
// HARDWARE PINS
// ============================================

#define THROTTLE_R  12   // → PWMV #1 → Right controller
#define THROTTLE_L  10   // → PWMV #2 → Left  controller

// ── PWM config ──
#define PWM_FREQ       500   // Hz — reliable for PWMV module
#define PWM_RESOLUTION 8     // 8-bit: 0–255
#define PWM_MAX        255

// ── Voltage reference ──
#define MIN_SPIN  30

// ============================================
// FRAME GEOMETRY
// ============================================

#define FRAME_W     320   // HuskyLens frame width  (px)
#define FRAME_H     240   // HuskyLens frame height (px)
#define FRAME_CX    160   // horizontal center = FRAME_W / 2
#define FRAME_CY    120   // vertical   center = FRAME_H / 2

// ============================================
// CONTROL PARAMETERS
// ============================================

// ── Reference linear velocity (V_ref) ──
#define V_REF       60    // 2.35V — moderate cruise

// ── Angular velocity gain (Kω) ──
// Scales normalised image error → ω (PWM units)
// ω = Kx * ex  +  Ky * ey
// where ex, ey are normalised errors (−1.0 to +1.0)
//
// Kx — horizontal error gain (dominant steering axis)
// Ky — vertical error gain   (depth correction: pull forward
//       when person is high in frame = far, slow when low = close)
#define Kx   40.0f   // px: max ex=1.0 → ω contribution = ±40 PWM
#define Ky   15.0f   // supplementary depth assist

// ── Dead-band ──
// Normalised error below this threshold → no rotation command
// Prevents micro-oscillations when centred
#define DEAD_BAND   0.05f   // ±5% of half-frame

// ── Area thresholds (bounding box area in px²) ──
// Used only in Rotational Mode to gate forward motion
#define AREA_FAR    2000    // below → target far,  keep moving
#define AREA_CLOSE  9000   // above → target close, hold position

// ── Scan / reacquire ──
#define SCAN_TIMEOUT  80    // cycles ≈ 4s at 50ms/cycle
#define SCAN_SPEED    35    // PWM for slow pivot scan

// ============================================
// RUNTIME STATE
// ============================================

// Operating mode
enum Mode { LINEAR, ROTATIONAL };
Mode currentMode = LINEAR;

// Last known steering error — sets scan direction on target loss
float lastEx = 0.0f;

// Scan cycle counter
int scanTimer = 0;

// ============================================
// MOTOR LAYER
// ============================================

// Low-level write — enforces dead-zone floor on non-zero commands
void setMotors(int pwmR, int pwmL) {
  if (pwmR > 0) pwmR = constrain(pwmR, MIN_SPIN, PWM_MAX);
  if (pwmL > 0) pwmL = constrain(pwmL, MIN_SPIN, PWM_MAX);
  ledcWrite(THROTTLE_R, pwmR);
  ledcWrite(THROTTLE_L, pwmL);
  Serial.printf("  [Motors] R:%3d (%.2fV)  L:%3d (%.2fV)\n",
    pwmR, pwmR / 255.0f * 10.0f,
    pwmL, pwmL / 100.0f * 10.0f);
}

// True zero — bypasses floor clamp
void stopMotors() {
  ledcWrite(THROTTLE_R, 0);
  ledcWrite(THROTTLE_L, 0);
  Serial.println("  [Motors] STOP — 0V both");
}

// Pivot in place — one wheel drives, one stops
void pivotRight(int speed) { setMotors(speed, 0); }
void pivotLeft (int speed) { setMotors(0, speed); }

// Hold position when too close — no reverse on BLDC
void holdPosition() {
  stopMotors();
  delay(200);
  // Creep at minimum to keep lock; remove for hard stop
  setMotors(MIN_SPIN, MIN_SPIN);
  Serial.println("  [Motors] TOO CLOSE — hold + creep");
}

// ============================================
// MODE 1 — LINEAR MOTION
// ============================================
//
// Both wheels receive V_ref.
// Robot moves in a straight line.
// No vision input required.
// Used as the default / fallback mode.
//
void linearMode() {
  setMotors(V_REF, V_REF);
  Serial.printf("  [LINEAR] V_ref=%d → both wheels\n", V_REF);
}

// ============================================
// MODE 2 — ROTATIONAL MOTION
// ============================================
//
// Differential drive steered by camera error.
//
// Inputs (from HuskyLens bounding box):
//   xc  — x-centre of detected object  (0–320 px)
//   yc  — y-centre of detected object  (0–240 px)
//   bw  — bounding box width           (px)
//   bh  — bounding box height          (px)
//
// Error normalisation:
//   ex = (xc − FRAME_CX) / (FRAME_W / 2)    ∈ [−1, +1]
//   ey = (yc − FRAME_CY) / (FRAME_H / 2)    ∈ [−1, +1]
//
//   ex > 0 → target right of centre → turn right
//   ex < 0 → target left  of centre → turn left
//   ey > 0 → target low in frame    → target close → slow
//   ey < 0 → target high in frame   → target far   → speed up
//
// Angular velocity:
//   ω = Kx * ex  +  Ky * ey
//
// Wheel velocities:
//   V_left  = V_ref − ω
//   V_right = V_ref + ω
//
void rotationalMode(int xc, int yc, int bw, int bh) {

  int area = bw * bh;

  // ── Guard: target too close ──
  if (area > AREA_CLOSE) {
    holdPosition();
    return;
  }

  // ── Normalise errors to [−1, +1] ──
  float ex = (float)(xc - FRAME_CX) / (FRAME_W / 2.0f);
  float ey = (float)(yc - FRAME_CY) / (FRAME_H / 2.0f);

  // Save horizontal error for scan direction
  lastEx = ex;

  Serial.printf("  [ROT] xc:%d yc:%d  ex:%.3f  ey:%.3f  area:%d\n",
                xc, yc, ex, ey, area);

  // ── Dead-band: suppress micro-corrections ──
  if (fabsf(ex) < DEAD_BAND) ex = 0.0f;
  if (fabsf(ey) < DEAD_BAND) ey = 0.0f;

  // ── Angular velocity command ──
  // ω combines horizontal error (steering) and vertical error (depth assist)
  float omega = Kx * ex + Ky * ey;

  // ── Wheel velocity equations ──
  //   V_left  = V_ref − ω
  //   V_right = V_ref + ω
  int vLeft  = (int)(V_REF - omega);
  int vRight = (int)(V_REF + omega);

  Serial.printf("  [ROT] ω:%.1f  V_L:%d  V_R:%d\n", omega, vLeft, vRight);

  // ── Area-based forward gate ──
  // Target far → allow full speed from V_ref
  // (area already checked for AREA_CLOSE above)
  if (area < AREA_FAR) {
    // Target far — boost base by scaling up proportionally
    // Multiply both by 1.4 to close the gap faster, keep ratio intact
    vLeft  = (int)(vLeft  * 1.4f);
    vRight = (int)(vRight * 1.4f);
    Serial.println("  [ROT] FAR — speed boost active");
  }

  // ── Apply with dead-zone clamp ──
  // Negative values (from large ω) → treated as 0 → wheel stops = pivot
  if (vLeft  < 0) vLeft  = 0;
  if (vRight < 0) vRight = 0;

        if (vRight > vLeft)  { vRight=60; } 
  else if (vRight < vLeft) { vLeft=55; }

    setMotors(vRight, vLeft);   // setMotors(R, L)
}

// ============================================
// SCAN — reacquire lost target
// ============================================

void scanForTarget() {
  scanTimer++;
  Serial.printf("  [SCAN] cycle %d/%d  lastEx:%.2f\n",
                scanTimer, SCAN_TIMEOUT, lastEx);

  if (scanTimer >= SCAN_TIMEOUT) {
    stopMotors();
    Serial.println("  [SCAN] Timeout — stopped. Re-learn target if needed.");
    return;
  }

  // Rotate toward last known direction
  if (lastEx >= 0) {
    pivotRight(SCAN_SPEED);
    Serial.println("  [SCAN] Pivot RIGHT");
  } else {
    pivotLeft(SCAN_SPEED);
    Serial.println("  [SCAN] Pivot LEFT");
  }
}

// ============================================
// SETUP
// ============================================

void setup() {
  Serial.begin(115200);

  Wire.begin(8, 9);   // SDA=GPIO8  SCL=GPIO9

  ledcAttach(THROTTLE_R, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(THROTTLE_L, PWM_FREQ, PWM_RESOLUTION);

  stopMotors();

  Serial.println("╔══════════════════════════════════════╗");
  Serial.println("║  Human-Following Robot — Ready       ║");
  Serial.println("║  Mode 1: LINEAR  (no target)         ║");
  Serial.println("║  Mode 2: ROTATIONAL (target locked)  ║");
  Serial.println("╚══════════════════════════════════════╝");

  if (!huskylens.begin(Wire)) {
    Serial.println("[ERROR] HuskyLens not found — check wiring");
    while (1);
  }

  huskylens.writeAlgorithm(ALGORITHM_OBJECT_TRACKING);
  Serial.println("[OK] HuskyLens → Object Tracking");
  Serial.println("[OK] Point at person → press learn button → robot follows");
}

// ============================================
// MAIN LOOP
// ============================================

void loop() {

  // ── I2C communication check ──
  if (!huskylens.request()) {
    Serial.println("[ERR] HuskyLens I2C fail — stopping");
    stopMotors();
    delay(100);
    return;
  }

  // ── No target detected ──
  if (!huskylens.available()) {

    if (scanTimer == 0) {
      // First lost frame — switch to LINEAR briefly before scanning
      // Keeps robot moving forward rather than freezing instantly
      currentMode = LINEAR;
      Serial.println("[MODE] Target lost → LINEAR (1 cycle)");
      linearMode();
    } else {
      // Subsequent lost frames → scan
      currentMode = ROTATIONAL;   // mode label only; scan handles motion
      scanForTarget();
    }

    scanTimer = max(scanTimer, 1);   // ensure scan increments next cycle
    delay(50);
    return;
  }

  // ── Target detected — reset scan, enter rotational mode ──
  scanTimer = 0;
  currentMode = ROTATIONAL;

  HUSKYLENSResult result = huskylens.read();

  int xc = result.xCenter;
  int yc = result.yCenter;
  int bw = result.width;
  int bh = result.height;

  Serial.printf("[DETECT] xc:%d  yc:%d  w:%d  h:%d  area:%d\n",
                xc, yc, bw, bh, bw * bh);
  Serial.println("[MODE] ROTATIONAL");

  rotationalMode(xc, yc, bw, bh);

  delay(50);
}
