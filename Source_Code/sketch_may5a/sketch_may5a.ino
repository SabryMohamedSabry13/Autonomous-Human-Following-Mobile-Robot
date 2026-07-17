// ============================================
// AUTONOMOUS OBSTACLE AVOIDANCE
// PRIORITY-BASED VERSION v3
// ============================================

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

// ── Motor pins ──
#define IN1 11
#define IN2 13

#define ENA 10   // Right motor
#define ENB 12   // Left motor

// ── Thresholds ──
#define FRONT_THRESH  15
#define SIDE_THRESH   8

// ── Speeds ──
#define FWD_SPEED    30
#define TURN_SPEED   30
#define TURN_30      0.80
#define TURN_60      0.55
#define SIDE_NUDGE   0.35

// ============================================
// SENSOR
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

void readSensors(float &s1, float &s2, float &s3, float &s4, float &s5) {
  s1 = getDistance(TRIG_1, ECHO_1); delay(20);
  s2 = getDistance(TRIG_2, ECHO_2); delay(20);
  s3 = getDistance(TRIG_3, ECHO_3); delay(20);
  s4 = getDistance(TRIG_4, ECHO_4); delay(20);
  s5 = getDistance(TRIG_5, ECHO_5); delay(20);
}

// ============================================
// MOTOR
// ============================================

int currentPWM_R = 0;
int currentPWM_L = 0;

void setMotors(int rightPWM, int leftPWM, bool rightFwd, bool leftFwd) {
  currentPWM_R = rightPWM;
  currentPWM_L = leftPWM;
  analogWrite(ENA, rightPWM);
  analogWrite(ENB, leftPWM);
  digitalWrite(IN1, rightFwd ? HIGH : LOW);
  
  digitalWrite(IN2, leftFwd  ? LOW  : HIGH);
}

void printPWM() {
  Serial.printf("  PWM: R=%d L=%d\n", currentPWM_R, currentPWM_L);
}

void moveForward(int speed) {
  setMotors(speed, speed, true, true);
}

void stopMotors() {
  currentPWM_R = 0; currentPWM_L = 0;
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  
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
  setMotors(r, l, true, true);
}

void rotateRight(int speed) { setMotors(speed, speed, true,  false); }
void rotateLeft (int speed) { setMotors(speed, speed, false, true);  }

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

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  Serial.println("=== System Ready ===");
}

// ============================================
// MAIN LOOP
// ============================================

void loop() {

  float s1, s2, s3, s4, s5;
  readSensors(s1, s2, s3, s4, s5);

  bool b1 = s1 < FRONT_THRESH;
  bool b2 = s2 < FRONT_THRESH;
  bool b3 = s3 < FRONT_THRESH;
  bool b4 = s4 < SIDE_THRESH;
  bool b5 = s5 < SIDE_THRESH;

  Serial.println("─────────────────────────────");
  Serial.printf("S │ FL:%.1f FC:%.1f FR:%.1f L:%.1f R:%.1f\n", s1,s2,s3,s4,s5);
  Serial.printf("B │ FL:%d FC:%d FR:%d L:%d R:%d\n", b1,b2,b3,b4,b5);

  // ══════════════════════════════════════════
  // LEVEL 1 — CRITICAL
  // ══════════════════════════════════════════

  // C14: all blocked → stop
  if (b1 && b2 && b3 && b4 && b5) {
    stopMotors();
    Serial.println("C14: ALL BLOCKED — Stop");
    return;
  }

  // C12: 1+2+3+4 → rotate RIGHT until front clear
  if (b1 && b2 && b3 && b4 && !b5) {
    Serial.println("C12: Rotate RIGHT until clear");
    while (true) {
      rotateRight(TURN_SPEED);
      float f1, f2, f3, f4, f5;
      readSensors(f1, f2, f3, f4, f5);
      if (f1 > FRONT_THRESH && f2 > FRONT_THRESH && f3 > FRONT_THRESH) break;
      if (f1 < FRONT_THRESH && f2 < FRONT_THRESH && f3 < FRONT_THRESH &&
          f4 < SIDE_THRESH  && f5 < SIDE_THRESH) break;
    }
    stopMotors();
    return;
  }

  // C13: 1+2+3+5 → rotate LEFT until front clear
  if (b1 && b2 && b3 && !b4 && b5) {
    Serial.println("C13: Rotate LEFT until clear");
    while (true) {
      rotateLeft(TURN_SPEED);
      float f1, f2, f3, f4, f5;
      readSensors(f1, f2, f3, f4, f5);
      if (f1 > FRONT_THRESH && f2 > FRONT_THRESH && f3 > FRONT_THRESH) break;
      if (f1 < FRONT_THRESH && f2 < FRONT_THRESH && f3 < FRONT_THRESH &&
          f4 < SIDE_THRESH  && f5 < SIDE_THRESH) break;
    }
    stopMotors();
    return;
  }

  // C5: all 3 front blocked → stop
  if (b1 && b2 && b3) {
    stopMotors();
    Serial.println("C5: FRONT BLOCKED — Stop");
    printPWM();
    return;
  }

  // ══════════════════════════════════════════
  // LEVEL 2 — COMPLEX
  // ══════════════════════════════════════════

  // C10: 1+2+5 → stop, wait 3s, then decide
  if (b1 && b2 && !b3 && b5) {
    stopMotors();
    Serial.println("C10: 1+2+5 — Waiting 3s...");
    delay(3000);
    float f1, f2, f3, f4, f5;
    readSensors(f1, f2, f3, f4, f5);
    if (f1 < FRONT_THRESH && f2 < FRONT_THRESH && f5 < SIDE_THRESH) {
      if (f5 > 10) {
        Serial.println("C10: Nudge RIGHT");
        differentialTurn(TURN_30);
      } else {
        Serial.println("C10: Fallback LEFT");
        differentialTurn(-TURN_30);
      }
      printPWM();
      delay(400);
      stopMotors();
    }
    return;
  }

  // C11: 2+3+4 → stop, wait 3s, then decide
  if (!b1 && b2 && b3 && b4) {
    stopMotors();
    Serial.println("C11: 2+3+4 — Waiting 3s...");
    delay(3000);
    float f1, f2, f3, f4, f5;
    readSensors(f1, f2, f3, f4, f5);
    if (f2 < FRONT_THRESH && f3 < FRONT_THRESH && f4 < SIDE_THRESH) {
      if (f4 > 10) {
        Serial.println("C11: Nudge LEFT");
        differentialTurn(-TURN_30);
      } else {
        Serial.println("C11: Fallback RIGHT");
        differentialTurn(TURN_30);
      }
      printPWM();
      delay(400);
      stopMotors();
    }
    return;
  }

  // ══════════════════════════════════════════
  // LEVEL 3 — AVOIDANCE
  // ══════════════════════════════════════════

  // C2: 1+2 → turn RIGHT 60°
  if (b1 && b2 && !b3) {
    Serial.println("C2: 1+2 — Turn RIGHT 60°");
    differentialTurn(TURN_60);
    printPWM();
    return;
  }

  // C4: 2+3 → turn LEFT 60°
  if (!b1 && b2 && b3) {
    Serial.println("C4: 2+3 — Turn LEFT 60°");
    differentialTurn(-TURN_60);
    printPWM();
    return;
  }

  // C6: 1+3 → compare distances
  if (b1 && !b2 && b3) {
    if (s1 <= s3) {
      Serial.println("C6: 1+3 — More space RIGHT");
      differentialTurn(TURN_60);
    } else {
      Serial.println("C6: 1+3 — More space LEFT");
      differentialTurn(-TURN_60);
    }
    printPWM();
    return;
  }

  // C7: FC only → compare total side space
  if (!b1 && b2 && !b3) {
    float leftSpace  = s1 + s4;
    float rightSpace = s3 + s5;
    if (rightSpace >= leftSpace) {
      Serial.println("C7: FC — More space RIGHT");
      differentialTurn(TURN_60);
    } else {
      Serial.println("C7: FC — More space LEFT");
      differentialTurn(-TURN_60);
    }
    printPWM();
    return;
  }

  // C8: 1+5 → front-left + right-side → gentle RIGHT
  if (b1 && !b2 && !b3 && b5) {
    Serial.println("C8: 1+5 — Gentle RIGHT");
    differentialTurn(SIDE_NUDGE);
    printPWM();
    return;
  }

  // C9: 3+4 → front-right + left-side → gentle LEFT
  if (!b1 && !b2 && b3 && b4) {
    Serial.println("C9: 3+4 — Gentle LEFT");
    differentialTurn(-SIDE_NUDGE);
    printPWM();
    return;
  }

  // 1+4: front-left + left-side → nudge RIGHT
  if (b1 && !b2 && !b3 && b4) {
    Serial.println("1+4: Both LEFT — Nudge RIGHT");
    differentialTurn(TURN_30);
    printPWM();
    return;
  }

  // 3+5: front-right + right-side → nudge LEFT
  if (!b1 && !b2 && b3 && b5) {
    Serial.println("3+5: Both RIGHT — Nudge LEFT");
    differentialTurn(-TURN_30);
    printPWM();
    return;
  }

  // ══════════════════════════════════════════
  // LEVEL 4 — EARLY WARNING
  // ══════════════════════════════════════════

  // C1: FL only → turn RIGHT 30°
  if (b1 && !b2 && !b3) {
    Serial.println("C1: FL only — Turn RIGHT 30°");
    differentialTurn(TURN_30);
    printPWM();
    return;
  }

  // C3: FR only → turn LEFT 30°
  if (!b1 && !b2 && b3) {
    Serial.println("C3: FR only — Turn LEFT 30°");
    differentialTurn(-TURN_30);
    printPWM();
    return;
  }

  // ══════════════════════════════════════════
  // DEFAULT — FORWARD
  // ══════════════════════════════════════════
  Serial.println("FORWARD");
  moveForward(FWD_SPEED);
  printPWM();
}