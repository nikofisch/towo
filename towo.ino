// TCS3200 Color + Line Following (Single Sensor)
// Arduino Uno/Nano (AVR) - No external libs

#include <Arduino.h>

// =======================
// Pin Definitions (MUST MATCH)
// =======================
// Ultrasonic (stub only)
#define TRIG_PIN 8
#define ECHO_PIN 9

// TCS3200
#define S0_PIN 4
#define S1_PIN 5
#define S2_PIN 6
#define S3_PIN 7
#define OUT_PIN 12

// L298N Motor Driver
#define ENA_PIN A0
#define IN1_PIN A1
#define IN2_PIN A2
#define ENB_PIN A3
#define IN3_PIN 13
#define IN4_PIN A4

// =======================
// Tuning / Calibration Constants
// =======================
// Tuning order (using DEBUG boot prints):
// 1) Adjust BLACK_CLEAR_PULSE_MIN_US / WHITE_CLEAR_PULSE_MAX_US using RAW C.
// 2) Set TARGET_* using NORM R/G/B when the sensor is centered on each tape.
// 3) Adjust COLOR_DIST_MAX / TARGET_DIST_MAX / DOMINANCE_RATIO for robustness.
// TCS3200 sampling
#define COLOR_SAMPLES 5
#define PULSE_TIMEOUT_US 30000UL  // pulseIn timeout

// Normalization
#define INTENSITY_MAX 255

// Brightness thresholds using CLEAR pulse width (tune these first)
// NOTE: TCS3200 outputs shorter pulses for brighter surfaces.
// Black = high pulse width, White = low pulse width.
#define BLACK_CLEAR_PULSE_MIN_US 1800  // >= this is considered BLACK (dark)
#define WHITE_CLEAR_PULSE_MAX_US 450   // <= this is considered WHITE (bright)
#define BRIGHTNESS_OK_MIN_US (WHITE_CLEAR_PULSE_MAX_US + 1)
#define BRIGHTNESS_OK_MAX_US (BLACK_CLEAR_PULSE_MIN_US - 1)

// Calibrated target RGB (normalized) for tape colors (tune these after brightness)
#define TARGET_RED_R 210
#define TARGET_RED_G 30
#define TARGET_RED_B 15

#define TARGET_GREEN_R 30
#define TARGET_GREEN_G 210
#define TARGET_GREEN_B 30

#define TARGET_BLUE_R 25
#define TARGET_BLUE_G 40
#define TARGET_BLUE_B 210

// Classification thresholds (tune after target RGB)
#define DOMINANCE_RATIO 1.25f      // dominant / second must exceed this
#define COLOR_DIST_MAX 140         // max distance to a target color
#define TARGET_DIST_MAX 170        // line-follow confidence threshold

// BLUE debounce
#define BLUE_DEBOUNCE_MS 150
#define BLUE_PAUSE_MS 500

// Line following params (tune these)
#define BASE_SPEED 130
#define TURN_SPEED 70
#define WIGGLE_PERIOD_MS 250
#define LOST_TIMEOUT_MS 500
#define RECOVERY_ROTATE_SPEED 90
#define RECOVERY_FULL_TIMEOUT_MS 1500

// Debug mode at boot
#define DEBUG_BOOT_MS 5000
#define DEBUG_PERIOD_MS 100

// =======================
// Enums / State Machine
// =======================
enum ColorTag { C_UNKNOWN, C_BLACK, C_WHITE, C_RED, C_GREEN, C_BLUE };

enum RobotState { START, OBSTACLE_COURSE, TARGET_PATH, STOPPED };
RobotState state = START;

// =======================
// Globals
// =======================
unsigned long bootTime = 0;
unsigned long lastDebug = 0;

unsigned long lastWiggle = 0;
int wiggleDir = 1; // 1 = right bias, -1 = left bias
int lastGoodDir = 1;
int lastGoodDist = 10000;

unsigned long lastSeenTarget = 0;
unsigned long blueSeenSince = 0;
bool blueLatched = false;
unsigned long pausedUntil = 0;
bool blueMarkerEvent = false;

bool recoveryActive = false;
int recoveryDir = 1;
unsigned long recoveryStarted = 0;

ColorTag targetColor = C_UNKNOWN;

// Last normalized values for debugging / logic
int normR = 0, normG = 0, normB = 0;

// =======================
// Helper Structs
// =======================
struct RawColor {
  unsigned int r;
  unsigned int g;
  unsigned int b;
  unsigned int c;
};

// =======================
// Motor Control
// =======================
void setDrive(int leftPWM, int rightPWM) {
  leftPWM = constrain(leftPWM, -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);

  if (leftPWM >= 0) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  }

  if (rightPWM >= 0) {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
  } else {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
  }

  analogWrite(ENA_PIN, abs(leftPWM));
  analogWrite(ENB_PIN, abs(rightPWM));
}

void stopDrive() {
  analogWrite(ENA_PIN, 0);
  analogWrite(ENB_PIN, 0);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}

void driveForward(int pwm) {
  setDrive(pwm, pwm);
}

void rotateLeft(int pwm) {
  setDrive(-pwm, pwm);
}

void rotateRight(int pwm) {
  setDrive(pwm, -pwm);
}

// =======================
// TCS3200 Low-Level
// =======================
static unsigned int readChannel(bool s2, bool s3) {
  digitalWrite(S2_PIN, s2 ? HIGH : LOW);
  digitalWrite(S3_PIN, s3 ? HIGH : LOW);
  delayMicroseconds(100); // settle

  // Collect samples and compute median to reduce noise
  unsigned int samples[COLOR_SAMPLES];
  for (int i = 0; i < COLOR_SAMPLES; i++) {
    // pulseIn returns pulse width in microseconds
    unsigned long pulse = pulseIn(OUT_PIN, LOW, PULSE_TIMEOUT_US);
    if (pulse == 0) pulse = PULSE_TIMEOUT_US; // treat timeout as dark
    samples[i] = (unsigned int)pulse;
  }

  // Simple sort for median
  for (int i = 0; i < COLOR_SAMPLES - 1; i++) {
    for (int j = i + 1; j < COLOR_SAMPLES; j++) {
      if (samples[j] < samples[i]) {
        unsigned int t = samples[i];
        samples[i] = samples[j];
        samples[j] = t;
      }
    }
  }
  return samples[COLOR_SAMPLES / 2];
}

RawColor readRawColor() {
  RawColor rc;
  // Red: S2=LOW, S3=LOW
  rc.r = readChannel(false, false);
  // Blue: S2=LOW, S3=HIGH
  rc.b = readChannel(false, true);
  // Green: S2=HIGH, S3=HIGH
  rc.g = readChannel(true, true);
  // Clear: S2=HIGH, S3=LOW
  rc.c = readChannel(true, false);
  return rc;
}

// Convert pulse widths to intensity (shorter pulse = more light)
// Also normalize to 0..255 based on sum
void normalizeColor(const RawColor &raw, int &rOut, int &gOut, int &bOut) {
  // Invert pulse width to intensity
  long r = (raw.r == 0) ? 0 : (100000L / raw.r);
  long g = (raw.g == 0) ? 0 : (100000L / raw.g);
  long b = (raw.b == 0) ? 0 : (100000L / raw.b);

  long sum = r + g + b;
  if (sum < 1) sum = 1;

  rOut = (int)((r * INTENSITY_MAX) / sum);
  gOut = (int)((g * INTENSITY_MAX) / sum);
  bOut = (int)((b * INTENSITY_MAX) / sum);
}

static int clearIntensity(const RawColor &raw) {
  if (raw.c == 0) return 0;
  return (int)(100000L / raw.c);
}

// =======================
// Color Classification
// =======================
ColorTag classifyColor(const RawColor &raw) {
  normalizeColor(raw, normR, normG, normB);
  unsigned int clearPulse = raw.c;

  // Black / White based on CLEAR pulse width
  if (clearPulse >= BLACK_CLEAR_PULSE_MIN_US) return C_BLACK;
  if (clearPulse <= WHITE_CLEAR_PULSE_MAX_US) return C_WHITE;

  // Dominance logic
  int maxv = max(normR, max(normG, normB));
  int midv = normR + normG + normB - maxv - min(normR, min(normG, normB));
  if (midv == 0) midv = 1;

  float ratio = (float)maxv / (float)midv;
  if (ratio < DOMINANCE_RATIO) return C_UNKNOWN;

  int dr = abs(normR - TARGET_RED_R) + abs(normG - TARGET_RED_G) + abs(normB - TARGET_RED_B);
  int dg = abs(normR - TARGET_GREEN_R) + abs(normG - TARGET_GREEN_G) + abs(normB - TARGET_GREEN_B);
  int db = abs(normR - TARGET_BLUE_R) + abs(normG - TARGET_BLUE_G) + abs(normB - TARGET_BLUE_B);

  int minDist = min(dr, min(dg, db));
  if (minDist > COLOR_DIST_MAX) return C_UNKNOWN;
  if (minDist == dr) return C_RED;
  if (minDist == dg) return C_GREEN;
  if (minDist == db) return C_BLUE;

  return C_UNKNOWN;
}

static const char *colorLabel(ColorTag c) {
  switch (c) {
    case C_BLACK: return "BLACK";
    case C_WHITE: return "WHITE";
    case C_RED: return "RED";
    case C_GREEN: return "GREEN";
    case C_BLUE: return "BLUE";
    default: return "UNKNOWN";
  }
}

// =======================
// Line Following Logic
// =======================
int colorDistanceToTarget(ColorTag target) {
  // Simple distance to ideal target (tune if needed)
  int tr = 0, tg = 0, tb = 0;
  if (target == C_RED)   { tr = TARGET_RED_R; tg = TARGET_RED_G; tb = TARGET_RED_B; }
  if (target == C_GREEN) { tr = TARGET_GREEN_R; tg = TARGET_GREEN_G; tb = TARGET_GREEN_B; }
  if (target == C_BLUE)  { tr = TARGET_BLUE_R; tg = TARGET_BLUE_G; tb = TARGET_BLUE_B; }

  int dr = abs(normR - tr);
  int dg = abs(normG - tg);
  int db = abs(normB - tb);
  return dr + dg + db;
}

void followLine(ColorTag target) {
  RawColor raw = readRawColor();
  ColorTag current = classifyColor(raw);

  unsigned long now = millis();

  // BLUE marker detection with debounce (non-blocking pause)
  bool brightnessOk = (raw.c >= BRIGHTNESS_OK_MIN_US && raw.c <= BRIGHTNESS_OK_MAX_US);
  // BLUE marker detection with debounce
  if (current == C_BLUE && brightnessOk) {
    if (blueSeenSince == 0) blueSeenSince = now;
    if (!blueLatched && (now - blueSeenSince >= BLUE_DEBOUNCE_MS)) {
      blueLatched = true;
      blueMarkerEvent = true;
    }
  } else {
    blueSeenSince = 0;
    blueLatched = false;
  }

  if (blueMarkerEvent) {
    blueMarkerEvent = false;
    Serial.println("BLUE MARKER");
    pausedUntil = now + BLUE_PAUSE_MS;
  }

  if (now < pausedUntil) {
    stopDrive();
    return;
  }

  // Track target color
  int dist = colorDistanceToTarget(target);
  bool onLine = (brightnessOk && dist <= TARGET_DIST_MAX);

  if (onLine) {
    lastSeenTarget = now;
    lastGoodDist = dist;

    // Wiggle scan
    if (now - lastWiggle >= WIGGLE_PERIOD_MS) {
      wiggleDir = -wiggleDir;
      lastWiggle = now;
    }

    // If confidence worse, steer opposite
    static int lastDist = 10000;
    if (dist > lastDist) {
      wiggleDir = -wiggleDir;
    }
    lastDist = dist;

    if (wiggleDir > 0) {
      setDrive(BASE_SPEED + TURN_SPEED, BASE_SPEED - TURN_SPEED);
      lastGoodDir = 1;
    } else {
      setDrive(BASE_SPEED - TURN_SPEED, BASE_SPEED + TURN_SPEED);
      lastGoodDir = -1;
    }
    return;
  }

  // Lost recovery
  if (now - lastSeenTarget > LOST_TIMEOUT_MS) {
    if (!recoveryActive) {
      recoveryActive = true;
      recoveryDir = lastGoodDir;
      recoveryStarted = now;
    }

    if (now - recoveryStarted >= RECOVERY_FULL_TIMEOUT_MS) {
      recoveryDir = -recoveryDir;
      recoveryStarted = now;
    }

    if (recoveryDir > 0) rotateRight(RECOVERY_ROTATE_SPEED);
    else rotateLeft(RECOVERY_ROTATE_SPEED);
    return;
  } else {
    recoveryActive = false;
  }

  // If not lost yet, keep moving forward gently
  driveForward(BASE_SPEED / 2);
}

// =======================
// Setup / Loop
// =======================
void setup() {
  Serial.begin(115200);

  // Sensor pins
  pinMode(S0_PIN, OUTPUT);
  pinMode(S1_PIN, OUTPUT);
  pinMode(S2_PIN, OUTPUT);
  pinMode(S3_PIN, OUTPUT);
  pinMode(OUT_PIN, INPUT);

  // TCS3200 scaling: 20% (S0 HIGH, S1 LOW)
  digitalWrite(S0_PIN, HIGH);
  digitalWrite(S1_PIN, LOW);

  // Motor pins
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  // Ultrasonic pins (stub)
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  stopDrive();

  bootTime = millis();
  lastSeenTarget = millis();
}

void loop() {
  unsigned long now = millis();

  // Debug / calibration mode for first 5 seconds
  if (now - bootTime < DEBUG_BOOT_MS) {
    if (now - lastDebug >= DEBUG_PERIOD_MS) {
      RawColor raw = readRawColor();
      ColorTag c = classifyColor(raw);
      int cInt = clearIntensity(raw);

      Serial.print("RAW R:");
      Serial.print(raw.r);
      Serial.print(" G:");
      Serial.print(raw.g);
      Serial.print(" B:");
      Serial.print(raw.b);
      Serial.print(" C:");
      Serial.print(raw.c);

      Serial.print(" | NORM R:");
      Serial.print(normR);
      Serial.print(" G:");
      Serial.print(normG);
      Serial.print(" B:");
      Serial.print(normB);

      Serial.print(" | CLR_INT:");
      Serial.print(cInt);

      Serial.print(" | CLASS:");
      Serial.println(colorLabel(c));

      lastDebug = now;
    }
    stopDrive();
    return;
  }

  switch (state) {
    case START: {
      driveForward(80);
      RawColor raw = readRawColor();
      ColorTag c = classifyColor(raw);

      if (c == C_RED) {
        state = OBSTACLE_COURSE;
        targetColor = C_RED;
        lastSeenTarget = now;
      } else if (c == C_GREEN) {
        state = TARGET_PATH;
        targetColor = C_GREEN;
        lastSeenTarget = now;
      }
    } break;

    case OBSTACLE_COURSE:
      followLine(C_RED);
      break;

    case TARGET_PATH:
      followLine(C_GREEN);
      break;

    case STOPPED:
    default:
      stopDrive();
      break;
  }
}
