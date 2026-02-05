#include <ESP32Servo.h>

static const int PIN_PAN  = 18;
static const int PIN_TILT = 19;

Servo servoPan;
Servo servoTilt;

// current and target angles (degrees)
float panCur  = 90.0f;
float tiltCur = 0.0f;
float panTgt  = 90.0f;
float tiltTgt = 0.0f;

// max degrees per update tick (smaller = smoother, slower)
static const float MAX_STEP_DEG = 2.0f;

// update rate (ms) -> 20ms = 50Hz
static const uint32_t SERVO_UPDATE_MS = 20;
uint32_t lastUpdateMs = 0;

// pulse calibration
static const int SERVO_MIN_US = 500;
static const int SERVO_MAX_US = 2500;

static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline float moveToward(float cur, float tgt, float maxStep) {
  float d = tgt - cur;
  if (d >  maxStep) d =  maxStep;
  if (d < -maxStep) d = -maxStep;
  return cur + d;
}

bool parseAngles(const String& line, float& panOut, float& tiltOut) {
  int comma = line.indexOf(',');
  if (comma < 0) return false;

  String a = line.substring(0, comma);
  String b = line.substring(comma + 1);

  a.trim();
  b.trim();
  if (a.length() == 0 || b.length() == 0) return false;

  // toFloat() returns 0.0 on failure too, so we do a small extra check:
  // accept if it has at least one digit.
  auto hasDigit = [](const String& s) {
    for (size_t i = 0; i < s.length(); i++) {
      if (isDigit((unsigned char)s[i])) return true;
    }
    return false;
  };
  if (!hasDigit(a) || !hasDigit(b)) return false;

  panOut  = a.toFloat();
  tiltOut = b.toFloat();
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5); // keep reads snappy

  // ESP32Servo setup
  servoPan.setPeriodHertz(50);
  servoTilt.setPeriodHertz(50);

  servoPan.attach(PIN_PAN, SERVO_MIN_US, SERVO_MAX_US);
  servoTilt.attach(PIN_TILT, SERVO_MIN_US, SERVO_MAX_US);

  // Move to neutral
  servoPan.write(panCur);
  servoTilt.write(tiltCur);
}

void loop() {
  // read incoming command lines (non-blocking)
  while (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    float p, t;
    if (parseAngles(line, p, t)) {
      panTgt  = clampf(p, 0.0f, 180.0f);
      tiltTgt = clampf(t, 0.0f, 180.0f);
    } else {
    }
  }

  // update servos at a fixed rate (smooth movement)
  uint32_t now = millis();
  if (now - lastUpdateMs >= SERVO_UPDATE_MS) {
    lastUpdateMs = now;

    panCur  = moveToward(panCur,  panTgt,  MAX_STEP_DEG);
    tiltCur = moveToward(tiltCur, tiltTgt, MAX_STEP_DEG);

    servoPan.write(panCur);
    servoTilt.write(tiltCur);
  }
}