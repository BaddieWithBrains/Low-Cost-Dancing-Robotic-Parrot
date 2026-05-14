#include <ESP32Servo.h>
#include "esp_pm.h"

// ── Pin config ────────────────────────────────────────────
#define MIC_PIN         34
#define SERVO_BODY_PIN  18
#define SERVO_LWING_PIN 19
#define SERVO_RWING_PIN 21

// ── Body positions ────────────────────────────────────────
#define BODY_NEUTRAL    90
#define BODY_BOB_UP     110
#define BODY_BOB_DOWN   70

// ── Wing positions (mirrored) ─────────────────────────────
#define LWING_NEUTRAL   90
#define LWING_UP        140
#define LWING_DOWN      40

#define RWING_NEUTRAL   90
#define RWING_UP        40
#define RWING_DOWN      140

// ── Beat detection ────────────────────────────────────────
#define SAMPLE_WINDOW     5
#define BEAT_THRESHOLD    1000
#define BEAT_CEILING      3500
#define SOFTWARE_GAIN     1.0f

// ── Tempo adaptive limits ─────────────────────────────────
#define MIN_COOLDOWN      40
#define MAX_COOLDOWN      350
#define MIN_HOLD          20
#define MAX_HOLD          120

Servo servoBody;
Servo servoLWing;
Servo servoRWing;

enum WingPhase {
  PHASE_UP,
  PHASE_DOWN,
  PHASE_SEESAW_A,   // Left up, right down
  PHASE_SEESAW_B    // Right up, left down
};

WingPhase currentPhase = PHASE_UP;
bool bobbedUp          = true;
bool servosActive      = false;

unsigned long lastBeat     = 0;
unsigned long servoTimer   = 0;
unsigned long beatInterval = 300;
int currentHoldTime        = 80;
int currentCooldown        = 150;

int bodyPos  = BODY_NEUTRAL;
int lwingPos = LWING_NEUTRAL;
int rwingPos = RWING_NEUTRAL;

// ── Read mic amplitude ────────────────────────────────────
int readAmplitude() {
  unsigned long start = millis();
  int signalMax = 0;
  int signalMin = 4095;

  while (millis() - start < SAMPLE_WINDOW) {
    int sample = analogRead(MIC_PIN);
    if (sample > signalMax) signalMax = sample;
    if (sample < signalMin) signalMin = sample;
  }
  return (int)((signalMax - signalMin) * SOFTWARE_GAIN);
}

// ── Map beat interval to hold and cooldown ────────────────
void updateTempo(unsigned long interval) {
  interval        = constrain(interval, MIN_COOLDOWN, MAX_COOLDOWN);
  currentHoldTime = map(interval, MIN_COOLDOWN, MAX_COOLDOWN, MIN_HOLD, MAX_HOLD);
  currentCooldown = map(interval, MIN_COOLDOWN, MAX_COOLDOWN, MIN_COOLDOWN, MAX_COOLDOWN);
}

// ── Servo helpers ─────────────────────────────────────────
void allNeutral() {
  bodyPos  = BODY_NEUTRAL;
  lwingPos = LWING_NEUTRAL;
  rwingPos = RWING_NEUTRAL;
  servoBody.write(bodyPos);
  servoLWing.write(lwingPos);
  servoRWing.write(rwingPos);
}

void bobBody() {
  if (bobbedUp) {
    bodyPos = BODY_BOB_DOWN;
  } else {
    bodyPos = BODY_BOB_UP;
  }
  bobbedUp = !bobbedUp;
  servoBody.write(bodyPos);
}

// ── Both wings UP ─────────────────────────────────────────
void triggerBothUp() {
  bobBody();
  lwingPos = LWING_UP;
  rwingPos = RWING_UP;
  servoLWing.write(lwingPos);
  servoRWing.write(rwingPos);
}

// ── Both wings DOWN ───────────────────────────────────────
void triggerBothDown() {
  bobBody();
  lwingPos = LWING_DOWN;
  rwingPos = RWING_DOWN;
  servoLWing.write(lwingPos);
  servoRWing.write(rwingPos);
}

// ── Seesaw A — left UP, right DOWN ───────────────────────
void triggerSeesawA() {
  bobBody();
  lwingPos = LWING_UP;
  rwingPos = RWING_DOWN;
  servoLWing.write(lwingPos);
  servoRWing.write(rwingPos);
}

// ── Seesaw B — right UP, left DOWN ───────────────────────
void triggerSeesawB() {
  bobBody();
  lwingPos = LWING_DOWN;
  rwingPos = RWING_UP;
  servoLWing.write(lwingPos);
  servoRWing.write(rwingPos);
}

// ── Advance to next phase in 4 beat cycle ────────────────
void nextPhase() {
  switch (currentPhase) {
    case PHASE_UP:       currentPhase = PHASE_DOWN;     break;
    case PHASE_DOWN:     currentPhase = PHASE_SEESAW_A; break;
    case PHASE_SEESAW_A: currentPhase = PHASE_SEESAW_B; break;
    case PHASE_SEESAW_B: currentPhase = PHASE_UP;       break;
  }
}

void setup() {
  esp_pm_config_esp32_t pm_config = {
    .max_freq_mhz       = 240,
    .min_freq_mhz       = 240,
    .light_sleep_enable = false
  };
  esp_pm_configure(&pm_config);

  analogSetAttenuation(ADC_11db);
  analogSetWidth(12);
  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);

  servoBody.setPeriodHertz(50);
  servoLWing.setPeriodHertz(50);
  servoRWing.setPeriodHertz(50);

  servoBody.attach(SERVO_BODY_PIN,   500, 2400);
  servoLWing.attach(SERVO_LWING_PIN, 500, 2400);
  servoRWing.attach(SERVO_RWING_PIN, 500, 2400);

  allNeutral();
  delay(500);
  Serial.println("Parrot ready!");
}

void loop() {
  unsigned long now = millis();

  // ── Return to neutral after hold time ─────────────────
  if (servosActive && (now - servoTimer >= currentHoldTime)) {
    allNeutral();
    servosActive = false;
  }

  // ── Read mic ───────────────────────────────────────────
  int amplitude = readAmplitude();

  // ── Serial plotter ─────────────────────────────────────
  int ampScaled       = map(amplitude, 0, 4095, 0, 180);
  int thresholdScaled = map(BEAT_THRESHOLD, 0, 4095, 0, 180);

  Serial.print("Body:");
  Serial.print(bodyPos);
  Serial.print(",");
  Serial.print("LWing:");
  Serial.print(lwingPos);
  Serial.print(",");
  Serial.print("RWing:");
  Serial.print(rwingPos);
  Serial.print(",");
  Serial.print("Amplitude:");
  Serial.print(ampScaled);
  Serial.print(",");
  Serial.print("Threshold:");
  Serial.println(thresholdScaled);

  // ── Beat detection ─────────────────────────────────────
  bool isBeat = amplitude > BEAT_THRESHOLD &&
                amplitude < BEAT_CEILING &&
                (now - lastBeat > currentCooldown);

  if (isBeat) {
    beatInterval = now - lastBeat;
    updateTempo(beatInterval);

    lastBeat     = now;
    servoTimer   = now;
    servosActive = true;

    // ── Execute current phase then advance ────────────
    switch (currentPhase) {
      case PHASE_UP:       triggerBothUp();   break;
      case PHASE_DOWN:     triggerBothDown(); break;
      case PHASE_SEESAW_A: triggerSeesawA();  break;
      case PHASE_SEESAW_B: triggerSeesawB();  break;
    }

    nextPhase();
  }
}