/*
  =============================================================================
  Project: NodeMCU ESP8266 12E - Landslide Pre-Detection & Safety Gate System
  Pin Allocation:
    - MPU-6050 IMU: SCL -> GPIO5 (D1), SDA -> GPIO4 (D2)
    - YL-69 Soil Sensor: D0 -> GPIO0 (D3), A0 -> ADC0 (A0)
    - SW-420 Vibration: DO -> GPIO14 (D5, Interrupt)
    - IR Proximity Sensor: DO -> GPIO12 (D6)
    - Servo Motor Gate Signal: -> GPIO13 (D7)
    - 2-Pin Active Buzzer: -> GPIO15 (D8)
  
  Emergency Safety & Actuation Logic:
    - If Landslide Risk >= 70%:
        * Distinct buzzer alarm pattern sounds
        * User-configured evacuation countdown starts (Default: 30s, configurable via Admin)
        * High-end animated circular countdown & warning glows on public portal
        * Digital emergency lockdown clock on admin console
    - When Countdown expires:
        * Servo rotates 375ms anticlockwise to physically block the road
        * Servo stops & detaches immediately (no self-movement)
    - Manual Web Override:
        * User clicks "Open Road" button on dashboard
        * Servo rotates 375ms clockwise to reopen gate and parks
  =============================================================================
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <Wire.h>
#include <Servo.h>
#include <EEPROM.h>
#include <LittleFS.h>

// ---------------------- Wi-Fi Hotspot Configuration ----------------------
const char* AP_SSID = "Landslide Detection Device";
const char* AP_PASS = ""; // Open Hotspot (No password required)

// ---------------------- Hardware Pin Definitions -------------------------
const uint8_t I2C_SDA_PIN      = 4;  // D2 on NodeMCU -> GPIO4 (MPU-6050 SDA)
const uint8_t I2C_SCL_PIN      = 5;  // D1 on NodeMCU -> GPIO5 (MPU-6050 SCL)
const uint8_t SOIL_DIGITAL_PIN = 0;  // D3 on NodeMCU -> GPIO00 (Soil LM393 D0)
const uint8_t SOIL_ANALOG_PIN  = A0; // A0 on NodeMCU -> ADC0 (Soil LM393 A0)
const uint8_t VIB_PIN          = 14; // D5 on NodeMCU -> GPIO14 (SW-420 DO)
const uint8_t IR_PIN           = 12; // D6 on NodeMCU -> GPIO12 (IR DO)
const uint8_t SERVO_PIN        = 13; // D7 on NodeMCU -> GPIO13 (Servo Signal)
const uint8_t BUZZER_PIN       = 15; // D8 on NodeMCU -> GPIO15 (Active Buzzer)

// ---------------------- Actuation Objects & States -----------------------
Servo gateServo;

enum GateState { GATE_OPEN, GATE_CLOSING, GATE_CLOSED, GATE_OPENING };
GateState gateStatus = GATE_OPEN;

bool servoMoving = false;
unsigned long servoMoveStartTime = 0;
const unsigned long SERVO_PULSE_DURATION_MS = 375; // Rotate for exactly 375ms
GateState targetGateStatus = GATE_OPEN;

// Buzzer timing pattern
bool buzzerActive = false;
bool buzzerMuted = false;
unsigned long lastBuzzerToggle = 0;
bool buzzerState = false;

// ---------------------- Emergency & Countdown Logic ---------------------
const float EMERGENCY_RISK_THRESHOLD = 70.0;
unsigned int countdownDurationSecs = 30; // Configurable duration (default 30s)
unsigned long countdownDurationMs = 30000;

bool emergencyActive = false;
unsigned long emergencyStartTime = 0;
int remainingCountdownSecs = 30;
bool gateClosedForEmergency = false;

// EEPROM Persistent Storage for Emergency Timing Settings
#define EEPROM_CONFIG_SIZE 32
#define EEPROM_MAGIC_KEY   0x4C53 // "LS" LandSlide identifier

void initConfigEEPROM() {
  EEPROM.begin(EEPROM_CONFIG_SIZE);
  uint16_t magic = 0;
  EEPROM.get(0, magic);
  if (magic == EEPROM_MAGIC_KEY) {
    uint16_t savedDuration = 0;
    EEPROM.get(2, savedDuration);
    if (savedDuration >= 5 && savedDuration <= 300) {
      countdownDurationSecs = savedDuration;
      countdownDurationMs = (unsigned long)countdownDurationSecs * 1000;
      remainingCountdownSecs = countdownDurationSecs;
      Serial.print(F("[EEPROM] Loaded custom countdown duration: "));
      Serial.print(countdownDurationSecs);
      Serial.println(F(" seconds"));
      return;
    }
  }
  Serial.println(F("[EEPROM] No custom config found. Defaulting to 30s countdown."));
}

bool saveCountdownConfig(unsigned int newSecs) {
  if (newSecs < 5 || newSecs > 300) return false;
  countdownDurationSecs = newSecs;
  countdownDurationMs = (unsigned long)countdownDurationSecs * 1000;
  if (!emergencyActive) {
    remainingCountdownSecs = countdownDurationSecs;
  }
  EEPROM.put(0, (uint16_t)EEPROM_MAGIC_KEY);
  EEPROM.put(2, (uint16_t)newSecs);
  bool ok = EEPROM.commit();
  Serial.print(F("[EEPROM] Saved countdown duration: "));
  Serial.print(newSecs);
  Serial.println(ok ? F("s (SUCCESS)") : F("s (COMMIT FAILED)"));
  return ok;
}

// ---------------------- MPU-6050 State -----------------------------------
const int MPU_ADDR = 0x68;
bool mpuAvailable = false;
float accelX = 0, accelY = 0, accelZ = 1.0;
float pitch = 0, roll = 0;
float baselinePitch = 0, baselineRoll = 0;
bool baselineCalibrated = false;
unsigned long calibrationStartTime = 0;

// ---------------------- Soil & Calibration -------------------------------
const int DRY_ADC_VALUE = 1023;
const int WET_ADC_VALUE = 300;

// ---------------------- Vibration State Tracking -------------------------
const int VIB_WINDOW_SECS = 4;
#define VIB_RING_SIZE 16
volatile unsigned long vibTimestamps[VIB_RING_SIZE];
volatile uint8_t vibHead = 0;
volatile unsigned long totalVibCount = 0;
volatile unsigned long lastVibIsrTime = 0;

void IRAM_ATTR handleVibrationInterrupt() {
  unsigned long now = millis();
  // Debounce protection: ignore pulses < 20ms to prevent CPU interrupt storms & WDT lockups
  if (now - lastVibIsrTime > 20) {
    vibTimestamps[vibHead % VIB_RING_SIZE] = now;
    vibHead++;
    totalVibCount++;
    lastVibIsrTime = now;
  }
}

// ---------------------- IR Displacement Tracking -------------------------
volatile unsigned long irDetectionCount = 0;
int lastIrState = -1;
unsigned long lastIrChangeTime = 0;

// ---------------------- Landslide Scoring System -------------------------
const float TILT_NOISE_THRESHOLD_DEG   = 2.5;  // Filter wind/settling (< 2.5 deg)
const float TILT_MAX_CRITICAL_DEG      = 15.0; // Severe rotation (> 15 deg)
const int   SOIL_SATURATION_THRESHOLD  = 50;   // Soil starts weakening at > 50%
const int   VIB_CONTINUOUS_RATE_THRESH = 3;    // >= 3 hits in 4s window = continuous tremor

// Temporal Filtering (consecutive evaluation cycle verification window)
const uint8_t REQUIRED_CONSECUTIVE_CYCLES = 4; // Require 4 consecutive cycles (~400-500ms)
uint8_t consecutiveElevatedCycles = 0;
unsigned long lastAlgoEvaluationTime = 0;

unsigned long elevatedRiskStartTime = 0;
bool elevatedStateActive = false;
float cumulativeRiskPct = 0.0;
uint8_t concurrentAbnormalCount = 0;

// Breakdown & Sensor Contribution Telemetry Variables
float lastScoreSoil = 0.0;
float lastSoilMultiplier = 1.0;
float lastScoreTilt = 0.0;
float lastScoreVib = 0.0;
float lastScoreDisp = 0.0;
float lastInstantRisk = 0.0;
bool lastSynergySoilAndVib = false;
bool lastSynergyTiltAndVib = false;
bool lastSynergyTripleCritical = false;
bool lastSuppressionActive = false;

// ---------------------- Networking & Web Server --------------------------
const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 4, 1);
IPAddress netMsk(255, 255, 255, 0);

DNSServer dnsServer;
ESP8266WebServer server(80);

// ---------------------- MPU-6050 Driver ----------------------------------
bool initMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  byte err = Wire.endTransmission();
  return (err == 0);
}

void readMPU6050() {
  if (!mpuAvailable) return;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    // I2C communication glitch, return without stalling loop
    return;
  }
  
  uint8_t count = Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14, (bool)true);
  if (count >= 14) {
    int16_t rawAx = (Wire.read() << 8) | Wire.read();
    int16_t rawAy = (Wire.read() << 8) | Wire.read();
    int16_t rawAz = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Temp
    Wire.read(); Wire.read(); // Gyro X
    Wire.read(); Wire.read(); // Gyro Y
    Wire.read(); Wire.read(); // Gyro Z

    float ax = rawAx / 16384.0;
    float ay = rawAy / 16384.0;
    float az = rawAz / 16384.0;

    accelX = accelX * 0.75 + ax * 0.25;
    accelY = accelY * 0.75 + ay * 0.25;
    accelZ = accelZ * 0.75 + az * 0.25;

    roll = atan2(accelY, accelZ) * 57.2957795;
    pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 57.2957795;

    if (!baselineCalibrated) {
      baselinePitch = baselinePitch * 0.8 + pitch * 0.2;
      baselineRoll  = baselineRoll  * 0.8 + roll  * 0.2;
      if (millis() - calibrationStartTime > 3000) {
        baselineCalibrated = true;
        Serial.println(F("[ALGO] Baseline IMU Tilt Zeroed."));
      }
    }
  }
}

// ---------------------- Multi-Sensor Fusion & Weighted Risk Logic --------
void logTelemetrySnapshot(bool forceLog = false);
void logAuditEvent(const char* category, const char* severity, const char* source, const String& message);

void computeLandslideRisk() {
  readMPU6050();

  // 1. SOIL MOISTURE (YL-69): Vulnerability Baseline & Dynamic Multiplier
  int rawAdc = analogRead(SOIL_ANALOG_PIN);
  int clampedAdc = constrain(rawAdc, WET_ADC_VALUE, DRY_ADC_VALUE);
  int moisturePct = map(clampedAdc, DRY_ADC_VALUE, WET_ADC_VALUE, 0, 100);

  // Compute Soil Saturation Score (S_soil) & Vulnerability Multiplier (M_soil)
  // Saturated ground loses internal shear strength, amplifying sensitivity of tilt & tremors
  float scoreSoil = 0.0;
  float soilVulnerabilityMultiplier = 1.0;

  if (moisturePct > SOIL_SATURATION_THRESHOLD) {
    if (moisturePct <= 80) {
      // 50% to 80% moisture: progressive ground destabilization
      float ratio = (float)(moisturePct - SOIL_SATURATION_THRESHOLD) / (80 - SOIL_SATURATION_THRESHOLD);
      scoreSoil = ratio * 60.0;
      soilVulnerabilityMultiplier = 1.0 + (0.4 * ratio); // 1.0x to 1.4x amplification
    } else {
      // > 80% moisture: high saturation / mudslide liquefaction hazard
      float ratio = (float)(moisturePct - 80) / (100 - 80);
      scoreSoil = 60.0 + (ratio * 40.0);
      soilVulnerabilityMultiplier = 1.4 + (0.4 * ratio); // 1.4x to 1.8x amplification
    }
  }
  lastScoreSoil = scoreSoil;
  lastSoilMultiplier = soilVulnerabilityMultiplier;

  // 2. SLOPE TILT DEVIATION (MPU-6050): Dynamic Structural Angle Shift
  float deltaPitch = abs(pitch - baselinePitch);
  float deltaRoll  = abs(roll  - baselineRoll);
  float totalTiltDelta = sqrt(deltaPitch * deltaPitch + deltaRoll * deltaRoll);

  float scoreTilt = 0.0;
  if (totalTiltDelta > TILT_NOISE_THRESHOLD_DEG) {
    scoreTilt = ((totalTiltDelta - TILT_NOISE_THRESHOLD_DEG) / (TILT_MAX_CRITICAL_DEG - TILT_NOISE_THRESHOLD_DEG)) * 100.0;
    scoreTilt = constrain(scoreTilt, 0.0, 100.0);
  }
  lastScoreTilt = scoreTilt;

  // 3. CONTINUOUS SEISMIC TREMORS (SW-420): Dynamic High-Frequency Shaking
  unsigned long now = millis();
  int recentVibHits = 0;
  for (int i = 0; i < VIB_RING_SIZE; i++) {
    if (vibTimestamps[i] > 0 && (now - vibTimestamps[i] < (VIB_WINDOW_SECS * 1000))) {
      recentVibHits++;
    }
  }

  float scoreVib = 0.0;
  if (recentVibHits >= VIB_CONTINUOUS_RATE_THRESH) {
    // 3 to 12 hits scaled from 40% to 100%
    scoreVib = map(constrain(recentVibHits, VIB_CONTINUOUS_RATE_THRESH, 12), VIB_CONTINUOUS_RATE_THRESH, 12, 40, 100);
  } else if (recentVibHits == 2) {
    scoreVib = 20.0; // Minor micro-tremor
  }
  lastScoreVib = scoreVib;

  // 4. GROUND DISPLACEMENT (IR Sensor): Surface Rupture / Rockfall Confirmation
  int irState = digitalRead(IR_PIN);
  float scoreDisp = (irState == LOW) ? 100.0 : 0.0;
  lastScoreDisp = scoreDisp;

  // 5. CONCURRENT ABNORMAL SENSOR COUNT
  concurrentAbnormalCount = 0;
  if (scoreTilt >= 25.0) concurrentAbnormalCount++;
  if (scoreSoil >= 25.0) concurrentAbnormalCount++;
  if (scoreVib  >= 30.0) concurrentAbnormalCount++;
  if (scoreDisp >= 50.0) concurrentAbnormalCount++;

  // 6. WEIGHTED FUSION WITH VULNERABILITY MULTIPLIER
  // Dynamic factors: Tilt (35%), Vibration (30%), Surface Displacement (20%)
  // Static vulnerability baseline: Soil Moisture (15%)
  float dynamicSensorsScore = (scoreTilt * 0.35) + (scoreVib * 0.30) + (scoreDisp * 0.20);
  float amplifiedDynamicScore = dynamicSensorsScore * soilVulnerabilityMultiplier;
  float instantRisk = amplifiedDynamicScore + (scoreSoil * 0.15);

  // 7. SMART LOGICAL COMBINATION RULES (Synergy Triggers)
  // Synergy Rule A: Saturated Soil + Continuous Vibrations (Liquefaction Hazard)
  bool synergySoilAndVib = (moisturePct >= 65 && recentVibHits >= VIB_CONTINUOUS_RATE_THRESH);
  if (synergySoilAndVib) {
    instantRisk = max(instantRisk, 72.0f);
  }

  // Synergy Rule B: Structural Slope Tilting + Continuous Vibrations (Active Mass Slip)
  bool synergyTiltAndVib = (totalTiltDelta >= 4.5 && recentVibHits >= VIB_CONTINUOUS_RATE_THRESH);
  if (synergyTiltAndVib) {
    instantRisk = max(instantRisk, 78.0f);
  }

  // Synergy Rule C: High Moisture + Tilt Shift + Surface Rupture (Catastrophic Failure)
  bool synergyTripleCritical = (moisturePct >= 60 && totalTiltDelta >= 3.5 && scoreDisp > 50.0);
  if (synergyTripleCritical) {
    instantRisk = max(instantRisk, 90.0f);
  }

  // Synergy Rule D: Isolated Spikes Suppression (Anti-False-Positive Filter)
  // If only a single isolated sensor triggers without corroboration, cap below 38%
  bool suppressionActive = false;
  if (concurrentAbnormalCount < 2 && !synergySoilAndVib && !synergyTiltAndVib && !synergyTripleCritical) {
    instantRisk = min(instantRisk, 38.0f);
    suppressionActive = true;
  }

  instantRisk = constrain(instantRisk, 0.0f, 100.0f);
  lastInstantRisk = instantRisk;
  lastSynergySoilAndVib = synergySoilAndVib;
  lastSynergyTiltAndVib = synergyTiltAndVib;
  lastSynergyTripleCritical = synergyTripleCritical;
  lastSuppressionActive = suppressionActive;

  // 8. TEMPORAL FILTERING (Time-Window Persistence Filter)
  // Require anomaly states to persist over consecutive evaluation cycles
  // to eliminate electrical spikes or momentary transients
  if (instantRisk >= 60.0 && (concurrentAbnormalCount >= 2 || synergySoilAndVib || synergyTiltAndVib)) {
    if (consecutiveElevatedCycles < REQUIRED_CONSECUTIVE_CYCLES + 5) {
      consecutiveElevatedCycles++;
    }
  } else {
    if (consecutiveElevatedCycles > 0) {
      consecutiveElevatedCycles--; // Leaky bucket filter
    }
  }

  // Smooth the cumulative risk with an Exponential Moving Average (EMA)
  float alpha = (consecutiveElevatedCycles >= REQUIRED_CONSECUTIVE_CYCLES) ? 0.45 : 0.25;
  cumulativeRiskPct = (cumulativeRiskPct * (1.0 - alpha)) + (instantRisk * alpha);

  // Track threshold transitions for system audit trail
  static int lastRiskAuditTier = 0; // 0: Normal (<45%), 1: Elevated (45-69%), 2: Critical (>=70%)
  int currentAuditTier = 0;
  if (cumulativeRiskPct >= EMERGENCY_RISK_THRESHOLD) currentAuditTier = 2;
  else if (cumulativeRiskPct >= 45.0) currentAuditTier = 1;
  else currentAuditTier = 0;

  if (currentAuditTier != lastRiskAuditTier) {
    if (currentAuditTier == 1 && lastRiskAuditTier == 0) {
      logAuditEvent("THRESHOLD", "WARNING", "Fusion Engine", "Geotechnical risk crossed elevated threshold (45.0%). Cumulative risk: " + String(cumulativeRiskPct, 1) + "%");
    } else if (currentAuditTier == 0 && lastRiskAuditTier > 0) {
      logAuditEvent("THRESHOLD", "INFO", "Fusion Engine", "Geotechnical risk normalized below elevated threshold (<45.0%). Cumulative risk: " + String(cumulativeRiskPct, 1) + "%");
    }
    lastRiskAuditTier = currentAuditTier;
  }

  // Elevated State Active only when verified across temporal cycle window
  if (consecutiveElevatedCycles >= REQUIRED_CONSECUTIVE_CYCLES && cumulativeRiskPct >= 60.0) {
    elevatedStateActive = true;
  } else {
    elevatedStateActive = false;
  }

  // 9. EMERGENCY ACTUATION EVALUATION
  // Trigger emergency if risk >= 70% AND confirmed over temporal window
  if (cumulativeRiskPct >= EMERGENCY_RISK_THRESHOLD && consecutiveElevatedCycles >= REQUIRED_CONSECUTIVE_CYCLES && !emergencyActive && !gateClosedForEmergency) {
    emergencyActive = true;
    emergencyStartTime = now;
    buzzerActive = true;
    buzzerMuted = false;
    Serial.println(F("[SAFETY] *** EMERGENCY TRIGGERED! *** Confirmed Landslide Risk >= 70%!"));
    logAuditEvent("EMERGENCY", "CRITICAL", "Safety Logic", "CRITICAL RISK >= 70% confirmed (" + String(cumulativeRiskPct, 1) + "%). Initiating " + String(countdownDurationSecs) + "s evacuation countdown and siren.");
    logTelemetrySnapshot(true); // Force snapshot log on emergency trigger
  }

  // Handle Emergency Countdown
  if (emergencyActive && !gateClosedForEmergency) {
    long elapsed = now - emergencyStartTime;
    long remainingMs = (long)countdownDurationMs - elapsed;
    
    if (remainingMs > 0) {
      remainingCountdownSecs = (remainingMs + 999) / 1000;
    } else {
      // Countdown Expired: Physically block road
      remainingCountdownSecs = 0;
      gateClosedForEmergency = true;
      Serial.println(F("[SAFETY] Countdown Elapsed. Closing Road Gate!"));
      logAuditEvent("ACTUATION", "CRITICAL", "Safety Logic", "Evacuation countdown elapsed. Automated road gate closed (375ms anticlockwise blockade).");
      triggerGateClose();
    }
  }
}

// ---------------------- Servo Motor Commands -----------------------------
void triggerGateClose() {
  if (servoMoving) return;
  Serial.println(F("[SERVO] Rotating 375ms Anticlockwise to Close Gate..."));
  gateServo.attach(SERVO_PIN);
  gateServo.write(0); // Full speed anticlockwise
  servoMoveStartTime = millis();
  servoMoving = true;
  gateStatus = GATE_CLOSING;
  targetGateStatus = GATE_CLOSED;
  logTelemetrySnapshot(true); // Log gate closure
}

void triggerGateOpen() {
  if (servoMoving) return;
  Serial.println(F("[SERVO] Rotating 375ms Clockwise to Open Gate..."));
  gateServo.attach(SERVO_PIN);
  gateServo.write(180); // Full speed clockwise
  servoMoveStartTime = millis();
  servoMoving = true;
  gateStatus = GATE_OPENING;
  targetGateStatus = GATE_OPEN;

  // Reset emergency state when manually cleared by user
  emergencyActive = false;
  gateClosedForEmergency = false;
  buzzerActive = false;
  buzzerMuted = false;
  digitalWrite(BUZZER_PIN, LOW);
  remainingCountdownSecs = countdownDurationSecs;
  logTelemetrySnapshot(true); // Log gate reopen
}

void updateServoMovement() {
  if (servoMoving) {
    if (millis() - servoMoveStartTime >= SERVO_PULSE_DURATION_MS) {
      // Rotate duration finished: Stop and detach servo so it NEVER moves on its own
      gateServo.write(90); // Neutral stop signal
      gateServo.detach();  // Cut PWM completely
      servoMoving = false;
      gateStatus = targetGateStatus;
      Serial.print(F("[SERVO] Gate Movement Complete. Parked at: "));
      Serial.println(gateStatus == GATE_CLOSED ? "BLOCKED" : "OPEN");
    }
  }
}

// ---------------------- Active Buzzer Engine -----------------------------
void updateBuzzerPattern() {
  if (buzzerActive && !buzzerMuted) {
    unsigned long now = millis();
    // Distinct urgent cadence: 150ms ON, 150ms OFF
    if (now - lastBuzzerToggle >= 150) {
      lastBuzzerToggle = now;
      buzzerState = !buzzerState;
      digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
    }
  } else {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerState = false;
  }
}

// ---------------------- Admin Security Configuration --------------------
const char* ADMIN_KEY = "123"; // Passkey for privileged gate actuation & admin panel

/// ---------------------- 1. PUBLIC/VISITOR LANDING PAGE (/) --------------
const char PUBLIC_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no, viewport-fit=cover">
  <meta name="theme-color" content="#1e3a8a">
  <meta name="apple-mobile-web-app-capable" content="yes">
  <meta name="apple-mobile-web-app-status-bar-style" content="default">
  <title>Geotechnical Hazard Advisory — Mountain Corridor Safety Portal</title>
  <style>
    :root {
      --bg: #f8fafc;
      --surface: #ffffff;
      --surface-subtle: #f1f5f9;
      --border: #e2e8f0;
      --border-strong: #cbd5e1;
      --text-main: #0f172a;
      --text-sub: #334155;
      --text-muted: #64748b;
      --navy-gov: #1e3a8a;
      --navy-light: #eff6ff;
      --navy-border: #bfdbfe;
      --emerald: #059669;
      --emerald-bg: #ecfdf5;
      --emerald-border: #a7f3d0;
      --emerald-text: #065f46;
      --amber: #d97706;
      --amber-bg: #fffbeb;
      --amber-border: #fde68a;
      --amber-text: #92400e;
      --crimson: #dc2626;
      --crimson-bg: #fef2f2;
      --crimson-border: #fca5a5;
      --crimson-text: #991b1b;
      --shadow-sm: 0 1px 3px rgba(15, 23, 42, 0.06), 0 1px 2px rgba(15, 23, 42, 0.04);
      --shadow-md: 0 4px 6px -1px rgba(15, 23, 42, 0.07), 0 2px 4px -2px rgba(15, 23, 42, 0.05);
      --shadow-lg: 0 10px 15px -3px rgba(15, 23, 42, 0.08), 0 4px 6px -4px rgba(15, 23, 42, 0.04);
    }
    *, *::before, *::after {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
      -webkit-tap-highlight-color: transparent;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    }
    body {
      background-color: var(--bg);
      background-image: 
        radial-gradient(at 0% 0%, rgba(224, 231, 255, 0.5) 0px, transparent 50%),
        radial-gradient(at 100% 0%, rgba(241, 245, 249, 0.8) 0px, transparent 50%);
      color: var(--text-main);
      min-height: 100vh;
      min-height: -webkit-fill-available;
      width: 100%;
      max-width: 100vw;
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: max(10px, env(safe-area-inset-top)) 10px max(24px, env(safe-area-inset-bottom));
      overflow-x: hidden;
      -webkit-font-smoothing: antialiased;
      font-feature-settings: "tnum" 1;
    }
    .wrapper {
      width: 100%;
      max-width: 480px;
      margin: 0 auto;
      display: flex;
      flex-direction: column;
      gap: 12px;
      box-sizing: border-box;
    }

    /* Government Agency Header */
    .gov-header {
      width: 100%;
      background: var(--surface);
      border: 1px solid var(--border);
      border-top: 4px solid var(--navy-gov);
      border-radius: 14px;
      padding: 16px 14px;
      box-shadow: var(--shadow-sm);
      text-align: center;
      display: flex;
      flex-direction: column;
      align-items: center;
      box-sizing: border-box;
    }
    .gov-agency-badge {
      display: inline-flex;
      align-items: center;
      justify-content: center;
      gap: 6px;
      padding: 4px 10px;
      font-size: 0.68rem;
      font-weight: 800;
      border-radius: 9999px;
      background: var(--navy-light);
      color: var(--navy-gov);
      border: 1px solid var(--navy-border);
      text-transform: uppercase;
      letter-spacing: 0.5px;
      margin-bottom: 8px;
    }
    .beacon-dot {
      width: 7px;
      height: 7px;
      border-radius: 50%;
      background: var(--emerald);
      box-shadow: 0 0 6px var(--emerald);
      animation: pulse-beacon 1.6s infinite ease-in-out;
    }
    @keyframes pulse-beacon {
      0%, 100% { opacity: 1; transform: scale(1); }
      50% { opacity: 0.4; transform: scale(0.85); }
    }
    .gov-title {
      font-size: clamp(1.15rem, 4.5vw, 1.38rem);
      font-weight: 900;
      color: var(--text-main);
      letter-spacing: -0.3px;
      line-height: 1.3;
      text-align: center;
    }
    .gov-subtitle {
      color: var(--text-muted);
      font-size: 0.78rem;
      margin-top: 4px;
      font-weight: 500;
      line-height: 1.4;
      text-align: center;
    }

    /* Clean Card Architecture */
    .card {
      width: 100%;
      background: var(--surface);
      border: 1px solid var(--border);
      border-radius: 14px;
      padding: 16px 14px;
      box-shadow: var(--shadow-sm);
      display: flex;
      flex-direction: column;
      gap: 10px;
      box-sizing: border-box;
      overflow: hidden;
    }

    /* Emergency Alert & Evacuation Order Banner */
    .timer-alert-card {
      width: 100%;
      box-sizing: border-box;
      position: relative;
      background: var(--crimson-bg);
      border: 2px solid var(--crimson);
      border-radius: 14px;
      padding: 16px 14px;
      box-shadow: 0 8px 24px -4px rgba(220, 38, 38, 0.25);
      animation: alert-card-pulse 1.3s infinite alternate ease-in-out;
      overflow: hidden;
    }
    @keyframes alert-card-pulse {
      0% { border-color: rgba(220, 38, 38, 0.7); box-shadow: 0 4px 14px rgba(220, 38, 38, 0.2); }
      100% { border-color: #dc2626; box-shadow: 0 8px 26px rgba(220, 38, 38, 0.35); }
    }
    .timer-header-banner {
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 8px;
      font-size: 0.78rem;
      font-weight: 900;
      letter-spacing: 0.8px;
      text-transform: uppercase;
      color: var(--crimson-text);
      margin-bottom: 12px;
      padding-bottom: 8px;
      border-bottom: 1px dashed rgba(220, 38, 38, 0.3);
      text-align: center;
    }
    .siren-icon { font-size: 1.25rem; animation: siren-bob 0.8s infinite alternate ease-in-out; }
    @keyframes siren-bob { 0% { transform: scale(0.96) rotate(-4deg); } 100% { transform: scale(1.1) rotate(4deg); } }

    /* Single-Column Stack for Mobile Phone Alert */
    .timer-body-flex {
      display: flex;
      flex-direction: column;
      align-items: center;
      text-align: center;
      gap: 12px;
      width: 100%;
      box-sizing: border-box;
    }
    .timer-radial-wrap {
      position: relative;
      width: 114px;
      height: 114px;
      margin: 0 auto;
      flex-shrink: 0;
    }
    .timer-svg {
      width: 100%;
      height: 100%;
      transform: rotate(-90deg);
      display: block;
    }
    .timer-digits-container {
      position: absolute;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
      text-align: center;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      pointer-events: none;
    }
    .timer-seconds-num {
      font-size: 2.3rem;
      font-weight: 900;
      line-height: 1;
      color: var(--crimson-text);
      font-family: ui-monospace, "SF Mono", "Roboto Mono", monospace;
    }
    .timer-unit-text {
      font-size: 0.6rem;
      font-weight: 800;
      letter-spacing: 1px;
      color: var(--crimson);
      text-transform: uppercase;
      margin-top: 2px;
    }
    .timer-details {
      width: 100%;
      display: flex;
      flex-direction: column;
      align-items: center;
      text-align: center;
      gap: 6px;
      box-sizing: border-box;
    }
    .timer-headline {
      font-size: clamp(0.95rem, 4vw, 1.1rem);
      font-weight: 900;
      color: var(--crimson-text);
      letter-spacing: -0.2px;
      line-height: 1.3;
      text-align: center;
    }
    .timer-body-desc {
      font-size: 0.78rem;
      line-height: 1.5;
      color: #7f1d1d;
      text-align: center;
    }
    .timer-action-pill {
      display: inline-flex;
      align-items: center;
      justify-content: center;
      text-align: center;
      padding: 6px 12px;
      min-height: 38px;
      border-radius: 8px;
      background: #fee2e2;
      border: 1.5px solid var(--crimson);
      font-size: 0.72rem;
      font-weight: 900;
      color: var(--crimson-text);
      letter-spacing: 0.4px;
      margin-top: 4px;
    }

    /* Primary Traffic Status Beacon */
    .traffic-beacon {
      width: 100%;
      box-sizing: border-box;
      border-radius: 14px;
      padding: 20px 14px;
      text-align: center;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      gap: 6px;
      border: 2px solid var(--emerald);
      background: var(--emerald-bg);
      box-shadow: var(--shadow-sm);
      transition: all 0.35s ease;
      overflow: hidden;
    }
    .traffic-beacon.state-amber {
      border-color: var(--amber);
      background: var(--amber-bg);
    }
    .traffic-beacon.state-crimson {
      border-color: var(--crimson);
      background: var(--crimson-bg);
      animation: alert-beacon-pulse 1.3s infinite alternate ease-in-out;
    }
    @keyframes alert-beacon-pulse {
      0% { box-shadow: 0 0 10px rgba(220, 38, 38, 0.2); }
      100% { box-shadow: 0 0 25px rgba(220, 38, 38, 0.35); }
    }
    .beacon-icon { font-size: 2.7rem; line-height: 1; margin-bottom: 4px; }
    .beacon-headline {
      font-size: clamp(1.15rem, 4.5vw, 1.35rem);
      font-weight: 900;
      letter-spacing: 0.3px;
      text-transform: uppercase;
      text-align: center;
    }
    .beacon-desc {
      font-size: 0.82rem;
      line-height: 1.5;
      color: var(--text-sub);
      font-weight: 500;
      text-align: center;
    }

    /* Radial Risk Gauge Component */
    .gauge-header-row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      flex-wrap: wrap;
      gap: 6px;
      width: 100%;
      margin-bottom: 4px;
    }
    .card-label {
      font-size: 0.74rem;
      font-weight: 800;
      text-transform: uppercase;
      letter-spacing: 0.8px;
      color: var(--text-muted);
    }
    .risk-state-tag {
      font-size: 0.68rem;
      font-weight: 800;
      padding: 3px 10px;
      border-radius: 9999px;
      letter-spacing: 0.4px;
      text-transform: uppercase;
      transition: all 0.25s ease;
    }
    .tag-safe { background: var(--emerald-bg); color: var(--emerald-text); border: 1px solid var(--emerald-border); }
    .tag-amber { background: var(--amber-bg); color: var(--amber-text); border: 1px solid var(--amber-border); }
    .tag-crimson { background: var(--crimson-bg); color: var(--crimson-text); border: 1px solid var(--crimson-border); }

    .radial-center-wrap {
      display: flex;
      justify-content: center;
      align-items: center;
      position: relative;
      width: 100%;
      margin: 4px auto;
    }
    .gauge-svg {
      width: 100%;
      max-width: 240px;
      height: auto;
      display: block;
      margin: 0 auto;
    }

    /* 3-Zone Segmented Status Bar */
    .segmented-bar-wrap {
      display: flex;
      flex-direction: column;
      gap: 8px;
      margin-top: 6px;
      width: 100%;
    }
    .segmented-track {
      width: 100%;
      height: 14px;
      background: var(--surface-subtle);
      border-radius: 9999px;
      border: 1px solid var(--border-strong);
      position: relative;
      overflow: hidden;
    }
    .segmented-fill {
      height: 100%;
      border-radius: 9999px;
      background: linear-gradient(90deg, #059669 0%, #d97706 52%, #dc2626 85%);
      transition: width 0.35s cubic-bezier(0.4, 0, 0.2, 1);
    }
    .divider-marker {
      position: absolute;
      top: 0;
      bottom: 0;
      width: 2px;
      background: #ffffff;
      box-shadow: 0 0 2px rgba(0, 0, 0, 0.3);
      z-index: 2;
    }
    .div-45 { left: 45%; }
    .div-70 { left: 70%; background: var(--crimson); }

    .segmented-labels {
      display: flex;
      justify-content: space-between;
      font-size: 0.68rem;
      font-weight: 700;
      color: var(--text-muted);
      padding: 0 2px;
    }
    .zone-txt { padding: 1px 4px; border-radius: 4px; transition: color 0.25s, font-weight 0.25s; }
    .zone-active-safe { color: var(--emerald-text); font-weight: 800; }
    .zone-active-mod { color: var(--amber-text); font-weight: 800; }
    .zone-active-crit { color: var(--crimson-text); font-weight: 800; }

    /* Telemetry Info Rows */
    .info-row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 8px;
      padding: 10px 0;
      border-bottom: 1px solid var(--border);
    }
    .info-row:last-child { border-bottom: none; padding-bottom: 0; }
    .info-label { font-size: 0.8rem; color: var(--text-sub); font-weight: 600; }
    .info-val { font-size: 0.88rem; font-weight: 800; text-transform: uppercase; text-align: right; }

    /* Advisory List */
    .rules-list { list-style: none; display: flex; flex-direction: column; gap: 8px; font-size: 0.79rem; color: var(--text-sub); }
    .rules-list li { display: flex; align-items: flex-start; gap: 8px; line-height: 1.45; }

    /* Admin Link Touch Target */
    .admin-link-card {
      width: 100%;
      text-align: center;
      padding: 14px;
      background: var(--surface);
      border: 1px solid var(--border);
      border-radius: 14px;
      box-sizing: border-box;
    }
    .btn-admin-portal {
      display: flex;
      align-items: center;
      justify-content: center;
      text-align: center;
      gap: 8px;
      width: 100%;
      min-height: 50px;
      padding: 14px 18px;
      background: var(--navy-light);
      color: var(--navy-gov);
      border: 1.5px solid var(--navy-border);
      border-radius: 12px;
      text-decoration: none;
      font-size: 0.86rem;
      font-weight: 800;
      text-transform: uppercase;
      letter-spacing: 0.5px;
      touch-action: manipulation;
      box-sizing: border-box;
      transition: all 0.2s ease;
    }
    .btn-admin-portal:hover { background: #dbeafe; }
    .btn-admin-portal:active { transform: scale(0.98); background: #bfdbfe; }
  </style>
</head>
<body>
  <div class="wrapper">
    <!-- Official Agency Header -->
    <div class="gov-header">
      <span class="gov-agency-badge"><span class="beacon-dot"></span> State Emergency Management &bull; Telemetry Hub</span>
      <h1 class="gov-title">Mountain Highway Safety Advisory</h1>
      <p class="gov-subtitle">Autonomous Geotechnical Monitoring &amp; Automated Barrier Shield</p>
    </div>

    <!-- Animated Evacuation Countdown Timer (Reveals upon risk >= 70%) -->
    <div id="publicTimerContainer" class="timer-alert-card" style="display: none;">
      <div class="timer-header-banner">
        <span class="siren-icon">🚨</span>
        <span id="timerBannerTitle">EMERGENCY EVACUATION ORDER</span>
        <span class="siren-icon">🚨</span>
      </div>
      
      <div class="timer-body-flex">
        <div class="timer-radial-wrap">
          <svg class="timer-svg" viewBox="0 0 140 140">
            <circle cx="70" cy="70" r="56" fill="none" stroke="#fee2e2" stroke-width="11" />
            <circle id="timerRingCircle" cx="70" cy="70" r="56" fill="none" stroke="#dc2626" stroke-width="11" stroke-linecap="round" stroke-dasharray="351.86" stroke-dashoffset="0" style="transition: stroke-dashoffset 0.4s linear, stroke 0.3s ease;" />
          </svg>
          <div class="timer-digits-container">
            <div id="publicTimerSecs" class="timer-seconds-num">30</div>
            <div class="timer-unit-text">SECS</div>
          </div>
        </div>
        
        <div class="timer-details">
          <div id="timerHeadline" class="timer-headline">LANDSLIDE IMMINENT AHEAD</div>
          <div id="timerBodyText" class="timer-body-desc">
            Confirmed risk reached critical threshold (<span id="publicTimerRiskPct">70%</span>). Automated road gate is closing in <strong id="timerSecsTxt">30</strong> seconds.
          </div>
          <div id="timerActionBadge" class="timer-action-pill">
            <span>⚡ CLEAR HAZARD CORRIDOR NOW</span>
          </div>
        </div>
      </div>
    </div>

    <!-- Primary Traffic Status Beacon -->
    <div id="trafficBeacon" class="traffic-beacon">
      <div id="beaconIcon" class="beacon-icon">🟢</div>
      <div id="beaconHeadline" class="beacon-headline" style="color: var(--emerald-text);">SAFE</div>
      <div id="beaconDesc" class="beacon-desc">
        Road clear. Geological tilt, seismic tremors, and soil moisture are within safe baseline limits. Travel permitted.
      </div>
    </div>

    <!-- Public Risk Meter (Radial Gauge & 3-Zone Segmented Bar) -->
    <div class="card">
      <div class="gauge-header-row">
        <span class="card-label">Geotechnical Risk Gauge</span>
        <span id="riskMeterBadge" class="risk-state-tag tag-safe">SAFE (0-44%)</span>
      </div>
      
      <div class="radial-center-wrap">
        <svg class="gauge-svg" viewBox="0 0 220 125">
          <defs>
            <linearGradient id="gaugeGradient" x1="0%" y1="0%" x2="100%" y2="0%">
              <stop offset="0%" stop-color="#059669" />
              <stop offset="50%" stop-color="#d97706" />
              <stop offset="85%" stop-color="#dc2626" />
            </linearGradient>
          </defs>
          <path d="M 28 105 A 82 82 0 0 1 192 105" fill="none" stroke="#e2e8f0" stroke-width="14" stroke-linecap="round" />
          <path id="gaugeMeterPath" d="M 28 105 A 82 82 0 0 1 192 105" fill="none" stroke="url(#gaugeGradient)" stroke-width="14" stroke-linecap="round" stroke-dasharray="257.61" stroke-dashoffset="257.61" style="transition: stroke-dashoffset 0.35s ease;" />
          <text id="gaugeScoreText" x="110" y="86" text-anchor="middle" fill="#0f172a" font-size="28" font-weight="900">0%</text>
          <text id="gaugeLevelText" x="110" y="105" text-anchor="middle" fill="#059669" font-size="11" font-weight="800" letter-spacing="0.8">STABLE SLOPE</text>
        </svg>
      </div>

      <div class="segmented-bar-wrap">
        <div class="segmented-track">
          <div id="riskBarFill" class="segmented-fill" style="width: 0%;"></div>
          <div class="divider-marker div-45" title="Moderate Threshold (45%)"></div>
          <div class="divider-marker div-70" title="Emergency Threshold (70%)"></div>
        </div>
        <div class="segmented-labels">
          <span id="zoneSafe" class="zone-txt zone-active-safe">🟢 0% Normal</span>
          <span id="zoneMod" class="zone-txt">🟡 45% Moderate</span>
          <span id="zoneCrit" class="zone-txt">🔴 70% Landslide</span>
        </div>
      </div>
    </div>

    <!-- Highway Status Summary -->
    <div class="card">
      <div class="info-row">
        <span class="info-label">Highway Status</span>
        <span id="txtHighwayStatus" class="info-val" style="color: var(--emerald-text);">Open to Traffic</span>
      </div>
      <div class="info-row">
        <span class="info-label">Automated Road Barrier</span>
        <span id="txtGateStatus" class="info-val" style="color: var(--emerald-text);">Barrier Open</span>
      </div>
      <div class="info-row">
        <span class="info-label">Slope Hazard Assessment</span>
        <span id="txtHazardLevel" class="info-val" style="color: var(--emerald-text);">Low / Normal</span>
      </div>
    </div>

    <!-- Driver Safety Directives -->
    <div class="card">
      <div class="card-label" style="margin-bottom: 6px;">Public Travel Directives</div>
      <ul class="rules-list">
        <li><span>⚠️</span><span>If automated barrier lowers, <strong>DO NOT</strong> bypass. Active rockfall or ground displacement detected ahead.</span></li>
        <li><span>🌧️</span><span>Continuous precipitation accelerates slope saturation. Maintain safe stopping distances.</span></li>
        <li><span>🚨</span><span>Telemetry hub operates 24/7. Obey all visual warnings and acoustic alarms.</span></li>
      </ul>
    </div>

    <!-- Authorized Personnel Access -->
    <div class="admin-link-card">
      <p style="font-size: 0.74rem; color: var(--text-muted); margin-bottom: 10px;">Authorized maintenance &amp; emergency operators:</p>
      <a href="/admin" class="btn-admin-portal">
        <span>🔒</span>
        <span>Authorized Personnel Portal</span>
      </a>
    </div>
  </div>

  <script>
    let targetRisk = 0;
    let smoothedRisk = 0;
    let isPolling = false;

    // Smooth 60fps interpolation loop for fluid gauge movement
    function renderPublicSmooth() {
      smoothedRisk += (targetRisk - smoothedRisk) * 0.15;
      const arcLength = 257.61;
      const targetOffset = arcLength * (1 - (Math.min(100, Math.max(0, smoothedRisk)) / 100));
      const gaugePath = document.getElementById('gaugeMeterPath');
      if (gaugePath) gaugePath.style.strokeDashoffset = targetOffset;
      const gaugeScore = document.getElementById('gaugeScoreText');
      if (gaugeScore) gaugeScore.textContent = Math.round(smoothedRisk) + '%';
      const barFill = document.getElementById('riskBarFill');
      if (barFill) barFill.style.width = Math.min(100, Math.max(0, smoothedRisk)) + '%';
      requestAnimationFrame(renderPublicSmooth);
    }
    requestAnimationFrame(renderPublicSmooth);

    async function updatePublicStatus() {
      if (isPolling) return;
      isPolling = true;
      try {
        const res = await fetch('/api/telemetry');
        if (!res.ok) throw new Error();
        const d = await res.json();

        targetRisk = Math.min(100, Math.max(0, d.risk.percentage));
        const isEmergency = (d.emergency.active || targetRisk >= 70);
        const isClosed = (d.gate.status === "CLOSED" || isEmergency);
        const isModerate = (targetRisk >= 45 && !isClosed);

        const beacon = document.getElementById('trafficBeacon');
        const icon = document.getElementById('beaconIcon');
        const headline = document.getElementById('beaconHeadline');
        const desc = document.getElementById('beaconDesc');
        const hwStatus = document.getElementById('txtHighwayStatus');
        const gateStatus = document.getElementById('txtGateStatus');
        const hazardLevel = document.getElementById('txtHazardLevel');

        const gaugeLevel = document.getElementById('gaugeLevelText');
        const riskBadge = document.getElementById('riskMeterBadge');
        const zoneSafe = document.getElementById('zoneSafe');
        const zoneMod = document.getElementById('zoneMod');
        const zoneCrit = document.getElementById('zoneCrit');

        zoneSafe.className = 'zone-txt';
        zoneMod.className = 'zone-txt';
        zoneCrit.className = 'zone-txt';

        if (targetRisk >= 70 || isClosed) {
          riskBadge.className = 'risk-state-tag tag-crimson';
          riskBadge.textContent = 'CRITICAL (70-100%)';
          gaugeLevel.textContent = 'LANDSLIDE AHEAD';
          gaugeLevel.setAttribute('fill', 'var(--crimson-text)');
          zoneCrit.className = 'zone-txt zone-active-crit';
        } else if (targetRisk >= 45) {
          riskBadge.className = 'risk-state-tag tag-amber';
          riskBadge.textContent = 'MODERATE (45-69%)';
          gaugeLevel.textContent = 'MODERATE RISK';
          gaugeLevel.setAttribute('fill', 'var(--amber-text)');
          zoneMod.className = 'zone-txt zone-active-mod';
        } else {
          riskBadge.className = 'risk-state-tag tag-safe';
          riskBadge.textContent = 'SAFE (0-44%)';
          gaugeLevel.textContent = 'STABLE SLOPE';
          gaugeLevel.setAttribute('fill', 'var(--emerald-text)');
          zoneSafe.className = 'zone-txt zone-active-safe';
        }

        // Animated Evacuation Countdown Timer
        const timerContainer = document.getElementById('publicTimerContainer');
        const timerRing = document.getElementById('timerRingCircle');
        const timerSecs = document.getElementById('publicTimerSecs');
        const timerSecsTxt = document.getElementById('timerSecsTxt');
        const timerRiskPct = document.getElementById('publicTimerRiskPct');
        const timerHeadline = document.getElementById('timerHeadline');
        const timerBody = document.getElementById('timerBodyText');
        const timerAction = document.getElementById('timerActionBadge');
        const timerBanner = document.getElementById('timerBannerTitle');

        if (isEmergency || (d.gate.status === "CLOSED" && d.emergency.countdown === 0)) {
          timerContainer.style.display = 'block';
          const dur = d.emergency.countdown_duration || 30;
          const rem = Math.max(0, d.emergency.countdown !== undefined ? d.emergency.countdown : 0);
          const ringCircumference = 351.86;
          const progress = dur > 0 ? (rem / dur) : 0;
          timerRing.style.strokeDashoffset = ringCircumference * (1 - progress);
          timerSecs.textContent = rem;
          if (timerSecsTxt) timerSecsTxt.textContent = rem;
          if (timerRiskPct) timerRiskPct.textContent = Math.round(targetRisk) + '%';

          if (d.gate.status === "CLOSED" || rem === 0) {
            timerRing.setAttribute('stroke', '#dc2626');
            timerBanner.textContent = 'ROAD CLOSURE ENFORCED';
            timerHeadline.textContent = 'BARRIER DEPLOYED — PASSAGE BLOCKED';
            timerBody.innerHTML = 'Highway is closed due to confirmed slope instability (' + Math.round(targetRisk) + '% risk). <strong>Do not attempt crossing.</strong> Emergency crews on scene.';
            timerAction.innerHTML = '<span>⛔ ROAD ENTRY PROHIBITED</span>';
          } else {
            timerRing.setAttribute('stroke', rem <= 10 ? '#dc2626' : '#ea580c');
            timerBanner.textContent = 'EMERGENCY EVACUATION ORDER';
            timerHeadline.textContent = 'LANDSLIDE IMMINENT AHEAD';
            timerBody.innerHTML = 'Risk reached critical threshold (' + Math.round(targetRisk) + '%). Automated gate is closing in <strong style="color:var(--crimson-text);">' + rem + '</strong> seconds. Clear the corridor immediately!';
            timerAction.innerHTML = '<span>⚡ CLEAR HAZARD CORRIDOR NOW</span>';
          }
        } else {
          timerContainer.style.display = 'none';
        }

        // Primary Traffic Beacon Update
        if (isClosed) {
          beacon.className = 'traffic-beacon state-crimson';
          icon.textContent = '⛔';
          headline.textContent = 'LANDSLIDE AHEAD - ROAD CLOSED';
          headline.style.color = 'var(--crimson-text)';
          desc.textContent = 'CRITICAL DANGER: Severe slope displacement detected. Automated barrier is deployed. Do not enter!';
          hwStatus.textContent = 'ROAD CLOSED';
          hwStatus.style.color = 'var(--crimson-text)';
          gateStatus.textContent = (d.gate.status === "CLOSED" ? 'BARRIER BLOCKED' : 'BARRIER CLOSING...');
          gateStatus.style.color = 'var(--crimson-text)';
          hazardLevel.textContent = 'CRITICAL HAZARD';
          hazardLevel.style.color = 'var(--crimson-text)';
        } else if (isModerate) {
          beacon.className = 'traffic-beacon state-amber';
          icon.textContent = '⚠️';
          headline.textContent = 'MODERATE RISK';
          headline.style.color = 'var(--amber-text)';
          desc.textContent = 'Elevated ground tremors or soil moisture detected. Drive cautiously and watch for falling stones.';
          hwStatus.textContent = 'Travel With Caution';
          hwStatus.style.color = 'var(--amber-text)';
          gateStatus.textContent = 'Barrier Open';
          gateStatus.style.color = 'var(--emerald-text)';
          hazardLevel.textContent = 'Moderate Activity';
          hazardLevel.style.color = 'var(--amber-text)';
        } else {
          beacon.className = 'traffic-beacon';
          icon.textContent = '🟢';
          headline.textContent = 'SAFE';
          headline.style.color = 'var(--emerald-text)';
          desc.textContent = 'Road clear. Geological tilt, seismic tremors, and soil moisture are within safe baseline limits. Travel permitted.';
          hwStatus.textContent = 'Open to Traffic';
          hwStatus.style.color = 'var(--emerald-text)';
          gateStatus.textContent = 'Barrier Open';
          gateStatus.style.color = 'var(--emerald-text)';
          hazardLevel.textContent = 'Low / Normal';
          hazardLevel.style.color = 'var(--emerald-text)';
        }
      } catch (e) {
      } finally {
        isPolling = false;
      }
    }
    // Reliable SoftAP polling interval
    setInterval(updatePublicStatus, 320);
    updatePublicStatus();
  </script>
</body>
</html>
)rawliteral";

// ---------------------- 2. PROTECTED ADMIN PANEL (/admin) ----------------
const char ADMIN_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no, viewport-fit=cover">
  <meta name="theme-color" content="#1e3a8a">
  <meta name="apple-mobile-web-app-capable" content="yes">
  <meta name="apple-mobile-web-app-status-bar-style" content="default">
  <title>Emergency Operations Center — Field Telemetry &amp; Actuation Hub</title>
  <style>
    :root {
      --bg: #f8fafc;
      --surface: #ffffff;
      --surface-subtle: #f1f5f9;
      --surface-elevated: #ffffff;
      --border: #e2e8f0;
      --border-strong: #cbd5e1;
      --border-accent: #bfdbfe;
      --text: #0f172a;
      --text-dim: #334155;
      --text-muted: #64748b;
      --navy-gov: #1e3a8a;
      --navy-light: #eff6ff;
      --navy-border: #bfdbfe;
      --emerald: #059669;
      --emerald-bg: #ecfdf5;
      --emerald-border: #a7f3d0;
      --emerald-text: #065f46;
      --amber: #d97706;
      --amber-bg: #fffbeb;
      --amber-border: #fde68a;
      --amber-text: #92400e;
      --crimson: #dc2626;
      --crimson-bg: #fef2f2;
      --crimson-border: #fca5a5;
      --crimson-text: #991b1b;
      --blue: #2563eb;
      --blue-light: #eff6ff;
      --shadow-sm: 0 1px 3px rgba(15, 23, 42, 0.06), 0 1px 2px rgba(15, 23, 42, 0.04);
      --shadow-md: 0 4px 6px -1px rgba(15, 23, 42, 0.07), 0 2px 4px -2px rgba(15, 23, 42, 0.05);
      --shadow-lg: 0 10px 15px -3px rgba(15, 23, 42, 0.08), 0 4px 6px -4px rgba(15, 23, 42, 0.04);
    }
    *, *::before, *::after {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
      -webkit-tap-highlight-color: transparent;
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    }
    body {
      background-color: var(--bg);
      background-image: 
        radial-gradient(at 0% 0%, rgba(224, 231, 255, 0.5) 0px, transparent 50%),
        radial-gradient(at 100% 0%, rgba(241, 245, 249, 0.8) 0px, transparent 50%);
      color: var(--text);
      min-height: 100vh;
      min-height: -webkit-fill-available;
      width: 100%;
      max-width: 100vw;
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: max(10px, env(safe-area-inset-top)) 10px max(24px, env(safe-area-inset-bottom));
      overflow-x: hidden;
      -webkit-font-smoothing: antialiased;
      font-feature-settings: "tnum" 1;
    }

    /* Passkey Gatekeeper Card */
    .login-container {
      width: 100%;
      max-width: 440px;
      margin: 20px auto 0;
      background: var(--surface);
      border: 1px solid var(--border);
      border-top: 4px solid var(--navy-gov);
      border-radius: 14px;
      padding: 26px 18px;
      text-align: center;
      box-shadow: var(--shadow-md);
      box-sizing: border-box;
    }
    .login-icon { font-size: 2.8rem; margin-bottom: 12px; }
    .login-title { font-size: 1.25rem; font-weight: 800; margin-bottom: 6px; letter-spacing: -0.3px; color: var(--text); }
    .login-desc { font-size: 0.82rem; color: var(--text-dim); margin-bottom: 20px; line-height: 1.45; }
    .input-field {
      width: 100%;
      min-height: 52px;
      padding: 14px 16px;
      background: var(--surface-subtle);
      border: 1px solid var(--border-strong);
      border-radius: 12px;
      color: var(--text);
      font-size: 1.1rem;
      text-align: center;
      letter-spacing: 2px;
      outline: none;
      margin-bottom: 14px;
      touch-action: manipulation;
      box-sizing: border-box;
      transition: border-color 0.2s, box-shadow 0.2s;
    }
    .input-field:focus { border-color: var(--blue); box-shadow: 0 0 0 3px rgba(37, 99, 235, 0.15); background: #ffffff; }
    .btn-login {
      width: 100%;
      min-height: 52px;
      padding: 14px;
      background: linear-gradient(135deg, #1e3a8a 0%, #1d4ed8 100%);
      color: #fff;
      border: none;
      border-radius: 12px;
      font-size: 0.92rem;
      font-weight: 800;
      text-transform: uppercase;
      letter-spacing: 0.8px;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      text-align: center;
      touch-action: manipulation;
      box-sizing: border-box;
      transition: all 0.2s ease;
      box-shadow: var(--shadow-sm);
    }
    .btn-login:active { transform: scale(0.98); }
    .login-err { color: var(--crimson-text); font-size: 0.8rem; font-weight: 700; margin-top: 12px; display: none; background: var(--crimson-bg); padding: 10px; border-radius: 8px; border: 1px solid var(--crimson-border); }

    /* Admin Console Header & Nav */
    .admin-console {
      width: 100%;
      max-width: 500px;
      margin: 0 auto;
      display: none;
      flex-direction: column;
      gap: 12px;
      box-sizing: border-box;
    }
    .top-bar {
      width: 100%;
      display: flex;
      flex-direction: column;
      gap: 10px;
      padding: 12px 14px;
      background: var(--surface);
      border: 1px solid var(--border);
      border-radius: 12px;
      box-shadow: var(--shadow-sm);
      box-sizing: border-box;
    }
    .top-titles {
      display: flex;
      align-items: center;
      justify-content: space-between;
      flex-wrap: wrap;
      gap: 6px;
      width: 100%;
    }
    .top-titles h1 { font-size: 0.95rem; font-weight: 800; color: var(--text); display: flex; align-items: center; gap: 8px; }
    .admin-tag { background: var(--crimson-bg); color: var(--crimson-text); border: 1px solid var(--crimson-border); padding: 2px 7px; border-radius: 6px; font-size: 0.65rem; font-weight: 800; }
    .top-actions {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
      width: 100%;
    }
    .btn-touch {
      min-height: 46px;
      width: 100%;
      display: inline-flex;
      align-items: center;
      justify-content: center;
      text-align: center;
      background: var(--surface-subtle);
      color: var(--text-dim);
      border: 1px solid var(--border);
      border-radius: 10px;
      padding: 10px 12px;
      font-size: 0.8rem;
      font-weight: 800;
      cursor: pointer;
      text-decoration: none;
      touch-action: manipulation;
      box-sizing: border-box;
      transition: all 0.2s ease;
    }
    .btn-touch:hover { background: var(--border); color: var(--text); }
    .btn-touch:active { transform: scale(0.97); }

    /* Standard Digital Lockdown Clock Banner - Mobile Single Column */
    .admin-emergency-banner {
      width: 100%;
      background: var(--crimson-bg);
      border: 2px solid var(--crimson);
      border-radius: 14px;
      padding: 16px 14px;
      display: flex;
      flex-direction: column;
      align-items: center;
      text-align: center;
      gap: 12px;
      box-shadow: 0 4px 15px rgba(220, 38, 38, 0.2);
      animation: alert-card-pulse 1.2s infinite alternate ease-in-out;
      box-sizing: border-box;
    }
    @keyframes alert-card-pulse {
      0% { border-color: rgba(220, 38, 38, 0.7); box-shadow: 0 4px 12px rgba(220, 38, 38, 0.15); }
      100% { border-color: #dc2626; box-shadow: 0 8px 24px rgba(220, 38, 38, 0.3); }
    }
    .banner-left-wrap {
      display: flex;
      flex-direction: column;
      align-items: center;
      text-align: center;
      gap: 6px;
      width: 100%;
    }
    .banner-siren { font-size: 2.2rem; animation: siren-bob 0.8s infinite alternate ease-in-out; }
    @keyframes siren-bob { 0% { transform: scale(0.96) rotate(-4deg); } 100% { transform: scale(1.1) rotate(4deg); } }
    .banner-title { font-size: 1.02rem; font-weight: 900; color: var(--crimson-text); letter-spacing: 0.3px; line-height: 1.3; }
    .banner-subtext { font-size: 0.76rem; color: #7f1d1d; line-height: 1.4; }
    .admin-digital-clock-box {
      background: #ffffff;
      border: 2px solid var(--crimson);
      border-radius: 12px;
      padding: 8px 16px;
      text-align: center;
      width: 100%;
      max-width: 200px;
      margin: 2px auto;
      box-shadow: var(--shadow-sm);
      box-sizing: border-box;
    }
    .clock-label { font-size: 0.64rem; font-weight: 900; color: var(--crimson-text); letter-spacing: 1px; text-transform: uppercase; }
    .clock-digits { font-size: 2.1rem; font-weight: 900; color: var(--crimson); letter-spacing: 2px; font-family: ui-monospace, SFMono-Regular, "Roboto Mono", Menlo, monospace; line-height: 1.1; margin-top: 2px; }
    .btn-emergency-mute-action {
      width: 100%;
      min-height: 48px;
      display: flex;
      align-items: center;
      justify-content: center;
      text-align: center;
      gap: 8px;
      font-weight: 800;
      font-size: 0.88rem;
      border-radius: 10px;
      border: 2px solid var(--crimson);
      background: #ffffff;
      color: var(--crimson-text);
      cursor: pointer;
      touch-action: manipulation;
      box-shadow: var(--shadow-sm);
      box-sizing: border-box;
    }

    /* Exclusive Manual Barrier Controls - Single Column Stack */
    .exclusive-ctrl-card {
      width: 100%;
      background: var(--surface);
      border: 1px solid var(--border);
      border-top: 4px solid var(--navy-gov);
      border-radius: 14px;
      padding: 16px 14px;
      box-shadow: var(--shadow-sm);
      box-sizing: border-box;
    }
    .exclusive-head {
      font-size: 0.78rem;
      font-weight: 800;
      text-transform: uppercase;
      color: var(--navy-gov);
      letter-spacing: 0.8px;
      margin-bottom: 12px;
      display: flex;
      flex-direction: column;
      gap: 8px;
      align-items: flex-start;
      width: 100%;
    }
    .exclusive-pills {
      display: flex;
      gap: 6px;
      align-items: center;
      flex-wrap: wrap;
      width: 100%;
    }
    .exclusive-btn-row {
      display: flex;
      flex-direction: column;
      gap: 10px;
      width: 100%;
      box-sizing: border-box;
    }
    .btn-actuate {
      width: 100%;
      min-height: 52px;
      display: flex;
      align-items: center;
      justify-content: center;
      text-align: center;
      gap: 8px;
      color: #fff;
      border: none;
      border-radius: 12px;
      padding: 14px 16px;
      font-size: 0.88rem;
      font-weight: 800;
      text-transform: uppercase;
      letter-spacing: 0.5px;
      cursor: pointer;
      touch-action: manipulation;
      transition: all 0.2s ease;
      box-shadow: var(--shadow-sm);
      box-sizing: border-box;
    }
    .btn-action-open { background: linear-gradient(135deg, #059669 0%, #047857 100%); }
    .btn-action-close { background: linear-gradient(135deg, #dc2626 0%, #b91c1c 100%); }
    .btn-actuate:active { transform: scale(0.97); }

    /* Single-Column Stack Layout for Mobile */
    .section-row {
      display: flex;
      flex-direction: column;
      gap: 12px;
      width: 100%;
      box-sizing: border-box;
    }
    .card {
      width: 100%;
      background: var(--surface);
      border: 1px solid var(--border);
      border-radius: 14px;
      padding: 16px 14px;
      box-shadow: var(--shadow-sm);
      display: flex;
      flex-direction: column;
      gap: 12px;
      box-sizing: border-box;
      overflow: hidden;
    }
    .card-head {
      display: flex;
      justify-content: space-between;
      align-items: center;
      flex-wrap: wrap;
      gap: 6px;
      width: 100%;
    }
    .card-title { font-size: 0.76rem; font-weight: 800; text-transform: uppercase; letter-spacing: 0.8px; color: var(--text-muted); }
    .card-badge { font-size: 0.66rem; font-weight: 700; padding: 2px 8px; border-radius: 6px; background: var(--surface-subtle); color: var(--text-dim); border: 1px solid var(--border); }

    /* Admin Risk Matrix Styles - Mobile Vertical Architecture */
    .admin-risk-master {
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 12px;
      padding: 14px 12px;
      background: var(--surface-subtle);
      border-radius: 12px;
      border: 1px solid var(--border);
      width: 100%;
      box-sizing: border-box;
    }
    .admin-risk-main-val {
      display: flex;
      flex-direction: column;
      align-items: center;
      text-align: center;
      gap: 4px;
      width: 100%;
    }
    .risk-score-num { font-size: 2.6rem; font-weight: 900; color: var(--text); line-height: 1; text-align: center; }
    .risk-score-meta { display: flex; flex-direction: column; align-items: center; text-align: center; }
    .meta-label { font-size: 0.65rem; font-weight: 700; color: var(--text-muted); text-transform: uppercase; }
    .meta-status { font-size: 0.84rem; font-weight: 800; text-transform: uppercase; }
    .admin-risk-secondary {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 6px;
      width: 100%;
      padding-top: 10px;
      border-top: 1px solid var(--border);
      text-align: center;
      box-sizing: border-box;
    }
    .sec-item { display: flex; flex-direction: column; align-items: center; text-align: center; gap: 3px; }
    .sec-lbl { font-size: 0.62rem; color: var(--text-muted); text-transform: uppercase; font-weight: 700; white-space: nowrap; }
    .sec-val { font-size: 0.92rem; font-weight: 800; color: var(--text); }

    /* Precision Gauge Segment Bar with Guaranteed Clear Spacing */
    .admin-gauge-bar-wrap {
      position: relative;
      width: 100%;
      padding: 8px 0 24px;
      box-sizing: border-box;
    }
    .admin-gauge-bar {
      width: 100%;
      height: 14px;
      background: var(--surface-subtle);
      border-radius: 9999px;
      border: 1px solid var(--border-strong);
      position: relative;
      overflow: visible;
    }
    .admin-gauge-fill {
      height: 100%;
      border-radius: 9999px;
      background: linear-gradient(90deg, #059669 0%, #d97706 50%, #dc2626 85%);
      transition: width 0.3s ease;
    }
    .gauge-marker {
      position: absolute;
      top: -4px;
      bottom: -4px;
      width: 2px;
      background: var(--text);
      z-index: 3;
    }
    .marker-45 { left: 45%; background: var(--amber); }
    .marker-70 { left: 70%; background: var(--crimson); }
    .marker-tag {
      position: absolute;
      bottom: -18px;
      left: 50%;
      transform: translateX(-50%);
      font-size: 0.58rem;
      font-weight: 800;
      white-space: nowrap;
      color: var(--text-muted);
    }

    /* Sensor Contribution Breakdown Rows */
    .contribution-section { display: flex; flex-direction: column; gap: 8px; width: 100%; margin-top: 4px; }
    .contrib-title { font-size: 0.72rem; font-weight: 800; text-transform: uppercase; letter-spacing: 0.5px; color: var(--text-muted); }
    .contrib-row {
      background: var(--surface-subtle);
      border-radius: 10px;
      padding: 9px 12px;
      border: 1px solid var(--border);
      display: flex;
      flex-direction: column;
      gap: 6px;
      width: 100%;
      box-sizing: border-box;
    }
    .contrib-info { display: flex; justify-content: space-between; align-items: center; font-size: 0.76rem; width: 100%; }
    .contrib-name { font-weight: 700; color: var(--text); }
    .contrib-name small { color: var(--text-muted); font-size: 0.65rem; margin-left: 4px; }
    .contrib-val { font-weight: 800; color: var(--text); text-align: right; }
    .contrib-pts { font-size: 0.68rem; color: var(--blue); margin-left: 4px; }
    .contrib-track { width: 100%; height: 7px; background: #e2e8f0; border-radius: 4px; overflow: hidden; }
    .contrib-fill { height: 100%; border-radius: 4px; transition: width 0.3s ease; }

    /* Synergy Matrix Badges */
    .synergy-status-wrap {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 6px;
      width: 100%;
      margin-top: 4px;
      box-sizing: border-box;
    }
    .syn-badge {
      min-height: 38px;
      display: flex;
      align-items: center;
      justify-content: center;
      text-align: center;
      font-size: 0.65rem;
      font-weight: 700;
      padding: 6px 8px;
      border-radius: 8px;
      text-transform: uppercase;
      letter-spacing: 0.3px;
      box-sizing: border-box;
    }
    .syn-inactive { background: var(--surface-subtle); color: var(--text-muted); border: 1px solid var(--border); }
    .syn-active { background: var(--crimson-bg); color: var(--crimson-text); border: 1px solid var(--crimson-border); font-weight: 800; }

    /* Configurable Timer Section */
    .config-current-row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 12px 14px;
      background: var(--surface-subtle);
      border-radius: 10px;
      border: 1px solid var(--border);
      width: 100%;
      box-sizing: border-box;
    }
    .preset-btn-grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 8px;
      width: 100%;
      box-sizing: border-box;
    }
    .btn-preset {
      min-height: 48px;
      padding: 10px 8px;
      background: var(--surface-subtle);
      border: 1px solid var(--border);
      border-radius: 10px;
      color: var(--text-dim);
      font-size: 0.82rem;
      font-weight: 700;
      cursor: pointer;
      touch-action: manipulation;
      transition: all 0.2s ease;
      text-align: center;
      display: flex;
      align-items: center;
      justify-content: center;
      box-sizing: border-box;
    }
    .btn-preset:hover { background: var(--border); color: var(--text); }
    .btn-preset:active { transform: scale(0.96); }
    .btn-preset.active { background: var(--blue-light); border-color: var(--blue); color: var(--blue); font-weight: 800; }
    .config-form-row {
      display: flex;
      flex-direction: column;
      gap: 8px;
      width: 100%;
      box-sizing: border-box;
    }
    .config-input-wrap {
      width: 100%;
      position: relative;
      display: flex;
      align-items: center;
      box-sizing: border-box;
    }
    .config-num-input {
      width: 100%;
      min-height: 50px;
      padding: 12px 48px 12px 14px;
      background: var(--surface-subtle);
      border: 1px solid var(--border-strong);
      border-radius: 10px;
      color: var(--text);
      font-size: 1rem;
      font-weight: 700;
      outline: none;
      touch-action: manipulation;
      box-sizing: border-box;
    }
    .config-num-input:focus { border-color: var(--blue); background: #ffffff; }
    .input-unit { position: absolute; right: 14px; font-size: 0.72rem; color: var(--text-muted); text-transform: uppercase; font-weight: 700; }
    .btn-save-cfg {
      width: 100%;
      min-height: 50px;
      padding: 12px 18px;
      background: linear-gradient(135deg, #1e3a8a 0%, #1d4ed8 100%);
      color: #fff;
      border: none;
      border-radius: 10px;
      font-size: 0.88rem;
      font-weight: 800;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      text-align: center;
      touch-action: manipulation;
      transition: all 0.2s ease;
      box-shadow: var(--shadow-sm);
      box-sizing: border-box;
    }
    .btn-save-cfg:hover { background: linear-gradient(135deg, #172554 0%, #1e40af 100%); }
    .btn-save-cfg:active { transform: scale(0.97); }
    .cfg-status-msg {
      font-size: 0.75rem;
      font-weight: 700;
      padding: 10px 12px;
      border-radius: 8px;
      line-height: 1.4;
      width: 100%;
      box-sizing: border-box;
    }
    .cfg-status-msg.success { background: var(--emerald-bg); border: 1px solid var(--emerald-border); color: var(--emerald-text); }
    .cfg-status-msg.error { background: var(--crimson-bg); border: 1px solid var(--crimson-border); color: var(--crimson-text); }
    .cfg-status-msg.pending { background: var(--blue-light); border: 1px solid var(--border-accent); color: var(--blue); }

    /* Road Gate Vector Visualizer */
    .gate-scene {
      width: 100%;
      height: 130px;
      background: #f1f5f9;
      border-radius: 12px;
      border: 1px solid var(--border);
      position: relative;
      overflow: hidden;
      box-sizing: border-box;
    }
    #gateSvg { width: 100%; height: 100%; display: block; }

    /* Artificial Horizon & Scope */
    .horizon-box { display: flex; flex-direction: column; align-items: center; gap: 12px; width: 100%; }
    .horizon-frame {
      width: 180px;
      height: 180px;
      border-radius: 50%;
      border: 4px solid #334155;
      background: #0f172a;
      box-shadow: 0 4px 12px rgba(0, 0, 0, 0.2);
      position: relative;
      overflow: hidden;
    }
    #horizonCanvas { width: 100%; height: 100%; display: block; }
    .horizon-reticle { position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); pointer-events: none; width: 90px; display: flex; align-items: center; justify-content: space-between; }
    .reticle-bar { width: 30px; height: 4px; background: #facc15; border-radius: 2px; }
    .reticle-pip { width: 8px; height: 8px; border-radius: 50%; background: #facc15; border: 2px solid #000; }
    .btn-cal {
      min-height: 46px;
      width: 100%;
      max-width: 240px;
      background: var(--surface-subtle);
      color: var(--navy-gov);
      border: 1.5px solid var(--border-strong);
      border-radius: 10px;
      padding: 10px 16px;
      font-size: 0.82rem;
      font-weight: 800;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      text-align: center;
      touch-action: manipulation;
      transition: all 0.2s ease;
      box-sizing: border-box;
    }
    .btn-cal:hover { background: var(--navy-light); border-color: var(--navy-border); }
    .btn-cal:active { transform: scale(0.97); }

    .scope-box {
      width: 100%;
      height: 120px;
      background: #0f172a;
      border-radius: 10px;
      border: 1px solid var(--border);
      position: relative;
      overflow: hidden;
      box-sizing: border-box;
    }
    #scopeCanvas { width: 100%; height: 100%; display: block; }

    /* Soil Liquid Tank */
    .soil-tank-wrap { display: flex; align-items: center; gap: 14px; width: 100%; box-sizing: border-box; }
    .liquid-tank { width: 55px; height: 110px; border-radius: 10px; border: 2px solid #94a3b8; background: #f1f5f9; position: relative; overflow: hidden; flex-shrink: 0; }
    .liquid-fill { position: absolute; bottom: 0; left: 0; right: 0; height: 0%; background: linear-gradient(180deg, #38bdf8 0%, #0284c7 100%); transition: height 0.4s ease; }
    .liquid-ticks { position: absolute; top: 0; bottom: 0; right: 3px; width: 6px; display: flex; flex-direction: column; justify-content: space-between; padding: 4px 0; }
    .tank-tick { width: 4px; height: 1px; background: rgba(0, 0, 0, 0.25); }

    .metric-grid { display: grid; grid-template-columns: repeat(2, 1fr); gap: 8px; width: 100%; box-sizing: border-box; }
    .metric-tile { background: var(--surface-subtle); border-radius: 10px; padding: 8px 10px; border: 1px solid var(--border); display: flex; flex-direction: column; gap: 2px; }
    .m-lbl { font-size: 0.62rem; text-transform: uppercase; font-weight: 700; color: var(--text-muted); }
    .m-val { font-size: 0.95rem; font-weight: 800; color: var(--text); }

    /* Historical Data & Trend Chart Component - Mobile Responsive */
    .chart-container-box {
      width: 100%;
      height: 190px;
      background: #ffffff;
      border: 1px solid var(--border);
      border-radius: 10px;
      position: relative;
      overflow: hidden;
      box-sizing: border-box;
    }
    #historyChartCanvas { width: 100%; height: 100%; display: block; }
    .chart-legend {
      display: flex;
      flex-wrap: wrap;
      gap: 6px 12px;
      align-items: center;
      font-size: 0.68rem;
      font-weight: 700;
      color: var(--text-dim);
      padding: 4px 0 2px;
      width: 100%;
    }
    .legend-item { display: inline-flex; align-items: center; gap: 5px; }
    .legend-dot { width: 10px; height: 10px; border-radius: 2px; }

    /* Strict Zero Horizontal Scrolling for Mobile Data Containers */
    .history-table-container {
      width: 100%;
      max-height: 480px;
      overflow-y: auto;
      overflow-x: hidden;
      border: 1px solid var(--border);
      border-radius: 10px;
      background: #ffffff;
      -webkit-overflow-scrolling: touch;
      box-sizing: border-box;
    }
    .history-table {
      width: 100%;
      border-collapse: collapse;
      box-sizing: border-box;
    }
    .history-table thead {
      display: none;
    }
    .history-table tbody {
      display: flex;
      flex-direction: column;
      gap: 8px;
      padding: 8px;
      width: 100%;
      box-sizing: border-box;
    }
    .history-table tr {
      display: flex;
      flex-direction: column;
      gap: 6px;
      background: var(--surface-subtle);
      border: 1px solid var(--border);
      border-radius: 10px;
      padding: 10px 12px;
      width: 100%;
      box-sizing: border-box;
    }
    .history-table td {
      border: none;
      padding: 0;
      white-space: normal;
      word-break: break-word;
      width: 100%;
      box-sizing: border-box;
    }

    /* Government Command Tabs - Mobile Segmented Grid */
    .admin-tab-nav {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 6px;
      width: 100%;
      box-sizing: border-box;
      margin-bottom: 6px;
    }
    .tab-nav-btn {
      width: 100%;
      min-height: 52px;
      padding: 8px 4px;
      background: var(--surface-subtle);
      border: 1.5px solid var(--border);
      border-radius: 10px;
      color: var(--text-dim);
      font-size: 0.72rem;
      font-weight: 700;
      cursor: pointer;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      text-align: center;
      gap: 3px;
      transition: all 0.18s ease;
      touch-action: manipulation;
      box-sizing: border-box;
    }
    .tab-nav-btn:hover { background: #e2e8f0; color: var(--text); }
    .tab-nav-btn.active {
      background: var(--navy-light);
      color: var(--navy-gov);
      border-color: var(--navy-gov);
      font-weight: 900;
      box-shadow: var(--shadow-sm);
    }
    .tab-counter-pill {
      font-size: 0.62rem;
      font-weight: 800;
      padding: 1px 6px;
      border-radius: 9999px;
      background: var(--border);
      color: var(--text-muted);
    }
    .tab-nav-btn.active .tab-counter-pill {
      background: #bfdbfe;
      color: var(--navy-gov);
      border: 1px solid var(--navy-border);
    }
    .tab-pane {
      display: none;
      flex-direction: column;
      gap: 12px;
      width: 100%;
      box-sizing: border-box;
    }
    .tab-pane.active { display: flex; }

    /* Municipal Audit Trail & Event Log Toolbar - Mobile Stack */
    .audit-toolbar {
      display: flex;
      flex-direction: column;
      gap: 8px;
      background: var(--surface-subtle);
      border: 1px solid var(--border);
      border-radius: 10px;
      padding: 10px 12px;
      width: 100%;
      box-sizing: border-box;
    }
    .audit-search-wrap {
      display: flex;
      align-items: center;
      gap: 8px;
      background: #ffffff;
      border: 1px solid var(--border-strong);
      border-radius: 8px;
      padding: 8px 12px;
      width: 100%;
      box-sizing: border-box;
    }
    .audit-search-field {
      border: none;
      outline: none;
      font-size: 0.82rem;
      color: var(--text);
      background: transparent;
      width: 100%;
    }
    .audit-filter-chips {
      display: flex;
      gap: 5px;
      flex-wrap: wrap;
      width: 100%;
    }
    .filter-chip {
      background: #ffffff;
      border: 1px solid var(--border);
      border-radius: 8px;
      padding: 6px 10px;
      font-size: 0.72rem;
      font-weight: 700;
      cursor: pointer;
      color: var(--text-dim);
      transition: all 0.15s ease;
      touch-action: manipulation;
      min-height: 38px;
      display: inline-flex;
      align-items: center;
      justify-content: center;
    }
    .filter-chip:hover { border-color: var(--navy-gov); color: var(--text); }
    .filter-chip.active { background: var(--navy-gov); color: #ffffff; border-color: var(--navy-gov); }
    .filter-chip.chip-crit.active { background: var(--crimson); border-color: var(--crimson); color: #ffffff; }
    .filter-chip.chip-warn.active { background: var(--amber); border-color: var(--amber); color: #ffffff; }
    .filter-chip.chip-info.active { background: #0284c7; border-color: #0284c7; color: #ffffff; }
    .filter-chip.chip-succ.active { background: var(--emerald); border-color: var(--emerald); color: #ffffff; }
    .btn-compact { min-height: 42px; padding: 8px 12px; font-size: 0.76rem; font-weight: 800; border-radius: 8px; }
  </style>
</head>
<body>
  <!-- PASSKEY LOGIN GATEKEEPER -->
  <div id="loginGate" class="login-container">
    <div class="login-icon">🔒</div>
    <div class="login-title">Emergency Command Portal</div>
    <div class="login-desc">Authorized personnel access only. Authenticate to view live multi-sensor telemetry, configure timer settings, and operate the road gate.</div>
    <input type="password" id="passkeyInput" class="input-field" placeholder="Enter Admin Passkey" autofocus>
    <button class="btn-login" onclick="verifyAdminPasskey()">Unlock Console</button>
    <div id="loginErr" class="login-err">Invalid Passkey! Access Denied.</div>
    <div style="margin-top: 18px;"><a href="/" class="btn-touch" style="min-height: 44px;">&larr; Back to Public Road Status</a></div>
  </div>

  <!-- PROTECTED ADMIN COMMAND CONSOLE -->
  <div id="adminConsole" class="admin-console">
    <div class="top-bar">
      <div class="top-titles">
        <h1><span>OPERATOR CONSOLE</span></h1>
        <span class="admin-tag">RESTRICTED ACCESS</span>
      </div>
      <div class="top-actions">
        <a href="/" class="btn-touch">&larr; Public View</a>
        <button class="btn-touch" onclick="logoutAdmin()">Logout</button>
      </div>
    </div>

    <!-- Active Emergency Digital Timer Banner -->
    <div id="adminEmergencyBanner" class="admin-emergency-banner" style="display: none;">
      <div class="banner-left-wrap">
        <span class="banner-siren">🚨</span>
        <div>
          <div class="banner-title">EMERGENCY EVACUATION ACTIVE (RISK &ge; 70%)</div>
          <div class="banner-subtext" id="adminEmergencySubtext">Automatic barrier lockdown in progress &bull; Acoustic siren active</div>
        </div>
      </div>
      <div class="admin-digital-clock-box">
        <div class="clock-label">LOCKDOWN IN</div>
        <div id="adminTimerClock" class="clock-digits">00:30</div>
      </div>
      <button type="button" id="btnEmergencyMute" class="btn-emergency-mute-action" onclick="toggleBuzzerMute()">
        <span id="emergencyMuteIcon">🔇</span>
        <span id="emergencyMuteText">Mute Buzzer</span>
      </button>
    </div>

    <!-- COMMAND CENTER NAVIGATION TABS -->
    <div class="admin-tab-nav">
      <button type="button" class="tab-nav-btn active" id="tabNavOps" onclick="switchAdminTab('ops')">
        <span style="font-size: 1.1rem;">🎛️</span>
        <span>Operations</span>
      </button>
      <button type="button" class="tab-nav-btn" id="tabNavTrends" onclick="switchAdminTab('trends')">
        <span style="font-size: 1.1rem;">📈</span>
        <span>Trends <span id="tabTrendBadge" class="tab-counter-pill">0</span></span>
      </button>
      <button type="button" class="tab-nav-btn" id="tabNavAudit" onclick="switchAdminTab('audit')">
        <span style="font-size: 1.1rem;">📜</span>
        <span>Audit Log <span id="tabAuditBadge" class="tab-counter-pill">0</span></span>
      </button>
    </div>

    <!-- TAB PANE 1: LIVE OPERATIONS -->
    <div id="tabPaneOps" class="tab-pane active">
      <!-- EXCLUSIVE MANUAL ROAD GATE ACTUATION -->
      <div class="exclusive-ctrl-card">
        <div class="exclusive-head">
          <span>Barrier &amp; Siren Controls (D7 &amp; D8)</span>
          <div class="exclusive-pills">
            <span id="adminSirenPill" style="font-size: 0.72rem; padding: 3px 8px; border-radius: 6px; background: var(--surface-subtle); color: var(--text-dim); border: 1px solid var(--border); font-weight: 800;">SIREN: IDLE</span>
            <span id="adminGatePill" style="font-size: 0.72rem; padding: 3px 8px; border-radius: 6px; background: var(--emerald-bg); color: var(--emerald-text); border: 1px solid var(--emerald-border); font-weight: 800;">BARRIER OPEN</span>
          </div>
        </div>
        <div class="exclusive-btn-row">
          <button class="btn-actuate btn-action-open" onclick="execGateCmd('open')">
            <span>🔓 Open Road Gate</span>
          </button>
          <button class="btn-actuate btn-action-close" onclick="execGateCmd('close')">
            <span>⛔ Emergency Blockade</span>
          </button>
          <button type="button" id="btnOpsBuzzerMute" class="btn-actuate" style="background: var(--surface-subtle); color: var(--text-dim); border: 1.5px solid var(--border-strong);" onclick="toggleBuzzerMute()">
            <span id="opsMuteIcon">🔇</span>
            <span id="opsMuteText">Mute Siren (GPIO15)</span>
          </button>
        </div>
        <div id="adminGateMsg" style="font-size: 0.74rem; color: var(--text-muted); margin-top: 10px; text-align: center; line-height: 1.4;">Rotates 375ms &amp; firmly parks. Silencing siren leaves countdown &amp; gate closure active.</div>
      </div>

      <!-- ADMIN RISK METER COMPONENT: Precise Breakdowns & Contributions -->
      <div class="card">
        <div class="card-head">
          <span class="card-title">Risk Matrix &amp; Sensor Breakdown</span>
          <span id="adminRiskBadge" class="card-badge" style="background: var(--emerald-bg); color: var(--emerald-text); border: 1px solid var(--emerald-border); font-weight: 800;">SAFE (0-44%)</span>
        </div>

        <!-- Master Risk Score Row -->
        <div class="admin-risk-master">
          <div class="admin-risk-main-val">
            <span id="adminRiskScoreBig" class="risk-score-num">0.0%</span>
            <div class="risk-score-meta">
              <span class="meta-label">Fused EMA Risk Score</span>
              <span id="adminRiskStateLbl" class="meta-status" style="color: var(--emerald-text);">Normal Baseline</span>
            </div>
          </div>
          <div class="admin-risk-secondary">
            <div class="sec-item"><span class="sec-lbl">Instantaneous</span><span id="adminInstantRisk" class="sec-val">0.0%</span></div>
            <div class="sec-item"><span class="sec-lbl">Abnormal</span><span id="adminAbnormalCount" class="sec-val">0 / 4</span></div>
            <div class="sec-item"><span class="sec-lbl">Soil Mult</span><span id="adminSoilMult" class="sec-val">1.00x</span></div>
          </div>
        </div>

        <!-- Precision Gauge Segment Bar with Threshold Markers -->
        <div class="admin-gauge-bar-wrap">
          <div class="admin-gauge-bar">
            <div id="adminRiskFill" class="admin-gauge-fill" style="width: 0%;"></div>
            <div class="gauge-marker marker-45"><span class="marker-tag">45% ELEV</span></div>
            <div class="gauge-marker marker-70"><span class="marker-tag">70% CRIT</span></div>
          </div>
        </div>

        <!-- Sensor Contribution Metrics Breakdown -->
        <div class="contribution-section">
          <div class="contrib-title">Weighted Sensor Contributions:</div>
          
          <!-- 1. MPU-6050 Slope Tilt (35% Max Weight) -->
          <div class="contrib-row">
            <div class="contrib-info">
              <span class="contrib-name">📐 MPU-6050 Tilt <small>(35%)</small></span>
              <span id="tiltContribVal" class="contrib-val">0.0% <span class="contrib-pts">(+0.0)</span></span>
            </div>
            <div class="contrib-track"><div id="tiltContribBar" class="contrib-fill" style="width: 0%; background: #2563eb;"></div></div>
          </div>

          <!-- 2. SW-420 Seismic Tremors (30% Max Weight) -->
          <div class="contrib-row">
            <div class="contrib-info">
              <span class="contrib-name">〰️ SW-420 Tremors <small>(30%)</small></span>
              <span id="vibContribVal" class="contrib-val">0.0% <span class="contrib-pts">(+0.0)</span></span>
            </div>
            <div class="contrib-track"><div id="vibContribBar" class="contrib-fill" style="width: 0%; background: #d97706;"></div></div>
          </div>

          <!-- 3. IR Ground Rupture (20% Max Weight) -->
          <div class="contrib-row">
            <div class="contrib-info">
              <span class="contrib-name">⚡ IR Ground Shift <small>(20%)</small></span>
              <span id="dispContribVal" class="contrib-val">CLEAR <span class="contrib-pts">(+0.0)</span></span>
            </div>
            <div class="contrib-track"><div id="dispContribBar" class="contrib-fill" style="width: 0%; background: #dc2626;"></div></div>
          </div>

          <!-- 4. YL-69 Soil Saturation (15% Baseline + Multiplier) -->
          <div class="contrib-row">
            <div class="contrib-info">
              <span class="contrib-name">💧 YL-69 Soil Moisture <small>(15%+)</small></span>
              <span id="soilContribVal" class="contrib-val">0.0% <span class="contrib-pts">(+0.0)</span></span>
            </div>
            <div class="contrib-track"><div id="soilContribBar" class="contrib-fill" style="width: 0%; background: #0891b2;"></div></div>
          </div>
        </div>

        <!-- Active Synergy Logic Flags -->
        <div class="synergy-status-wrap">
          <span id="synLiqBadge" class="syn-badge syn-inactive">Liquefaction: Inactive</span>
          <span id="synSlipBadge" class="syn-badge syn-inactive">Active Slip: Inactive</span>
          <span id="synRuptBadge" class="syn-badge syn-inactive">Triple Failure: Inactive</span>
          <span id="synSuppBadge" class="syn-badge syn-inactive">Filter: Normal</span>
        </div>
      </div>

      <!-- CONFIGURABLE TIMER CONTROLS (ADMIN SETTINGS SECTION) -->
      <div class="card">
        <div class="card-head">
          <span class="card-title">⚙️ Emergency Countdown Settings</span>
          <span class="card-badge" style="background: var(--blue-light); color: var(--blue); border: 1px solid var(--border-accent); font-weight: 800;">EEPROM FLASH</span>
        </div>
        
        <div style="font-size: 0.8rem; color: var(--text-dim); line-height: 1.45;">
          Configure warning seconds before physical barrier closes upon reaching emergency risk (&ge;70%). Stored in NodeMCU EEPROM flash across reboots.
        </div>

        <div class="config-current-row">
          <span style="font-size: 0.82rem; color: var(--text-muted); font-weight: 600;">Active Countdown:</span>
          <span id="cfgCurrentSecs" style="font-size: 1.1rem; font-weight: 900; color: var(--navy-gov);">30 Seconds</span>
        </div>

        <!-- Quick Preset Selection Buttons -->
        <div class="preset-btn-grid">
          <button type="button" class="btn-preset" onclick="setCountdownPreset(15)">15s (Rapid)</button>
          <button type="button" class="btn-preset active" onclick="setCountdownPreset(30)">30s (Default)</button>
          <button type="button" class="btn-preset" onclick="setCountdownPreset(45)">45s (Extended)</button>
          <button type="button" class="btn-preset" onclick="setCountdownPreset(60)">60s (Long Pass)</button>
        </div>

        <!-- Custom Number Input & Save Button -->
        <div class="config-form-row">
          <div class="config-input-wrap">
            <input type="number" id="countdownInput" min="5" max="300" value="30" class="config-num-input" placeholder="Custom Seconds">
            <span class="input-unit">secs</span>
          </div>
          <button type="button" class="btn-save-cfg" onclick="saveCountdownSetting()">💾 Save to EEPROM</button>
        </div>

        <div id="cfgStatusMsg" class="cfg-status-msg" style="display: none;"></div>
      </div>

      <!-- ROW 1: Barrier Graphic & Attitude Horizon (Single Column Mobile Flow) -->
      <div class="section-row">
        <!-- High-Performance Vector Barrier SVG Scene -->
        <div class="card">
          <div class="card-head"><span class="card-title">Barrier Gate Visualizer</span><span class="card-badge">SERVO D7</span></div>
          <div class="gate-scene">
            <svg id="gateSvg" viewBox="0 0 400 140">
              <rect x="0" y="105" width="400" height="35" fill="#475569"/>
              <line x1="0" y1="122" x2="400" y2="122" stroke="#facc15" stroke-width="2.5" stroke-dasharray="18, 14"/>
              <rect x="42" y="55" width="28" height="58" rx="4" fill="#334155" stroke="#1e293b" stroke-width="2"/>
              <circle id="gateStrobe" cx="56" cy="46" r="8" fill="#059669"/>
              <circle cx="56" cy="74" r="7" fill="#cbd5e1" stroke="#334155" stroke-width="1.5"/>
              <g id="gateBoomGroup" style="transform-origin: 56px 74px; transform: rotate(-80deg); transition: transform 0.375s cubic-bezier(0.34, 1.3, 0.64, 1);">
                <rect x="56" y="68" width="270" height="12" rx="4" fill="#ffffff" stroke="#94a3b8" stroke-width="1"/>
                <path d="M 85 68 L 100 80 L 90 80 L 75 68 Z" fill="#dc2626"/>
                <path d="M 125 68 L 140 80 L 130 80 L 115 68 Z" fill="#dc2626"/>
                <path d="M 165 68 L 180 80 L 170 80 L 155 68 Z" fill="#dc2626"/>
                <path d="M 205 68 L 220 80 L 210 80 L 195 68 Z" fill="#dc2626"/>
                <path d="M 245 68 L 260 80 L 250 80 L 235 68 Z" fill="#dc2626"/>
                <path d="M 285 68 L 300 80 L 290 80 L 275 68 Z" fill="#dc2626"/>
                <circle cx="320" cy="74" r="4.5" fill="#dc2626"/>
              </g>
            </svg>
          </div>
        </div>

        <!-- MPU-6050 Attitude Horizon -->
        <div class="card">
          <div class="card-head"><span class="card-title">Attitude Indicator (MPU-6050)</span><span class="card-badge">I2C D1/D2</span></div>
          <div class="horizon-box">
            <div class="horizon-frame">
              <canvas id="horizonCanvas" width="180" height="180"></canvas>
              <div class="horizon-reticle"><div class="reticle-bar"></div><div class="reticle-pip"></div><div class="reticle-bar"></div></div>
            </div>
            <button class="btn-cal" onclick="zeroHorizon()">🎯 <span id="calTxt">Zero Horizon</span></button>
          </div>
          <div class="metric-grid">
            <div class="metric-tile"><span class="m-lbl">Pitch (θ)</span><span id="pitchVal" class="m-val">0.0°</span></div>
            <div class="metric-tile"><span class="m-lbl">Roll (φ)</span><span id="rollVal" class="m-val">0.0°</span></div>
          </div>
        </div>
      </div>

      <!-- ROW 2: Seismic Oscilloscope & Soil Moisture (Single Column Mobile Flow) -->
      <div class="section-row">
        <!-- SW-420 Scope -->
        <div class="card">
          <div class="card-head"><span class="card-title">Seismic Oscilloscope</span><span class="card-badge">SW-420 D5</span></div>
          <div class="scope-box"><canvas id="scopeCanvas" width="360" height="120"></canvas></div>
          <div class="metric-grid">
            <div class="metric-tile"><span class="m-lbl">Tremor Frequency</span><span id="vibFreqTxt" class="m-val">0 / 4s</span></div>
            <div class="metric-tile"><span class="m-lbl">Total Triggers</span><span id="vibHitsTxt" class="m-val">0</span></div>
          </div>
        </div>

        <!-- YL-69 Soil & IR -->
        <div class="card">
          <div class="card-head"><span class="card-title">Soil Saturation &amp; Shift</span><span class="card-badge">A0 &amp; D6</span></div>
          <div class="soil-tank-wrap">
            <div class="liquid-tank">
              <div id="liquidFill" class="liquid-fill"></div>
              <div class="liquid-ticks"><div class="tank-tick"></div><div class="tank-tick"></div><div class="tank-tick"></div><div class="tank-tick"></div></div>
            </div>
            <div style="flex-grow: 1; display: flex; flex-direction: column; gap: 8px;">
              <div>
                <div id="soilPctBig" style="font-size: 2rem; font-weight: 900; color: #0284c7; line-height: 1;">0%</div>
                <div id="soilStatusBadge" style="font-size: 0.72rem; font-weight: 800; color: var(--emerald-text); margin-top: 3px;">SAFE CAPACITY</div>
              </div>
              <div class="metric-grid">
                <div class="metric-tile"><span class="m-lbl">IR Ground</span><span id="irStatusTxt" class="m-val">CLEAR</span></div>
                <div class="metric-tile"><span class="m-lbl">Landslide Risk</span><span id="riskScoreTxt" class="m-val">0%</span></div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div> <!-- END TAB PANE 1: LIVE OPERATIONS -->

    <!-- TAB PANE 2: SENSOR TELEMETRY TRENDS -->
    <div id="tabPaneTrends" class="tab-pane">
      <div class="card">
        <div class="card-head">
          <span class="card-title">📈 Telemetry &amp; Risk Trend Analysis</span>
          <span id="logCountBadge" class="card-badge" style="background: var(--blue-light); color: var(--blue); border: 1px solid var(--border-accent); font-weight: 800;">LITTLEFS: 0 SNAPSHOTS</span>
        </div>

        <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 8px; width: 100%;">
          <button type="button" class="btn-touch btn-compact" onclick="fetchHistoricalLogs()">🔄 Refresh</button>
          <button type="button" class="btn-touch btn-compact" style="color: var(--crimson-text); border-color: var(--crimson-border); background: var(--crimson-bg);" onclick="clearHistoricalLogs()">🗑️ Purge</button>
        </div>

        <div style="font-size: 0.78rem; color: var(--text-dim); line-height: 1.45;">
          Continuous 10-second snapshots recorded to NodeMCU internal flash storage (LittleFS). Review past trends leading up to geotechnical risk spikes.
        </div>

        <!-- Trend Chart Box -->
        <div class="chart-container-box">
          <canvas id="historyChartCanvas" width="480" height="190"></canvas>
        </div>

        <!-- Legend -->
        <div class="chart-legend">
          <span class="legend-item"><span class="legend-dot" style="background: var(--crimson);"></span> Risk %</span>
          <span class="legend-item"><span class="legend-dot" style="background: #0284c7;"></span> Soil %</span>
          <span class="legend-item"><span class="legend-dot" style="background: #d97706;"></span> Tremors</span>
          <span class="legend-item"><span class="legend-dot" style="background: #dc2626;"></span> Rupture</span>
          <span class="legend-item" style="color: var(--amber);">&mdash;&mdash; 45% Limit</span>
          <span class="legend-item" style="color: var(--crimson);">&mdash;&mdash; 70% Limit</span>
        </div>

        <!-- Non-Scrolling Mobile Log Container -->
        <div class="history-table-container">
          <table class="history-table">
            <thead>
              <tr><th>Snapshot Telemetry</th></tr>
            </thead>
            <tbody id="historyLogTableBody">
              <tr>
                <td style="text-align: center; color: var(--text-muted); padding: 24px;">
                  Loading historical telemetry from LittleFS flash storage...
                </td>
              </tr>
            </tbody>
          </table>
        </div>
      </div>
    </div> <!-- END TAB PANE 2: SENSOR TRENDS -->

    <!-- TAB PANE 3: MUNICIPAL EVENT LOG & AUDIT TRAIL -->
    <div id="tabPaneAudit" class="tab-pane">
      <div class="card">
        <div class="card-head">
          <span class="card-title">📜 Municipal System Audit Trail</span>
          <span id="auditCountBadge" class="card-badge" style="background: var(--navy-light); color: var(--navy-gov); border: 1px solid var(--navy-border); font-weight: 800;">TOTAL: 0 EVENTS</span>
        </div>

        <div style="display: grid; grid-template-columns: repeat(3, 1fr); gap: 6px; width: 100%;">
          <button type="button" class="btn-touch btn-compact" onclick="fetchAuditEvents()">🔄 Refresh</button>
          <button type="button" class="btn-touch btn-compact" style="color: var(--blue); border-color: var(--border-accent); background: var(--blue-light);" onclick="exportAuditCsv()">📥 Export</button>
          <button type="button" class="btn-touch btn-compact" style="color: var(--crimson-text); border-color: var(--crimson-border); background: var(--crimson-bg);" onclick="clearAuditEvents()">🗑️ Purge</button>
        </div>

        <div style="font-size: 0.78rem; color: var(--text-dim); line-height: 1.45;">
          Certified chronological record of critical system transitions, threshold crossings, evacuation warnings, and operator interventions.
        </div>

        <!-- Filter & Control Toolbar -->
        <div class="audit-toolbar">
          <div class="audit-search-wrap">
            <span style="font-size: 0.82rem;">🔍</span>
            <input type="text" id="auditSearchInput" class="audit-search-field" placeholder="Search event records..." oninput="filterAuditEvents()">
          </div>
          <div class="audit-filter-chips">
            <button type="button" class="filter-chip active" data-sev="ALL" onclick="setAuditFilter('ALL')">All</button>
            <button type="button" class="filter-chip chip-crit" data-sev="CRITICAL" onclick="setAuditFilter('CRITICAL')">🚨 Critical</button>
            <button type="button" class="filter-chip chip-warn" data-sev="WARNING" onclick="setAuditFilter('WARNING')">⚠️ Warning</button>
            <button type="button" class="filter-chip chip-info" data-sev="INFO" onclick="setAuditFilter('INFO')">ℹ️ Info</button>
            <button type="button" class="filter-chip chip-succ" data-sev="SUCCESS" onclick="setAuditFilter('SUCCESS')">✅ Success</button>
          </div>
          <button type="button" id="auditSortBtn" class="btn-touch btn-compact" onclick="toggleAuditSort()">⬇️ Chronological (Oldest First)</button>
        </div>

        <!-- Non-Scrolling Mobile Event Log -->
        <div class="history-table-container">
          <table class="history-table">
            <thead>
              <tr><th>Audit Record</th></tr>
            </thead>
            <tbody id="auditTableBody">
              <tr>
                <td style="text-align: center; color: var(--text-muted); padding: 24px;">
                  Loading system audit records from NodeMCU memory...
                </td>
              </tr>
            </tbody>
          </table>
        </div>
      </div>
    </div> <!-- END TAB PANE 3: AUDIT TRAIL -->
  </div> <!-- END ADMIN CONSOLE -->

  <script>
    let activeKey = sessionStorage.getItem('admin_passkey') || '';
    let currentAdminTab = 'ops';

    function switchAdminTab(tab) {
      currentAdminTab = tab;
      document.querySelectorAll('.tab-nav-btn').forEach(b => b.classList.remove('active'));
      document.querySelectorAll('.tab-pane').forEach(p => p.classList.remove('active'));

      if (tab === 'ops') {
        const btn = document.getElementById('tabNavOps');
        const pane = document.getElementById('tabPaneOps');
        if (btn) btn.classList.add('active');
        if (pane) pane.classList.add('active');
      } else if (tab === 'trends') {
        const btn = document.getElementById('tabNavTrends');
        const pane = document.getElementById('tabPaneTrends');
        if (btn) btn.classList.add('active');
        if (pane) pane.classList.add('active');
        initHiDPI();
        fetchHistoricalLogs();
      } else if (tab === 'audit') {
        const btn = document.getElementById('tabNavAudit');
        const pane = document.getElementById('tabPaneAudit');
        if (btn) btn.classList.add('active');
        if (pane) pane.classList.add('active');
        fetchAuditEvents();
      }
    }

    function checkAuthSession() {
      if (activeKey) {
        document.getElementById('loginGate').style.display = 'none';
        document.getElementById('adminConsole').style.display = 'flex';
        initHiDPI();
        startTelemetryLoop();
        fetchHistoricalLogs();
        fetchAuditEvents();
      }
    }
    checkAuthSession();

    async function verifyAdminPasskey() {
      const pass = document.getElementById('passkeyInput').value;
      try {
        const res = await fetch('/api/admin/verify?key=' + encodeURIComponent(pass), { method: 'POST' });
        if (res.ok) {
          activeKey = pass;
          sessionStorage.setItem('admin_passkey', pass);
          document.getElementById('loginGate').style.display = 'none';
          document.getElementById('adminConsole').style.display = 'flex';
          initHiDPI();
          startTelemetryLoop();
          fetchHistoricalLogs();
          fetchAuditEvents();
        } else {
          document.getElementById('loginErr').style.display = 'block';
        }
      } catch (e) {
        document.getElementById('loginErr').style.display = 'block';
      }
    }

    function logoutAdmin() {
      sessionStorage.removeItem('admin_passkey');
      activeKey = '';
      window.location.reload();
    }

    async function execGateCmd(action) {
      if (!activeKey) return;
      const url = action === 'open' ? '/api/gate/open?key=' : '/api/gate/close?key=';
      const msg = document.getElementById('adminGateMsg');
      try {
        msg.textContent = (action === 'open' ? 'Commanding gate OPEN (375ms clockwise)...' : 'Commanding gate CLOSE (375ms anticlockwise)...');
        const res = await fetch(url + encodeURIComponent(activeKey), { method: 'POST' });
        if (!res.ok) {
          msg.textContent = 'Action denied: Invalid Admin Key!';
        }
      } catch (e) {
        msg.textContent = 'Command transmission error!';
      }
    }

    async function toggleBuzzerMute() {
      if (!activeKey) return;
      try {
        const res = await fetch('/api/buzzer/mute?key=' + encodeURIComponent(activeKey), { method: 'POST' });
        const d = await res.json();
        if (d.success) {
          const isM = d.muted;
          const eIcon = document.getElementById('emergencyMuteIcon');
          const eText = document.getElementById('emergencyMuteText');
          if (eIcon) eIcon.textContent = isM ? '🔊' : '🔇';
          if (eText) eText.textContent = isM ? 'Unmute Buzzer' : 'Mute Buzzer';
          const oIcon = document.getElementById('opsMuteIcon');
          const oText = document.getElementById('opsMuteText');
          if (oIcon) oIcon.textContent = isM ? '🔊' : '🔇';
          if (oText) oText.textContent = isM ? 'Unmute Siren' : 'Mute Siren (GPIO15)';
        } else {
          alert('Failed to toggle buzzer mute: ' + (d.error || 'Unknown error'));
        }
      } catch (e) {
        alert('Communication error with buzzer controller!');
      }
    }

    // Configurable Timer Controls (Admin Settings)
    function setCountdownPreset(secs) {
      document.getElementById('countdownInput').value = secs;
      document.querySelectorAll('.btn-preset').forEach(b => {
        b.classList.toggle('active', parseInt(b.textContent, 10) === secs);
      });
      saveCountdownSetting(secs);
    }

    async function saveCountdownSetting(customSecs) {
      if (!activeKey) return;
      const val = customSecs !== undefined ? customSecs : parseInt(document.getElementById('countdownInput').value, 10);
      const statusEl = document.getElementById('cfgStatusMsg');

      if (isNaN(val) || val < 5 || val > 300) {
        statusEl.className = 'cfg-status-msg error';
        statusEl.textContent = 'Duration must be between 5 and 300 seconds.';
        statusEl.style.display = 'block';
        return;
      }

      try {
        statusEl.className = 'cfg-status-msg pending';
        statusEl.textContent = 'Saving setting to NodeMCU EEPROM...';
        statusEl.style.display = 'block';

        const res = await fetch('/api/admin/config?key=' + encodeURIComponent(activeKey) + '&countdown=' + val, { method: 'POST' });
        const data = await res.json();

        if (res.ok && data.success) {
          statusEl.className = 'cfg-status-msg success';
          statusEl.textContent = '✅ Successfully updated countdown to ' + data.countdown + 's (Saved in EEPROM flash)!';
          document.getElementById('cfgCurrentSecs').textContent = data.countdown + ' Seconds';
          document.querySelectorAll('.btn-preset').forEach(b => {
            b.classList.toggle('active', parseInt(b.textContent, 10) === data.countdown);
          });
          setTimeout(() => { statusEl.style.display = 'none'; }, 4500);
        } else {
          statusEl.className = 'cfg-status-msg error';
          statusEl.textContent = 'Error: ' + (data.error || 'Failed to save configuration');
        }
      } catch (e) {
        statusEl.className = 'cfg-status-msg error';
        statusEl.textContent = 'Network error saving EEPROM configuration!';
      }
    }

    // Hi-DPI Scaling for Crisp Mobile OLED/Retina Canvas Display
    const hCanvas = document.getElementById('horizonCanvas');
    const hCtx = hCanvas.getContext('2d');
    const sCanvas = document.getElementById('scopeCanvas');
    const sCtx = sCanvas.getContext('2d');
    let dpr = window.devicePixelRatio || 1;

    function initHiDPI() {
      dpr = window.devicePixelRatio || 1;
      const hRect = hCanvas.getBoundingClientRect();
      if (hRect.width > 0) {
        hCanvas.width = hRect.width * dpr;
        hCanvas.height = hRect.height * dpr;
      }
      const sRect = sCanvas.getBoundingClientRect();
      if (sRect.width > 0) {
        sCanvas.width = sRect.width * dpr;
        sCanvas.height = sRect.height * dpr;
      }
      const cCanvas = document.getElementById('historyChartCanvas');
      if (cCanvas) {
        const cRect = cCanvas.getBoundingClientRect();
        if (cRect.width > 0) {
          cCanvas.width = cRect.width * dpr;
          cCanvas.height = cRect.height * dpr;
          if (typeof renderHistoryChart === 'function' && historicalLogsData && historicalLogsData.length > 0) {
            renderHistoryChart(historicalLogsData);
          }
        }
      }
    }
    window.addEventListener('resize', initHiDPI);

    let rawP = 0, rawR = 0, calP = parseFloat(localStorage.getItem('cal_pitch')||'0'), calR = parseFloat(localStorage.getItem('cal_roll')||'0');
    let curP = 0, curR = 0;

    function zeroHorizon() {
      calP = rawP; calR = rawR;
      localStorage.setItem('cal_pitch', calP.toString());
      localStorage.setItem('cal_roll', calR.toString());
      document.getElementById('calTxt').textContent = 'Zeroed!';
      setTimeout(() => { document.getElementById('calTxt').textContent = 'Zero Horizon'; }, 1800);
    }

    // High-performance 60fps Artificial Horizon Canvas Loop
    function drawHorizon() {
      curP += ((rawP - calP) - curP) * 0.18;
      curR += ((rawR - calR) - curR) * 0.18;

      const w = hCanvas.width / dpr;
      const h = hCanvas.height / dpr;
      const cx = w / 2;
      const cy = h / 2;

      hCtx.save();
      hCtx.scale(dpr, dpr);
      hCtx.clearRect(0, 0, w, h);
      hCtx.beginPath();
      hCtx.arc(cx, cy, cx - 2, 0, Math.PI * 2);
      hCtx.clip();

      hCtx.translate(cx, cy);
      hCtx.rotate((-curR * Math.PI) / 180);
      hCtx.translate(0, curP * 1.8);

      // Sky Gradient
      const sky = hCtx.createLinearGradient(0, -h * 2, 0, 0);
      sky.addColorStop(0, '#0369a1');
      sky.addColorStop(1, '#38bdf8');
      hCtx.fillStyle = sky;
      hCtx.fillRect(-w * 2, -h * 2, w * 4, h * 2);

      // Ground Gradient
      const gnd = hCtx.createLinearGradient(0, 0, 0, h * 2);
      gnd.addColorStop(0, '#78350f');
      gnd.addColorStop(1, '#451a03');
      hCtx.fillStyle = gnd;
      hCtx.fillRect(-w * 2, 0, w * 4, h * 2);

      // White Horizon Dividing Line
      hCtx.strokeStyle = '#ffffff';
      hCtx.lineWidth = 2.5;
      hCtx.beginPath();
      hCtx.moveTo(-w * 2, 0);
      hCtx.lineTo(w * 2, 0);
      hCtx.stroke();

      // Pitch Ladder Ticks
      hCtx.strokeStyle = 'rgba(255, 255, 255, 0.7)';
      hCtx.lineWidth = 1.5;
      hCtx.fillStyle = 'rgba(255, 255, 255, 0.8)';
      hCtx.font = '8px sans-serif';
      hCtx.textAlign = 'center';
      for (let deg = -40; deg <= 40; deg += 10) {
        if (deg === 0) continue;
        const y = -deg * 1.8;
        const tickW = (Math.abs(deg) % 20 === 0) ? 28 : 16;
        hCtx.beginPath();
        hCtx.moveTo(-tickW, y);
        hCtx.lineTo(tickW, y);
        hCtx.stroke();
      }

      hCtx.restore();
      requestAnimationFrame(drawHorizon);
    }
    requestAnimationFrame(drawHorizon);

    // High-performance 60fps Seismic Oscilloscope Canvas Loop
    const waveform = new Float32Array(140).fill(0);
    let shockEnergy = 0, lastHits = 0;

    function drawScope() {
      shockEnergy *= 0.93;
      const noise = (Math.random() - 0.5) * 3;
      const spike = shockEnergy * (Math.sin(Date.now() * 0.05) * 32);
      for (let i = 0; i < 139; i++) waveform[i] = waveform[i + 1];
      waveform[139] = spike + noise;

      const sw = sCanvas.width / dpr;
      const sh = sCanvas.height / dpr;

      sCtx.save();
      sCtx.scale(dpr, dpr);
      sCtx.fillStyle = '#0f172a';
      sCtx.fillRect(0, 0, sw, sh);

      // Sub-graticule Grid
      sCtx.strokeStyle = 'rgba(148, 163, 184, 0.15)';
      sCtx.lineWidth = 1;
      sCtx.beginPath();
      for (let x = 0; x < sw; x += 30) { sCtx.moveTo(x, 0); sCtx.lineTo(x, sh); }
      for (let y = 0; y < sh; y += 20) { sCtx.moveTo(0, y); sCtx.lineTo(sw, y); }
      sCtx.stroke();

      // Trace Line
      sCtx.strokeStyle = shockEnergy > 0.3 ? '#ef4444' : '#38bdf8';
      sCtx.lineWidth = 2.2;
      sCtx.shadowColor = shockEnergy > 0.3 ? 'rgba(239, 68, 68, 0.8)' : 'rgba(56, 189, 248, 0.8)';
      sCtx.shadowBlur = 4;
      sCtx.beginPath();
      const step = sw / 139;
      for (let i = 0; i < 140; i++) {
        const x = i * step;
        const y = (sh / 2) - waveform[i];
        if (i === 0) sCtx.moveTo(x, y); else sCtx.lineTo(x, y);
      }
      sCtx.stroke();
      sCtx.restore();

      requestAnimationFrame(drawScope);
    }
    requestAnimationFrame(drawScope);

    // Sequential Asynchronous Telemetry Poller for ESP8266
    let telemetryTimer = null;
    let isFetching = false;
    let targetAdminRisk = 0;
    let curAdminRisk = 0;

    function renderAdminRiskSmooth() {
      curAdminRisk += (targetAdminRisk - curAdminRisk) * 0.15;
      const rFill = document.getElementById('adminRiskFill');
      if (rFill) rFill.style.width = curAdminRisk + '%';
      const rBig = document.getElementById('adminRiskScoreBig');
      if (rBig) rBig.textContent = curAdminRisk.toFixed(1) + '%';
      requestAnimationFrame(renderAdminRiskSmooth);
    }
    requestAnimationFrame(renderAdminRiskSmooth);

    function startTelemetryLoop() {
      if (telemetryTimer) return;
      initHiDPI();

      telemetryTimer = setInterval(async () => {
        if (isFetching) return;
        isFetching = true;

        try {
          const res = await fetch('/api/telemetry');
          if (!res.ok) throw new Error();
          const d = await res.json();

          rawP = d.mpu.pitch;
          rawR = d.mpu.roll;
          document.getElementById('pitchVal').textContent = (rawP - calP).toFixed(1) + '°';
          document.getElementById('rollVal').textContent = (rawR - calR).toFixed(1) + '°';

          if (d.vibration.window_hits > lastHits || d.risk.percentage > 60) shockEnergy = 1.0;
          lastHits = d.vibration.window_hits;
          document.getElementById('vibFreqTxt').textContent = d.vibration.window_hits + ' / 4s';
          document.getElementById('vibHitsTxt').textContent = d.vibration.total_hits;

          // Road Barrier Gate Vector Visualizer
          const boom = document.getElementById('gateBoomGroup');
          const strobe = document.getElementById('gateStrobe');
          const gPill = document.getElementById('adminGatePill');

          if (d.gate.status === "CLOSED") {
            boom.style.transform = 'rotate(0deg)';
            strobe.setAttribute('fill', '#dc2626');
            gPill.style.background = 'var(--crimson-bg)';
            gPill.style.color = 'var(--crimson-text)';
            gPill.style.borderColor = 'var(--crimson-border)';
            gPill.textContent = '⛔ BARRIER BLOCKED';
          } else if (d.gate.status === "CLOSING" || d.gate.status === "OPENING") {
            boom.style.transform = d.gate.status === "CLOSING" ? 'rotate(-35deg)' : 'rotate(-55deg)';
            strobe.setAttribute('fill', '#d97706');
            gPill.style.background = 'var(--amber-bg)';
            gPill.style.color = 'var(--amber-text)';
            gPill.style.borderColor = 'var(--amber-border)';
            gPill.textContent = 'BARRIER MOVING...';
          } else {
            boom.style.transform = 'rotate(-80deg)';
            strobe.setAttribute('fill', '#059669');
            gPill.style.background = 'var(--emerald-bg)';
            gPill.style.color = 'var(--emerald-text)';
            gPill.style.borderColor = 'var(--emerald-border)';
            gPill.textContent = 'BARRIER OPEN';
          }

          document.getElementById('soilPctBig').textContent = d.soil.percentage + '%';
          document.getElementById('liquidFill').style.height = Math.min(100, Math.max(0, d.soil.percentage)) + '%';
          document.getElementById('riskScoreTxt').textContent = d.risk.percentage.toFixed(0) + '%';
          document.getElementById('irStatusTxt').textContent = d.ir.obstacle ? 'DISPLACEMENT!' : 'CLEAR';
          document.getElementById('irStatusTxt').style.color = d.ir.obstacle ? 'var(--crimson-text)' : 'var(--emerald-text)';

          // Standard Digital Emergency Timer Banner
          const banner = document.getElementById('adminEmergencyBanner');
          const timerDigits = document.getElementById('adminTimerClock');
          if (d.emergency.active || d.risk.percentage >= 70) {
            banner.style.display = 'flex';
            const rem = Math.max(0, d.emergency.countdown !== undefined ? d.emergency.countdown : 0);
            const mm = String(Math.floor(rem / 60)).padStart(2, '0');
            const ss = String(rem % 60).padStart(2, '0');
            timerDigits.textContent = mm + ':' + ss;
          } else {
            banner.style.display = 'none';
          }

          // Real-time Buzzer Mute Status Synchronization
          const isMuted = d.emergency.buzzer_muted === true;
          const eIcon = document.getElementById('emergencyMuteIcon');
          const eText = document.getElementById('emergencyMuteText');
          const eBtn = document.getElementById('btnEmergencyMute');
          const eSub = document.getElementById('adminEmergencySubtext');
          const sPill = document.getElementById('adminSirenPill');
          const oIcon = document.getElementById('opsMuteIcon');
          const oText = document.getElementById('opsMuteText');
          const oBtn = document.getElementById('btnOpsBuzzerMute');

          if (isMuted) {
            if (eIcon) eIcon.textContent = '🔊';
            if (eText) eText.textContent = 'Unmute Buzzer';
            if (eBtn) {
              eBtn.style.background = '#fffbeb';
              eBtn.style.borderColor = 'var(--amber)';
              eBtn.style.color = 'var(--amber-text)';
            }
            if (eSub) eSub.textContent = 'Automatic barrier lockdown in progress • 🔇 Siren Silenced (Countdown running)';
            if (sPill) {
              sPill.textContent = 'SIREN: MUTED';
              sPill.style.background = 'var(--amber-bg)';
              sPill.style.color = 'var(--amber-text)';
              sPill.style.borderColor = 'var(--amber-border)';
            }
            if (oIcon) oIcon.textContent = '🔊';
            if (oText) oText.textContent = 'Unmute Siren';
            if (oBtn) {
              oBtn.style.background = 'var(--amber-bg)';
              oBtn.style.borderColor = 'var(--amber-border)';
              oBtn.style.color = 'var(--amber-text)';
            }
          } else {
            if (eIcon) eIcon.textContent = '🔇';
            if (eText) eText.textContent = 'Mute Buzzer';
            if (eBtn) {
              eBtn.style.background = '#ffffff';
              eBtn.style.borderColor = 'var(--crimson)';
              eBtn.style.color = 'var(--crimson-text)';
            }
            if (eSub) eSub.textContent = 'Automatic barrier lockdown in progress • Acoustic siren active';
            if (sPill) {
              if (d.emergency.buzzer) {
                sPill.textContent = 'SIREN: SOUNDING';
                sPill.style.background = 'var(--crimson-bg)';
                sPill.style.color = 'var(--crimson-text)';
                sPill.style.borderColor = 'var(--crimson-border)';
              } else {
                sPill.textContent = 'SIREN: IDLE';
                sPill.style.background = 'var(--surface-subtle)';
                sPill.style.color = 'var(--text-dim)';
                sPill.style.borderColor = 'var(--border)';
              }
            }
            if (oIcon) oIcon.textContent = '🔇';
            if (oText) oText.textContent = 'Mute Siren (GPIO15)';
            if (oBtn) {
              oBtn.style.background = 'var(--surface-subtle)';
              oBtn.style.borderColor = 'var(--border)';
              oBtn.style.color = 'var(--text-dim)';
            }
          }

          if (d.emergency.countdown_duration) {
            document.getElementById('cfgCurrentSecs').textContent = d.emergency.countdown_duration + ' Seconds';
          }

          // ADMIN RISK METER COMPONENT: Precise Breakdowns & Contributions
          targetAdminRisk = Math.min(100, Math.max(0, d.risk.percentage));
          const rBadge = document.getElementById('adminRiskBadge');
          const rState = document.getElementById('adminRiskStateLbl');

          if (targetAdminRisk >= 70) {
            rBadge.style.background = 'var(--crimson-bg)';
            rBadge.style.color = 'var(--crimson-text)';
            rBadge.style.borderColor = 'var(--crimson-border)';
            rBadge.textContent = 'CRITICAL (70-100%)';
            rState.textContent = 'Critical Emergency';
            rState.style.color = 'var(--crimson-text)';
          } else if (targetAdminRisk >= 45) {
            rBadge.style.background = 'var(--amber-bg)';
            rBadge.style.color = 'var(--amber-text)';
            rBadge.style.borderColor = 'var(--amber-border)';
            rBadge.textContent = 'ELEVATED (45-69%)';
            rState.textContent = 'Moderate Activity';
            rState.style.color = 'var(--amber-text)';
          } else {
            rBadge.style.background = 'var(--emerald-bg)';
            rBadge.style.color = 'var(--emerald-text)';
            rBadge.style.borderColor = 'var(--emerald-border)';
            rBadge.textContent = 'SAFE (0-44%)';
            rState.textContent = 'Normal Baseline';
            rState.style.color = 'var(--emerald-text)';
          }

          // Secondary Metrics
          document.getElementById('adminInstantRisk').textContent = (d.risk.instant !== undefined ? d.risk.instant.toFixed(1) : targetAdminRisk.toFixed(1)) + '%';
          document.getElementById('adminAbnormalCount').textContent = (d.risk.concurrent_abnormal !== undefined ? d.risk.concurrent_abnormal : 0) + ' / 4';
          document.getElementById('adminSoilMult').textContent = (d.risk.soil_mult !== undefined ? d.risk.soil_mult.toFixed(2) : '1.00') + 'x';

          // Sensor Contributions
          const tiltScore = d.risk.tilt_score !== undefined ? d.risk.tilt_score : 0;
          const tiltPts = (tiltScore * 0.35);
          document.getElementById('tiltContribVal').innerHTML = tiltScore.toFixed(1) + '% <span class="contrib-pts">(+' + tiltPts.toFixed(1) + ' pts)</span>';
          document.getElementById('tiltContribBar').style.width = Math.min(100, tiltScore) + '%';

          const vibScore = d.risk.vib_score !== undefined ? d.risk.vib_score : 0;
          const vibPts = (vibScore * 0.30);
          document.getElementById('vibContribVal').innerHTML = vibScore.toFixed(1) + '% <span class="contrib-pts">(+' + vibPts.toFixed(1) + ' pts)</span>';
          document.getElementById('vibContribBar').style.width = Math.min(100, vibScore) + '%';

          const dispScore = d.risk.disp_score !== undefined ? d.risk.disp_score : 0;
          const dispPts = (dispScore * 0.20);
          const dispTxt = dispScore > 0 ? 'RUPTURE' : 'CLEAR';
          document.getElementById('dispContribVal').innerHTML = dispTxt + ' <span class="contrib-pts">(+' + dispPts.toFixed(1) + ' pts)</span>';
          document.getElementById('dispContribBar').style.width = Math.min(100, dispScore) + '%';

          const soilScore = d.risk.soil_score !== undefined ? d.risk.soil_score : 0;
          const soilPts = (soilScore * 0.15);
          document.getElementById('soilContribVal').innerHTML = soilScore.toFixed(1) + '% <span class="contrib-pts">(+' + soilPts.toFixed(1) + ' pts)</span>';
          document.getElementById('soilContribBar').style.width = Math.min(100, soilScore) + '%';

          // Synergy Badges
          const bLiq = document.getElementById('synLiqBadge');
          const bSlip = document.getElementById('synSlipBadge');
          const bRupt = document.getElementById('synRuptBadge');
          const bSupp = document.getElementById('synSuppBadge');

          bLiq.className = d.risk.syn_liquefaction ? 'syn-badge syn-active' : 'syn-badge syn-inactive';
          bLiq.textContent = d.risk.syn_liquefaction ? 'Liquefaction: ACTIVE (Floor 72%)' : 'Liquefaction: Inactive';

          bSlip.className = d.risk.syn_slip ? 'syn-badge syn-active' : 'syn-badge syn-inactive';
          bSlip.textContent = d.risk.syn_slip ? 'Active Slip: ACTIVE (Floor 78%)' : 'Active Slip: Inactive';

          bRupt.className = d.risk.syn_rupture ? 'syn-badge syn-active' : 'syn-badge syn-inactive';
          bRupt.textContent = d.risk.syn_rupture ? 'Triple Hazard: ACTIVE (Floor 90%)' : 'Triple Hazard: Inactive';

          bSupp.className = d.risk.suppression ? 'syn-badge syn-active' : 'syn-badge syn-inactive';
          bSupp.textContent = d.risk.suppression ? 'Single-Sensor Filter: ENGAGED (Cap 38%)' : 'Single-Sensor Filter: Normal';
        } catch (e) {
        } finally {
          isFetching = false;
        }
      }, 250);
    }

    // Historical Telemetry Engine (LittleFS)
    let historicalLogsData = [];
    const chartCanvas = document.getElementById('historyChartCanvas');
    const chartCtx = chartCanvas ? chartCanvas.getContext('2d') : null;

    async function fetchHistoricalLogs() {
      if (!activeKey) return;
      try {
        const res = await fetch('/api/admin/logs?key=' + encodeURIComponent(activeKey));
        if (!res.ok) throw new Error();
        const d = await res.json();
        if (d.success && Array.isArray(d.logs)) {
          historicalLogsData = d.logs;
          document.getElementById('logCountBadge').textContent = 'LITTLEFS: ' + d.logs.length + ' SNAPSHOTS';
          renderHistoryChart(d.logs);
          renderHistoryTable(d.logs);
        }
      } catch (e) {
      }
    }

    async function clearHistoricalLogs() {
      if (!activeKey) return;
      if (!confirm('Purge all historical telemetry snapshots from LittleFS flash memory?')) return;
      try {
        const res = await fetch('/api/admin/logs/clear?key=' + encodeURIComponent(activeKey), { method: 'POST' });
        const d = await res.json();
        if (d.success) {
          historicalLogsData = [];
          document.getElementById('logCountBadge').textContent = 'LITTLEFS: 0 SNAPSHOTS';
          renderHistoryChart([]);
          renderHistoryTable([]);
        }
      } catch (e) {
        alert('Failed to clear logs: ' + e);
      }
    }

    function formatRelativeTime(seconds) {
      if (seconds < 60) return seconds + 's';
      const m = Math.floor(seconds / 60);
      const s = seconds % 60;
      if (m < 60) return m + 'm ' + (s < 10 ? '0' : '') + s + 's';
      const h = Math.floor(m / 60);
      const remM = m % 60;
      return h + 'h ' + (remM < 10 ? '0' : '') + remM + 'm';
    }

    function renderHistoryTable(logs) {
      const tbody = document.getElementById('historyLogTableBody');
      if (!tbody) return;
      if (!logs || logs.length === 0) {
        tbody.innerHTML = '<tr><td style="text-align: center; color: var(--text-muted); padding: 24px 10px;">No historical snapshots recorded yet in LittleFS.</td></tr>';
        return;
      }

      let html = '';
      for (let i = logs.length - 1; i >= 0; i--) {
        const item = logs[i];
        const risk = Number(item.risk) || 0;
        let riskClass = 'tag-safe';
        let riskLabel = 'SAFE';
        if (risk >= 70) {
          riskClass = 'tag-crimson';
          riskLabel = 'CRITICAL';
        } else if (risk >= 45) {
          riskClass = 'tag-amber';
          riskLabel = 'ELEVATED';
        }

        const dispText = (item.disp === 1 || item.disp === true) ? '<span style="color:var(--crimson-text); font-weight:800;">RUPTURE</span>' : '<span style="color:var(--emerald-text); font-weight:700;">CLEAR</span>';
        const gateColor = (item.gate === 'CLOSED') ? 'color:var(--crimson-text); font-weight:800;' : 'color:var(--emerald-text); font-weight:700;';

        html += '<tr><td>' +
          '<div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:6px; flex-wrap:wrap; gap:4px;">' +
            '<span style="font-size:0.8rem; font-weight:800; color:var(--navy-gov);">⏱️ T+' + formatRelativeTime(item.time) + '</span>' +
            '<span class="risk-state-tag ' + riskClass + '" style="font-size:0.65rem; padding:2px 8px;">' + riskLabel + ' (' + risk.toFixed(1) + '%)</span>' +
            '<span style="font-size:0.75rem; ' + gateColor + '">Gate: ' + item.gate + '</span>' +
          '</div>' +
          '<div style="display:grid; grid-template-columns:1fr 1fr 1fr; gap:6px; font-size:0.74rem; background:var(--surface-subtle); padding:6px 8px; border-radius:6px; border:1px solid var(--border); box-sizing:border-box;">' +
            '<div style="text-align:center;"><span style="color:var(--text-dim); font-size:0.62rem; display:block; font-weight:700;">SOIL</span><strong style="color:#0284c7;">' + item.soil + '%</strong></div>' +
            '<div style="text-align:center;"><span style="color:var(--text-dim); font-size:0.62rem; display:block; font-weight:700;">SEISMIC</span>' + (item.vib > 0 ? ('<strong style="color:var(--amber-text);">' + item.vib + ' hits</strong>') : '<span style="color:var(--text-muted);">0</span>') + '</div>' +
            '<div style="text-align:center;"><span style="color:var(--text-dim); font-size:0.62rem; display:block; font-weight:700;">GROUND</span>' + dispText + '</div>' +
          '</div>' +
          '</td></tr>';
      }
      tbody.innerHTML = html;
    }

    function renderHistoryChart(logs) {
      if (!chartCanvas || !chartCtx) return;
      const rect = chartCanvas.getBoundingClientRect();
      if (rect.width > 0) {
        chartCanvas.width = rect.width * dpr;
        chartCanvas.height = rect.height * dpr;
      }
      const w = chartCanvas.width / dpr;
      const h = chartCanvas.height / dpr;

      chartCtx.save();
      chartCtx.scale(dpr, dpr);
      chartCtx.clearRect(0, 0, w, h);

      // Background
      chartCtx.fillStyle = '#ffffff';
      chartCtx.fillRect(0, 0, w, h);

      const padL = 36;
      const padR = 14;
      const padT = 16;
      const padB = 26;
      const plotW = w - padL - padR;
      const plotH = h - padT - padB;

      // Draw Grid Lines & Y-axis labels (0%, 25%, 50%, 75%, 100%)
      chartCtx.lineWidth = 1;
      chartCtx.font = '9px sans-serif';
      chartCtx.textAlign = 'right';
      chartCtx.textBaseline = 'middle';

      const yTicks = [0, 25, 50, 75, 100];
      for (let i = 0; i < yTicks.length; i++) {
        const val = yTicks[i];
        const y = padT + plotH - (val / 100) * plotH;
        chartCtx.strokeStyle = '#f1f5f9';
        chartCtx.beginPath();
        chartCtx.moveTo(padL, y);
        chartCtx.lineTo(w - padR, y);
        chartCtx.stroke();

        chartCtx.fillStyle = '#94a3b8';
        chartCtx.fillText(val + '%', padL - 6, y);
      }

      // Draw Threshold Reference Lines: 45% (Amber) and 70% (Crimson)
      const y45 = padT + plotH - (45 / 100) * plotH;
      chartCtx.strokeStyle = 'rgba(217, 119, 6, 0.45)';
      chartCtx.setLineDash([4, 4]);
      chartCtx.beginPath();
      chartCtx.moveTo(padL, y45);
      chartCtx.lineTo(w - padR, y45);
      chartCtx.stroke();

      const y70 = padT + plotH - (70 / 100) * plotH;
      chartCtx.strokeStyle = 'rgba(220, 38, 38, 0.55)';
      chartCtx.setLineDash([4, 4]);
      chartCtx.beginPath();
      chartCtx.moveTo(padL, y70);
      chartCtx.lineTo(w - padR, y70);
      chartCtx.stroke();
      chartCtx.setLineDash([]); // Reset dash

      // If no data, show watermark
      if (!logs || logs.length < 2) {
        chartCtx.fillStyle = '#94a3b8';
        chartCtx.textAlign = 'center';
        chartCtx.font = '11px sans-serif';
        chartCtx.fillText('Collecting snapshot telemetry in LittleFS...', w / 2, h / 2);
        chartCtx.restore();
        return;
      }

      const count = logs.length;
      const xStep = plotW / (count - 1);

      // 1. Draw Soil Moisture Line (Blue dashed)
      chartCtx.strokeStyle = '#0284c7';
      chartCtx.lineWidth = 1.6;
      chartCtx.setLineDash([3, 3]);
      chartCtx.beginPath();
      for (let i = 0; i < count; i++) {
        const x = padL + i * xStep;
        const val = Math.min(100, Math.max(0, logs[i].soil));
        const y = padT + plotH - (val / 100) * plotH;
        if (i === 0) chartCtx.moveTo(x, y); else chartCtx.lineTo(x, y);
      }
      chartCtx.stroke();
      chartCtx.setLineDash([]);

      // 2. Draw Geotechnical Risk Filled Area & Line
      const grad = chartCtx.createLinearGradient(0, padT, 0, padT + plotH);
      grad.addColorStop(0, 'rgba(220, 38, 38, 0.25)');
      grad.addColorStop(0.55, 'rgba(217, 119, 6, 0.15)');
      grad.addColorStop(1, 'rgba(5, 150, 105, 0.05)');

      chartCtx.fillStyle = grad;
      chartCtx.beginPath();
      chartCtx.moveTo(padL, padT + plotH);
      for (let i = 0; i < count; i++) {
        const x = padL + i * xStep;
        const val = Math.min(100, Math.max(0, logs[i].risk));
        const y = padT + plotH - (val / 100) * plotH;
        chartCtx.lineTo(x, y);
      }
      chartCtx.lineTo(padL + (count - 1) * xStep, padT + plotH);
      chartCtx.closePath();
      chartCtx.fill();

      // Draw Risk Line
      chartCtx.strokeStyle = '#dc2626';
      chartCtx.lineWidth = 2.4;
      chartCtx.beginPath();
      for (let i = 0; i < count; i++) {
        const x = padL + i * xStep;
        const val = Math.min(100, Math.max(0, logs[i].risk));
        const y = padT + plotH - (val / 100) * plotH;
        if (i === 0) chartCtx.moveTo(x, y); else chartCtx.lineTo(x, y);
      }
      chartCtx.stroke();

      // 3. Draw Points & Event Markers
      for (let i = 0; i < count; i++) {
        const item = logs[i];
        const x = padL + i * xStep;
        const rVal = Math.min(100, Math.max(0, item.risk));
        const y = padT + plotH - (rVal / 100) * plotH;

        if (item.vib > 0) {
          chartCtx.fillStyle = '#d97706';
          chartCtx.beginPath();
          chartCtx.arc(x, padT + plotH - 3, 3, 0, Math.PI * 2);
          chartCtx.fill();
        }

        if (item.disp === 1 || item.disp === true) {
          chartCtx.fillStyle = '#dc2626';
          chartCtx.beginPath();
          chartCtx.arc(x, padT + plotH - 3, 4.5, 0, Math.PI * 2);
          chartCtx.fill();
        }

        if (rVal >= 70) {
          chartCtx.fillStyle = '#dc2626';
          chartCtx.beginPath();
          chartCtx.arc(x, y, 3.5, 0, Math.PI * 2);
          chartCtx.fill();
        } else if (rVal >= 45) {
          chartCtx.fillStyle = '#d97706';
          chartCtx.beginPath();
          chartCtx.arc(x, y, 2.5, 0, Math.PI * 2);
          chartCtx.fill();
        }
      }

      // 4. X-Axis Time Labels
      chartCtx.fillStyle = '#64748b';
      chartCtx.font = '8.5px sans-serif';
      chartCtx.textAlign = 'center';
      chartCtx.textBaseline = 'top';

      const labelSteps = Math.min(count, 5);
      for (let j = 0; j < labelSteps; j++) {
        const idx = Math.floor(j * (count - 1) / (labelSteps - 1));
        const x = padL + idx * xStep;
        const timeTxt = 'T+' + formatRelativeTime(logs[idx].time);
        chartCtx.fillText(timeTxt, x, padT + plotH + 7);
      }

      chartCtx.restore();
    }

    // Municipal System Audit Trail & Event Registry Engine
    let auditEventsData = [];
    let currentAuditFilter = 'ALL';
    let isAuditSortChronological = true; // Chronological order (oldest first)

    async function fetchAuditEvents() {
      if (!activeKey) return;
      try {
        const res = await fetch('/api/admin/events?key=' + encodeURIComponent(activeKey));
        if (!res.ok) throw new Error();
        const d = await res.json();
        if (d.success && Array.isArray(d.events)) {
          auditEventsData = d.events;
          const ab = document.getElementById('tabAuditBadge');
          if (ab) ab.textContent = d.events.length;
          const acb = document.getElementById('auditCountBadge');
          if (acb) acb.textContent = 'TOTAL: ' + d.events.length + ' EVENTS';
          renderAuditTable();
        }
      } catch (e) {
      }
    }

    function setAuditFilter(sev) {
      currentAuditFilter = sev;
      document.querySelectorAll('.filter-chip').forEach(c => {
        c.classList.toggle('active', c.getAttribute('data-sev') === sev);
      });
      renderAuditTable();
    }

    function toggleAuditSort() {
      isAuditSortChronological = !isAuditSortChronological;
      const btn = document.getElementById('auditSortBtn');
      if (btn) btn.textContent = isAuditSortChronological ? '⬇️ Chronological (Oldest First)' : '⬆️ Reverse (Newest First)';
      renderAuditTable();
    }

    function filterAuditEvents() {
      renderAuditTable();
    }

    function renderAuditTable() {
      const tbody = document.getElementById('auditTableBody');
      if (!tbody) return;

      const searchInput = document.getElementById('auditSearchInput');
      const search = (searchInput ? searchInput.value : '').toLowerCase().trim();
      
      let list = auditEventsData.filter(item => {
        if (currentAuditFilter !== 'ALL' && item.sev !== currentAuditFilter) return false;
        if (search.length > 0) {
          const txt = (item.msg + ' ' + item.cat + ' ' + item.src + ' ' + item.sev).toLowerCase();
          if (!txt.includes(search)) return false;
        }
        return true;
      });

      list.sort((a, b) => {
        return isAuditSortChronological ? (Number(a.id) - Number(b.id)) : (Number(b.id) - Number(a.id));
      });

      if (list.length === 0) {
        tbody.innerHTML = '<tr><td style="text-align: center; color: var(--text-muted); padding: 26px 10px;">No matching audit records found.</td></tr>';
        return;
      }

      let html = '';
      for (let i = 0; i < list.length; i++) {
        const item = list[i];
        let sevClass = 'tag-safe';
        let sevStyle = '';
        if (item.sev === 'CRITICAL') sevClass = 'tag-crimson';
        else if (item.sev === 'WARNING') sevClass = 'tag-amber';
        else if (item.sev === 'INFO') {
          sevClass = 'tag-safe';
          sevStyle = 'background:var(--blue-light); color:var(--blue); border: 1px solid var(--border-accent);';
        }

        html += '<tr><td>' +
          '<div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:5px; flex-wrap:wrap; gap:4px;">' +
            '<div style="display:flex; align-items:center; gap:6px;">' +
              '<span style="font-family:monospace; font-weight:800; font-size:0.75rem; color:var(--text-muted);">#' + item.id + '</span>' +
              '<strong style="font-size:0.78rem; color:var(--navy-gov);">T+' + formatRelativeTime(item.time) + '</strong>' +
            '</div>' +
            '<span class="risk-state-tag ' + sevClass + '" style="font-size:0.65rem; padding:2px 8px;' + sevStyle + '">' + escapeHtml(item.sev) + '</span>' +
          '</div>' +
          '<div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:5px; font-size:0.72rem; color:var(--text-dim);">' +
            '<span>📁 <strong>' + escapeHtml(item.cat) + '</strong></span>' +
            '<span>👤 <strong style="color:var(--navy-gov);">' + escapeHtml(item.src) + '</strong></span>' +
          '</div>' +
          '<div style="font-size:0.78rem; line-height:1.45; color:var(--text); word-break:break-word; background:var(--surface-subtle); padding:6px 8px; border-radius:6px; border:1px solid var(--border); box-sizing:border-box;">' +
            escapeHtml(item.msg) +
          '</div>' +
          '</td></tr>';
      }
      tbody.innerHTML = html;
    }

    function escapeHtml(str) {
      if (!str) return '';
      return String(str).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;');
    }

    function exportAuditCsv() {
      if (!auditEventsData || auditEventsData.length === 0) {
        alert('No audit events recorded to export.');
        return;
      }
      let csv = 'ID,Uptime_Seconds,Relative_Timestamp,Category,Severity,Source,Action_Description\r\n';
      auditEventsData.forEach(item => {
        csv += item.id + ',' + item.time + ',"T+' + formatRelativeTime(item.time) + '","' + item.cat + '","' + item.sev + '","' + item.src + '","' + item.msg.replace(/"/g, '""') + '"\r\n';
      });
      const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = 'landslide_system_audit_log_' + Math.floor(Date.now() / 1000) + '.csv';
      document.body.appendChild(a);
      a.click();
      document.body.removeChild(a);
    }

    async function clearAuditEvents() {
      if (!activeKey) return;
      if (!confirm('Permanently purge the municipal system audit log from NodeMCU internal flash memory?')) return;
      try {
        const res = await fetch('/api/admin/events/clear?key=' + encodeURIComponent(activeKey), { method: 'POST' });
        const d = await res.json();
        if (d.success) {
          fetchAuditEvents();
        }
      } catch (e) {
        alert('Failed to clear audit trail: ' + e);
      }
    }

    // Periodic historical telemetry & audit event refresh
    setInterval(fetchHistoricalLogs, 15000);
    setInterval(fetchAuditEvents, 15000);
  </script>
</body>
</html>
)rawliteral";

void handleTelemetry() {
  computeLandslideRisk();

  unsigned long now = millis();
  int recentVibHits = 0;
  for (int i = 0; i < VIB_RING_SIZE; i++) {
    if (vibTimestamps[i] > 0 && (now - vibTimestamps[i] < (VIB_WINDOW_SECS * 1000))) {
      recentVibHits++;
    }
  }

  int rawAdc = analogRead(SOIL_ANALOG_PIN);
  int clampedAdc = constrain(rawAdc, WET_ADC_VALUE, DRY_ADC_VALUE);
  int moisturePct = map(clampedAdc, DRY_ADC_VALUE, WET_ADC_VALUE, 0, 100);

  int irLevel = digitalRead(IR_PIN);
  bool irObstacle = (irLevel == LOW);

  float deltaPitch = abs(pitch - baselinePitch);
  float deltaRoll  = abs(roll  - baselineRoll);
  float totalTiltDelta = sqrt(deltaPitch * deltaPitch + deltaRoll * deltaRoll);

  String gateStr = "OPEN";
  if (gateStatus == GATE_CLOSED) gateStr = "CLOSED";
  else if (gateStatus == GATE_CLOSING) gateStr = "CLOSING";
  else if (gateStatus == GATE_OPENING) gateStr = "OPENING";

  String json = "{";
  // Emergency & Countdown
  json += "\"emergency\":{";
  json += "\"active\":" + String(emergencyActive ? "true" : "false") + ",";
  json += "\"countdown\":" + String(remainingCountdownSecs) + ",";
  json += "\"countdown_duration\":" + String(countdownDurationSecs) + ",";
  json += "\"buzzer\":" + String(buzzerActive ? "true" : "false") + ",";
  json += "\"buzzer_muted\":" + String(buzzerMuted ? "true" : "false");
  json += "},";
  // Gate Status
  json += "\"gate\":{";
  json += "\"status\":\"" + gateStr + "\",";
  json += "\"moving\":" + String(servoMoving ? "true" : "false");
  json += "},";
  // Landslide Fusion Risk & Breakdown Metrics
  json += "\"risk\":{";
  json += "\"percentage\":" + String(cumulativeRiskPct, 1) + ",";
  json += "\"instant\":" + String(lastInstantRisk, 1) + ",";
  json += "\"concurrent_abnormal\":" + String(concurrentAbnormalCount) + ",";
  json += "\"tilt_delta\":" + String(totalTiltDelta, 1) + ",";
  json += "\"tilt_score\":" + String(lastScoreTilt, 1) + ",";
  json += "\"vib_score\":" + String(lastScoreVib, 1) + ",";
  json += "\"disp_score\":" + String(lastScoreDisp, 1) + ",";
  json += "\"soil_score\":" + String(lastScoreSoil, 1) + ",";
  json += "\"soil_mult\":" + String(lastSoilMultiplier, 2) + ",";
  json += "\"syn_liquefaction\":" + String(lastSynergySoilAndVib ? "true" : "false") + ",";
  json += "\"syn_slip\":" + String(lastSynergyTiltAndVib ? "true" : "false") + ",";
  json += "\"syn_rupture\":" + String(lastSynergyTripleCritical ? "true" : "false") + ",";
  json += "\"suppression\":" + String(lastSuppressionActive ? "true" : "false");
  json += "},";
  // MPU-6050
  json += "\"mpu\":{";
  json += "\"pitch\":" + String(pitch, 2) + ",";
  json += "\"roll\":" + String(roll, 2);
  json += "},";
  // Soil
  json += "\"soil\":{";
  json += "\"percentage\":" + String(moisturePct);
  json += "},";
  // Vibration
  json += "\"vibration\":{";
  json += "\"window_hits\":" + String(recentVibHits) + ",";
  json += "\"total_hits\":" + String(totalVibCount);
  json += "},";
  // IR
  json += "\"ir\":{";
  json += "\"obstacle\":" + String(irObstacle ? "true" : "false");
  json += "},";
  // System
  json += "\"system\":{";
  json += "\"clients\":" + String(WiFi.softAPgetStationNum()) + ",";
  json += "\"uptime\":" + String(millis() / 1000);
  json += "}";
  json += "}";

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

void handleCaptiveRedirect() {
  server.sendHeader("Location", String("http://") + apIP.toString() + "/", true);
  server.send(302, "text/plain", "");
}

// Extract authentication key from query param, form body, or JSON
String getRequestKey() {
  if (server.hasArg("key")) return server.arg("key");
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    int idx = body.indexOf("\"key\":\"");
    if (idx != -1) {
      int endIdx = body.indexOf("\"", idx + 7);
      if (endIdx != -1) return body.substring(idx + 7, endIdx);
    }
  }
  return "";
}

// ---------------------- Two-Tier Web Request Handlers --------------------

// 1. Public Visitor Landing Page (/)
void handleRoot() {
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
  server.send(200, "text/html", PUBLIC_HTML);
}

// 2. Protected Admin Dashboard (/admin)
void handleAdmin() {
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
  server.send(200, "text/html", ADMIN_HTML);
}

// 3. Admin Authentication Endpoint (/api/admin/verify)
void handleAdminVerify() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  String key = getRequestKey();
  if (key == ADMIN_KEY) {
    logAuditEvent("SECURITY", "INFO", "Web Console", "Admin console session authenticated successfully.");
    server.send(200, "application/json", "{\"auth\":true,\"status\":\"AUTHORIZED\"}");
  } else {
    logAuditEvent("SECURITY", "WARNING", "Web Console", "Failed admin authentication attempt: Invalid passkey.");
    server.send(401, "application/json", "{\"auth\":false,\"error\":\"Invalid credentials\"}");
  }
}

// 4. Configurable Timer Settings Endpoint (/api/admin/config)
void handleAdminConfig() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  String key = getRequestKey();
  if (key != ADMIN_KEY) {
    server.send(401, "application/json", "{\"success\":false,\"error\":\"UNAUTHORIZED: Admin passkey required\"}");
    return;
  }

  String durationStr = "";
  if (server.hasArg("countdown")) {
    durationStr = server.arg("countdown");
  } else if (server.hasArg("plain")) {
    String body = server.arg("plain");
    int idx = body.indexOf("\"countdown\":");
    if (idx != -1) {
      int start = idx + 12;
      while (start < (int)body.length() && (body[start] == ' ' || body[start] == '"')) start++;
      int end = start;
      while (end < (int)body.length() && isDigit(body[end])) end++;
      durationStr = body.substring(start, end);
    }
  }

  if (durationStr.length() > 0) {
    int val = durationStr.toInt();
    if (val >= 5 && val <= 300) {
      bool saved = saveCountdownConfig((unsigned int)val);
      logAuditEvent("CONFIG", "INFO", "EEPROM Config", "Evacuation countdown duration reconfigured to " + String(val) + "s in EEPROM.");
      server.send(200, "application/json", "{\"success\":true,\"countdown\":" + String(countdownDurationSecs) + ",\"persisted\":" + String(saved ? "true" : "false") + "}");
      return;
    } else {
      server.send(400, "application/json", "{\"success\":false,\"error\":\"Duration must be between 5 and 300 seconds\"}");
      return;
    }
  }

  server.send(200, "application/json", "{\"success\":true,\"countdown\":" + String(countdownDurationSecs) + "}");
}

// 5. Exclusive Manual Gate Open (/api/gate/open)
void handleGateOpen() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  String key = getRequestKey();
  if (key != ADMIN_KEY) {
    server.send(401, "application/json", "{\"success\":false,\"error\":\"UNAUTHORIZED: Admin passkey required\"}");
    return;
  }
  triggerGateOpen();
  logAuditEvent("ACTUATION", "SUCCESS", "Operator Web UI", "Manual gate OPEN override executed (375ms clockwise). Road re-opened.");
  server.send(200, "application/json", "{\"success\":true,\"status\":\"OPENING\"}");
}

// 6. Exclusive Manual Gate Close (/api/gate/close)
void handleGateClose() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  String key = getRequestKey();
  if (key != ADMIN_KEY) {
    server.send(401, "application/json", "{\"success\":false,\"error\":\"UNAUTHORIZED: Admin passkey required\"}");
    return;
  }
  triggerGateClose();
  logAuditEvent("ACTUATION", "WARNING", "Operator Web UI", "Manual gate CLOSE override executed (375ms anticlockwise). Emergency road blockade engaged.");
  server.send(200, "application/json", "{\"success\":true,\"status\":\"CLOSING\"}");
}

// ---------------------- LittleFS Historical Data Logger ------------------
bool littleFsAvailable = false;
const size_t MAX_LOG_FILE_SIZE = 16384; // 16KB limit to protect flash memory
unsigned long lastLogTime = 0;
const unsigned long LOG_INTERVAL_MS = 10000; // Log snapshot every 10 seconds
float lastLoggedRisk = 0.0;
unsigned long nextAuditId = 1;

void initLittleFS() {
  if (LittleFS.begin()) {
    littleFsAvailable = true;
    Serial.println(F("[LittleFS] Internal flash file system mounted successfully."));
    if (!LittleFS.exists("/telemetry_log.csv")) {
      File f = LittleFS.open("/telemetry_log.csv", "w");
      if (f) {
        f.println(F("time,risk,soil,vib,disp,gate"));
        f.close();
      }
    }
    if (!LittleFS.exists("/audit_log.csv")) {
      File af = LittleFS.open("/audit_log.csv", "w");
      if (af) {
        af.println(F("id,time,category,severity,source,message"));
        af.close();
      }
    } else {
      File countF = LittleFS.open("/audit_log.csv", "r");
      if (countF) {
        while (countF.available()) {
          String l = countF.readStringUntil('\n');
          l.trim();
          int c1 = l.indexOf(',');
          if (c1 != -1) {
            unsigned long idVal = l.substring(0, c1).toInt();
            if (idVal >= nextAuditId) nextAuditId = idVal + 1;
          }
        }
        countF.close();
      }
    }
  } else {
    Serial.println(F("[LittleFS] Mount failed. Formatting filesystem..."));
    if (LittleFS.format() && LittleFS.begin()) {
      littleFsAvailable = true;
      Serial.println(F("[LittleFS] Formatted and mounted successfully."));
      File f = LittleFS.open("/telemetry_log.csv", "w");
      if (f) {
        f.println(F("time,risk,soil,vib,disp,gate"));
        f.close();
      }
      File af = LittleFS.open("/audit_log.csv", "w");
      if (af) {
        af.println(F("id,time,category,severity,source,message"));
        af.close();
      }
    } else {
      Serial.println(F("[LittleFS] Critical Error: Failed to initialize LittleFS."));
    }
  }
}

void pruneLogFile() {
  File src = LittleFS.open("/telemetry_log.csv", "r");
  if (!src) return;
  int lineCount = 0;
  while (src.available()) {
    String l = src.readStringUntil('\n');
    if (l.length() > 0) lineCount++;
  }
  src.seek(0, SeekSet);
  int skipCount = lineCount / 2;
  File dst = LittleFS.open("/telemetry_tmp.csv", "w");
  if (!dst) { src.close(); return; }
  
  dst.println(F("time,risk,soil,vib,disp,gate"));
  int currentLine = 0;
  while (src.available()) {
    String l = src.readStringUntil('\n');
    l.trim();
    if (currentLine > 0 && currentLine > skipCount && l.length() > 0) {
      dst.println(l);
    }
    currentLine++;
  }
  src.close();
  dst.close();
  LittleFS.remove("/telemetry_log.csv");
  LittleFS.rename("/telemetry_tmp.csv", "/telemetry_log.csv");
  Serial.println(F("[LittleFS] Historical log pruned to maintain safe flash limits."));
}

void logTelemetrySnapshot(bool forceLog) {
  if (!littleFsAvailable) return;
  unsigned long now = millis();
  if (!forceLog && (now - lastLogTime < LOG_INTERVAL_MS) && abs(cumulativeRiskPct - lastLoggedRisk) < 10.0) {
    return;
  }
  lastLogTime = now;
  lastLoggedRisk = cumulativeRiskPct;

  int recentVibHits = 0;
  for (int i = 0; i < VIB_RING_SIZE; i++) {
    if (vibTimestamps[i] > 0 && (now - vibTimestamps[i] < (VIB_WINDOW_SECS * 1000))) {
      recentVibHits++;
    }
  }
  int rawAdc = analogRead(SOIL_ANALOG_PIN);
  int clampedAdc = constrain(rawAdc, WET_ADC_VALUE, DRY_ADC_VALUE);
  int moisturePct = map(clampedAdc, DRY_ADC_VALUE, WET_ADC_VALUE, 0, 100);
  int irLevel = digitalRead(IR_PIN);
  bool irObstacle = (irLevel == LOW);

  String gateStr = "OPEN";
  if (gateStatus == GATE_CLOSED) gateStr = "CLOSED";
  else if (gateStatus == GATE_CLOSING) gateStr = "CLOSING";
  else if (gateStatus == GATE_OPENING) gateStr = "OPENING";

  if (LittleFS.exists("/telemetry_log.csv")) {
    File checkF = LittleFS.open("/telemetry_log.csv", "r");
    if (checkF) {
      size_t sz = checkF.size();
      checkF.close();
      if (sz > MAX_LOG_FILE_SIZE) {
        pruneLogFile();
      }
    }
  }

  File f = LittleFS.open("/telemetry_log.csv", "a");
  if (f) {
    unsigned long timeSecs = now / 1000;
    f.print(timeSecs);
    f.print(',');
    f.print(String(cumulativeRiskPct, 1));
    f.print(',');
    f.print(moisturePct);
    f.print(',');
    f.print(recentVibHits);
    f.print(',');
    f.print(irObstacle ? 1 : 0);
    f.print(',');
    f.println(gateStr);
    f.close();
  }
}

// 7. Historical Telemetry Log Stream (/api/admin/logs)
void handleGetLogs() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  String key = getRequestKey();
  if (key != ADMIN_KEY) {
    server.send(401, "application/json", "{\"success\":false,\"error\":\"UNAUTHORIZED: Admin passkey required\"}");
    return;
  }
  if (!littleFsAvailable || !LittleFS.exists("/telemetry_log.csv")) {
    server.send(200, "application/json", "{\"success\":true,\"count\":0,\"logs\":[]}");
    return;
  }

  File f = LittleFS.open("/telemetry_log.csv", "r");
  if (!f) {
    server.send(500, "application/json", "{\"success\":false,\"error\":\"Failed to open log file\"}");
    return;
  }

  int totalLines = 0;
  while (f.available()) {
    String l = f.readStringUntil('\n');
    l.trim();
    if (l.length() > 0) totalLines++;
  }
  if (totalLines <= 1) {
    f.close();
    server.send(200, "application/json", "{\"success\":true,\"count\":0,\"logs\":[]}");
    return;
  }

  int dataLines = totalLines - 1;
  int maxToSend = 100;
  int skipLines = (dataLines > maxToSend) ? (dataLines - maxToSend) : 0;

  f.seek(0, SeekSet);
  f.readStringUntil('\n'); // skip CSV header

  for (int i = 0; i < skipLines; i++) {
    if (f.available()) f.readStringUntil('\n');
  }

  String json = "{\"success\":true,\"logs\":[";
  bool first = true;
  int count = 0;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    int c3 = line.indexOf(',', c2 + 1);
    int c4 = line.indexOf(',', c3 + 1);
    int c5 = line.indexOf(',', c4 + 1);

    if (c1 != -1 && c2 != -1 && c3 != -1 && c4 != -1 && c5 != -1) {
      String t = line.substring(0, c1);
      String r = line.substring(c1 + 1, c2);
      String s = line.substring(c2 + 1, c3);
      String v = line.substring(c3 + 1, c4);
      String d = line.substring(c4 + 1, c5);
      String g = line.substring(c5 + 1);

      if (!first) json += ",";
      json += "{\"time\":" + t + ",\"risk\":" + r + ",\"soil\":" + s + ",\"vib\":" + v + ",\"disp\":" + d + ",\"gate\":\"" + g + "\"}";
      first = false;
      count++;
    }
  }
  f.close();

  json += "],\"count\":" + String(count) + "}";
  server.send(200, "application/json", json);
}

// 8. Clear Historical Telemetry Log (/api/admin/logs/clear)
void handleClearLogs() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  String key = getRequestKey();
  if (key != ADMIN_KEY) {
    server.send(401, "application/json", "{\"success\":false,\"error\":\"UNAUTHORIZED: Admin passkey required\"}");
    return;
  }
  if (littleFsAvailable) {
    File f = LittleFS.open("/telemetry_log.csv", "w");
    if (f) {
      f.println(F("time,risk,soil,vib,disp,gate"));
      f.close();
    }
  }
  server.send(200, "application/json", "{\"success\":true,\"message\":\"Historical logs purged successfully.\"}");
}

// ---------------------- System Audit Trail & Event Logger -----------------
const size_t MAX_AUDIT_FILE_SIZE = 16384; // 16KB limit to protect flash memory

void pruneAuditLog() {
  File src = LittleFS.open("/audit_log.csv", "r");
  if (!src) return;
  int lineCount = 0;
  while (src.available()) {
    String l = src.readStringUntil('\n');
    if (l.length() > 0) lineCount++;
  }
  src.seek(0, SeekSet);
  int skipCount = lineCount / 2;
  File dst = LittleFS.open("/audit_tmp.csv", "w");
  if (!dst) { src.close(); return; }
  
  dst.println(F("id,time,category,severity,source,message"));
  int currentLine = 0;
  while (src.available()) {
    String l = src.readStringUntil('\n');
    l.trim();
    if (currentLine > 0 && currentLine > skipCount && l.length() > 0) {
      dst.println(l);
    }
    currentLine++;
  }
  src.close();
  dst.close();
  LittleFS.remove("/audit_log.csv");
  LittleFS.rename("/audit_tmp.csv", "/audit_log.csv");
  Serial.println(F("[LittleFS] Audit trail log pruned to maintain safe flash limits."));
}

void logAuditEvent(const char* category, const char* severity, const char* source, const String& message) {
  if (!littleFsAvailable) return;
  
  // Sanitize message: replace commas and newlines with spaces or semicolons
  String sanitizedMsg = message;
  sanitizedMsg.replace(',', ';');
  sanitizedMsg.replace('\n', ' ');
  sanitizedMsg.replace('\r', ' ');
  sanitizedMsg.replace('\"', '\'');

  String catStr = String(category);
  catStr.replace(',', ';');
  String sevStr = String(severity);
  sevStr.replace(',', ';');
  String srcStr = String(source);
  srcStr.replace(',', ';');

  if (LittleFS.exists("/audit_log.csv")) {
    File checkF = LittleFS.open("/audit_log.csv", "r");
    if (checkF) {
      size_t sz = checkF.size();
      checkF.close();
      if (sz > MAX_AUDIT_FILE_SIZE) {
        pruneAuditLog();
      }
    }
  }

  File f = LittleFS.open("/audit_log.csv", "a");
  if (f) {
    unsigned long timeSecs = millis() / 1000;
    f.print(nextAuditId++);
    f.print(',');
    f.print(timeSecs);
    f.print(',');
    f.print(catStr);
    f.print(',');
    f.print(sevStr);
    f.print(',');
    f.print(srcStr);
    f.print(',');
    f.println(sanitizedMsg);
    f.close();
  }
  Serial.print(F("[AUDIT] ["));
  Serial.print(severity);
  Serial.print(F("] "));
  Serial.print(source);
  Serial.print(F(": "));
  Serial.println(sanitizedMsg);
}

// 9. Municipal System Audit Events Stream (/api/admin/events)
void handleGetAuditLogs() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  String key = getRequestKey();
  if (key != ADMIN_KEY) {
    server.send(401, "application/json", "{\"success\":false,\"error\":\"UNAUTHORIZED: Admin passkey required\"}");
    return;
  }
  if (!littleFsAvailable || !LittleFS.exists("/audit_log.csv")) {
    server.send(200, "application/json", "{\"success\":true,\"count\":0,\"events\":[]}");
    return;
  }

  File f = LittleFS.open("/audit_log.csv", "r");
  if (!f) {
    server.send(500, "application/json", "{\"success\":false,\"error\":\"Failed to open audit log\"}");
    return;
  }

  int totalLines = 0;
  while (f.available()) {
    String l = f.readStringUntil('\n');
    l.trim();
    if (l.length() > 0) totalLines++;
  }
  if (totalLines <= 1) {
    f.close();
    server.send(200, "application/json", "{\"success\":true,\"count\":0,\"events\":[]}");
    return;
  }

  int dataLines = totalLines - 1;
  int maxToSend = 150;
  int skipLines = (dataLines > maxToSend) ? (dataLines - maxToSend) : 0;

  f.seek(0, SeekSet);
  f.readStringUntil('\n'); // skip CSV header

  for (int i = 0; i < skipLines; i++) {
    if (f.available()) f.readStringUntil('\n');
  }

  String json = "{\"success\":true,\"events\":[";
  bool first = true;
  int count = 0;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    int c3 = line.indexOf(',', c2 + 1);
    int c4 = line.indexOf(',', c3 + 1);
    int c5 = line.indexOf(',', c4 + 1);

    if (c1 != -1 && c2 != -1 && c3 != -1 && c4 != -1 && c5 != -1) {
      String id = line.substring(0, c1);
      String t = line.substring(c1 + 1, c2);
      String cat = line.substring(c2 + 1, c3);
      String sev = line.substring(c3 + 1, c4);
      String src = line.substring(c4 + 1, c5);
      String msg = line.substring(c5 + 1);

      if (!first) json += ",";
      json += "{\"id\":" + id + ",\"time\":" + t + ",\"cat\":\"" + cat + "\",\"sev\":\"" + sev + "\",\"src\":\"" + src + "\",\"msg\":\"" + msg + "\"}";
      first = false;
      count++;
    }
  }
  f.close();

  json += "],\"count\":" + String(count) + "}";
  server.send(200, "application/json", json);
}

// 10. Purge Municipal Audit Events (/api/admin/events/clear)
void handleClearAuditLogs() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  String key = getRequestKey();
  if (key != ADMIN_KEY) {
    server.send(401, "application/json", "{\"success\":false,\"error\":\"UNAUTHORIZED: Admin passkey required\"}");
    return;
  }
  if (littleFsAvailable) {
    File f = LittleFS.open("/audit_log.csv", "w");
    if (f) {
      f.println(F("id,time,category,severity,source,message"));
      f.close();
    }
    nextAuditId = 1;
    logAuditEvent("ADMIN_ACTION", "WARNING", "Web Console", "Audit log purged by administrator.");
  }
  server.send(200, "application/json", "{\"success\":true,\"message\":\"Audit trail purged successfully.\"}");
}

// 11. Remote Buzzer Mute Control (/api/buzzer/mute)
void handleBuzzerMute() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  String key = getRequestKey();
  if (key != ADMIN_KEY) {
    server.send(401, "application/json", "{\"success\":false,\"error\":\"UNAUTHORIZED: Admin passkey required\"}");
    return;
  }

  if (server.hasArg("state")) {
    String st = server.arg("state");
    buzzerMuted = (st == "1" || st == "true");
  } else {
    buzzerMuted = !buzzerMuted;
  }

  if (buzzerMuted) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerState = false;
    logAuditEvent("ACTUATION", "WARNING", "Operator Web UI", "Acoustic buzzer (GPIO15) remotely MUTED by administrator.");
  } else {
    logAuditEvent("ACTUATION", "INFO", "Operator Web UI", "Acoustic buzzer (GPIO15) remotely UNMUTED by administrator.");
  }

  server.send(200, "application/json", "{\"success\":true,\"muted\":" + String(buzzerMuted ? "true" : "false") + ",\"buzzer_active\":" + String(buzzerActive ? "true" : "false") + "}");
}

// ---------------------- Setup & Main Loop --------------------------------

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println(F("=========================================================="));
  Serial.println(F(" NodeMCU Landslide Warning & Safety Gate Control System  "));
  Serial.println(F("=========================================================="));

  // Initialize Persistent EEPROM Storage for Configurable Emergency Timing
  initConfigEEPROM();

  // Initialize LittleFS Flash File System for Historical Telemetry Logging
  initLittleFS();

  // ------------------------------------------------------------------------
  // STEP 1: INITIALIZE WI-FI HOTSPOT FIRST
  // Starting Wi-Fi immediately guarantees the Access Point is visible
  // even if sensors are disconnected, loose, or slow to initialize.
  // ------------------------------------------------------------------------
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  delay(100);

  WiFi.mode(WIFI_AP);
  WiFi.setSleepMode(WIFI_NONE_SLEEP); // Keep RF radio active
  WiFi.setOutputPower(20.5);          // Maximum 20.5 dBm RF transmit power

  // Start Open SoftAP (no password required): Channel 1, visible beacon (hidden=0), max 4 stations (ESP8266 hardware limit)
  bool apStarted = WiFi.softAP(AP_SSID, nullptr, 1, 0, 4);

  // Fallback: If initial launch fails, retry standard open SoftAP
  if (!apStarted) {
    Serial.println(F("[WiFi] Retrying SoftAP launch..."));
    apStarted = WiFi.softAP(AP_SSID);
  }

  delay(50);
  WiFi.softAPConfig(apIP, apIP, netMsk);

  if (apStarted) {
    Serial.println(F("[WiFi] =========================================="));
    Serial.print(F("[WiFi] HOTSPOT ACTIVE & BROADCASTING: "));
    Serial.println(AP_SSID);
    Serial.print(F("[WiFi] ACCESS POINT IP: "));
    Serial.println(WiFi.softAPIP());
    Serial.print(F("[WiFi] CHANNEL: 1 | MAX TX POWER: 20.5 dBm"));
    Serial.println();
    Serial.println(F("[WiFi] =========================================="));
  } else {
    Serial.println(F("[WiFi] CRITICAL ERROR: Failed to launch Hotspot!"));
  }

  // Captive Portal DNS Server (redirects all queries to 192.168.4.1)
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(DNS_PORT, "*", apIP);

  // Web Server Route Bindings (Two-Tier Architecture)
  server.on("/", HTTP_GET, handleRoot);
  server.on("/admin", HTTP_GET, handleAdmin);
  server.on("/api/admin/verify", HTTP_POST, handleAdminVerify);
  server.on("/api/admin/verify", HTTP_GET, handleAdminVerify);
  server.on("/api/admin/config", HTTP_POST, handleAdminConfig);
  server.on("/api/admin/config", HTTP_GET, handleAdminConfig);
  server.on("/api/telemetry", HTTP_GET, handleTelemetry);
  server.on("/api/gate/open", HTTP_POST, handleGateOpen);
  server.on("/api/gate/open", HTTP_GET, handleGateOpen);
  server.on("/api/gate/close", HTTP_POST, handleGateClose);
  server.on("/api/gate/close", HTTP_GET, handleGateClose);
  server.on("/api/admin/logs", HTTP_GET, handleGetLogs);
  server.on("/api/admin/logs", HTTP_POST, handleGetLogs);
  server.on("/api/admin/logs/clear", HTTP_POST, handleClearLogs);
  server.on("/api/admin/logs/clear", HTTP_GET, handleClearLogs);
  server.on("/api/admin/events", HTTP_GET, handleGetAuditLogs);
  server.on("/api/admin/events", HTTP_POST, handleGetAuditLogs);
  server.on("/api/admin/events/clear", HTTP_POST, handleClearAuditLogs);
  server.on("/api/admin/events/clear", HTTP_GET, handleClearAuditLogs);
  server.on("/api/buzzer/mute", HTTP_POST, handleBuzzerMute);
  server.on("/api/buzzer/mute", HTTP_GET, handleBuzzerMute);

  // OS Captive Portal Probes
  server.on("/hotspot-detect.html", handleCaptiveRedirect);
  server.on("/canonical.html", handleCaptiveRedirect);
  server.on("/generate_204", handleCaptiveRedirect);
  server.on("/gen_204", handleCaptiveRedirect);
  server.on("/ncsi.txt", handleCaptiveRedirect);
  server.on("/connecttest.txt", handleCaptiveRedirect);
  server.on("/redirect", handleCaptiveRedirect);

  server.onNotFound([]() {
    if (server.hostHeader() != apIP.toString()) {
      handleCaptiveRedirect();
    } else {
      server.send(404, "text/plain", "404: Not Found");
    }
  });

  server.begin();
  Serial.println(F("[HTTP] Local Web Server Listening on port 80"));

  // ------------------------------------------------------------------------
  // STEP 2: INITIALIZE ACTUATORS (Buzzer & Servo)
  // ------------------------------------------------------------------------
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); // Silent at startup

  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);

  // ------------------------------------------------------------------------
  // STEP 3: INITIALIZE DISCRETE SENSORS
  // ------------------------------------------------------------------------
  pinMode(IR_PIN, INPUT_PULLUP);
  pinMode(SOIL_DIGITAL_PIN, INPUT_PULLUP);
  pinMode(VIB_PIN, INPUT_PULLUP); // PULLUP prevents floating pin interrupt storms!

  // Attach hardware interrupt for SW-420 tremor detection
  attachInterrupt(digitalPinToInterrupt(VIB_PIN), handleVibrationInterrupt, CHANGE);
  lastIrState = digitalRead(IR_PIN);

  // ------------------------------------------------------------------------
  // STEP 4: INITIALIZE I2C BUS (MPU-6050) WITH TIMEOUT PROTECTION
  // ------------------------------------------------------------------------
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);           // 100kHz standard I2C for high noise immunity
  Wire.setClockStretchLimit(1500); // Prevent I2C clock-stretch lockups if disconnected

  mpuAvailable = initMPU6050();
  calibrationStartTime = millis();

  if (mpuAvailable) {
    Serial.println(F("[MPU] MPU-6050 IMU initialized at I2C 0x68"));
  } else {
    Serial.println(F("[MPU] NOTE: MPU-6050 not detected. Hotspot still operational!"));
  }

  Serial.println(F("[READY] System initialized and ready for connections."));
  logAuditEvent("SYSTEM", "INFO", "System Boot", "System initial boot completed. Open SoftAP [" + String(AP_SSID) + "] broadcasting at 192.168.4.1.");
}

void loop() {
  // Give Wi-Fi background stack CPU time for beacon transmissions
  dnsServer.processNextRequest();
  server.handleClient();
  yield();

  // Autonomous 100ms periodic background evaluation of landslide fusion engine
  unsigned long now = millis();
  if (now - lastAlgoEvaluationTime >= 100) {
    lastAlgoEvaluationTime = now;
    computeLandslideRisk();
  }

  // Periodic LittleFS snapshot logger (every 10s or upon significant risk spike)
  logTelemetrySnapshot();

  // Update non-blocking servo movement timer
  updateServoMovement();

  // Update non-blocking buzzer pattern
  updateBuzzerPattern();

  // IR Obstacle edge detection & debouncing
  int currentIrLevel = digitalRead(IR_PIN);
  if (currentIrLevel != lastIrState) {
    if (millis() - lastIrChangeTime > 50) {
      if (currentIrLevel == LOW) {
        irDetectionCount++;
        Serial.print(F("[IR] Ground displacement event! Count: "));
        Serial.println(irDetectionCount);
        logAuditEvent("SENSOR_ALERT", "WARNING", "IR Ground Sensor", "Ground surface displacement / rupture detected at D6.");
      }
      lastIrState = currentIrLevel;
      lastIrChangeTime = millis();
    }
  }
}
