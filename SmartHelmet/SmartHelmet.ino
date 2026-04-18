/*
 * Smart Helmet Firmware
 * Platform : ESP32
 *
 * Features:
 *  - Alcohol detection via MQ3 sensor
 *  - Helmet buckle (strap) detection
 *  - Ignition interlock via relay
 *  - Crash detection via MPU6050 accelerometer
 *  - GPS location reporting via TinyGPSPlus over HardwareSerial
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// ── Pin definitions ──────────────────────────────────────────────────────────
#define PIN_MQ3    34   // Analog input  – MQ3 alcohol sensor
#define PIN_BUCKLE 25   // Digital input (INPUT_PULLUP) – helmet buckle switch
#define PIN_RELAY  26   // Digital output – ignition relay

// ── Thresholds ───────────────────────────────────────────────────────────────
// MQ3: raw ADC value above which alcohol is considered detected
#define ALCOHOL_THRESHOLD   1500

// Crash: total acceleration magnitude (m/s²) above which a crash is flagged.
// 1 g ≈ 9.81 m/s².  A sudden impact typically exceeds 2.5 g (≈ 24.5 m/s²).
#define CRASH_ACCEL_THRESHOLD_MS2  24.5f

// Minimum number of consecutive samples that must exceed the threshold before
// a crash is confirmed (simple debounce / noise rejection).
#define CRASH_CONFIRM_SAMPLES 3

// ── GPS serial ───────────────────────────────────────────────────────────────
#define GPS_BAUD   9600
HardwareSerial gpsSerial(1);   // UART1 – TX=GPIO17, RX=GPIO16
TinyGPSPlus    gps;

// ── MPU6050 ──────────────────────────────────────────────────────────────────
Adafruit_MPU6050 mpu;

// ── Forward declarations ──────────────────────────────────────────────────────
bool detectCrash();
bool isAlcoholDetected();
bool isBuckleFastened();
void sendGPSAlert(const char* reason);

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // Pin modes
  pinMode(PIN_BUCKLE, INPUT_PULLUP);
  pinMode(PIN_RELAY,  OUTPUT);
  digitalWrite(PIN_RELAY, LOW);   // relay open → engine disabled at startup

  // GPS serial
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, 16, 17);

  // MPU6050 initialization
  if (!mpu.begin()) {
    Serial.println("[ERROR] MPU6050 not found – check wiring.");
    while (true) { delay(10); }
  }

  // Configure sensor ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("[INFO] Smart Helmet firmware ready.");
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  // Feed GPS data
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  bool buckle   = isBuckleFastened();
  bool alcohol  = isAlcoholDetected();
  bool crash    = detectCrash();

  // Ignition interlock: allow engine only when buckle is fastened and no
  // alcohol is detected
  if (buckle && !alcohol) {
    digitalWrite(PIN_RELAY, HIGH);  // relay closed → engine enabled
  } else {
    digitalWrite(PIN_RELAY, LOW);   // relay open  → engine disabled
    if (!buckle)   Serial.println("[WARN] Buckle not fastened.");
    if (alcohol)   Serial.println("[WARN] Alcohol detected.");
  }

  // Crash response
  if (crash) {
    Serial.println("[ALERT] Crash detected!");
    digitalWrite(PIN_RELAY, LOW);   // cut ignition immediately
    sendGPSAlert("Crash detected");
  }

  delay(100);   // 10 Hz main loop
}

// ─────────────────────────────────────────────────────────────────────────────
/**
 * detectCrash()
 *
 * Reads the MPU6050 accelerometer and flags a crash when the total
 * acceleration vector magnitude exceeds CRASH_ACCEL_THRESHOLD_MS2 for at
 * least CRASH_CONFIRM_SAMPLES consecutive samples.
 *
 * Crash detection logic:
 *   1. Obtain a fresh sensor event from the MPU6050.
 *   2. Compute the Euclidean magnitude of the 3-axis acceleration:
 *        |a| = sqrt(ax² + ay² + az²)
 *      Under normal conditions this equals roughly 9.81 m/s² (1 g, gravity).
 *      A sudden impact produces a spike well above this baseline.
 *   3. Increment a consecutive-sample counter while |a| > threshold; reset it
 *      when |a| falls back within normal range.  This rejects brief electrical
 *      noise spikes.
 *   4. Return true only when the counter reaches CRASH_CONFIRM_SAMPLES,
 *      which the caller (loop) is responsible for acting on.
 *
 * @return true if a crash is confirmed, false otherwise.
 */
bool detectCrash() {
  sensors_event_t accelEvent, gyroEvent, tempEvent;
  mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);

  // Accelerometer axes are in m/s²
  float ax = accelEvent.acceleration.x;
  float ay = accelEvent.acceleration.y;
  float az = accelEvent.acceleration.z;

  // Total acceleration magnitude
  float magnitude = sqrtf(ax * ax + ay * ay + az * az);

  // Persistent counter across calls
  static int consecutiveSamples = 0;

  if (magnitude > CRASH_ACCEL_THRESHOLD_MS2) {
    consecutiveSamples++;
    Serial.printf("[DEBUG] Crash candidate – |a| = %.2f m/s² (sample %d/%d)\n",
                  magnitude, consecutiveSamples, CRASH_CONFIRM_SAMPLES);
  } else {
    consecutiveSamples = 0;   // reset on any reading within normal range
  }

  if (consecutiveSamples >= CRASH_CONFIRM_SAMPLES) {
    consecutiveSamples = 0;   // reset so the alert fires only once per event
    return true;
  }

  return false;
}

// ─────────────────────────────────────────────────────────────────────────────
/**
 * isAlcoholDetected()
 *
 * Reads the MQ3 sensor ADC and returns true when the raw value exceeds
 * ALCOHOL_THRESHOLD.
 */
bool isAlcoholDetected() {
  int raw = analogRead(PIN_MQ3);
  return (raw > ALCOHOL_THRESHOLD);
}

// ─────────────────────────────────────────────────────────────────────────────
/**
 * isBuckleFastened()
 *
 * Returns true when the buckle switch is closed.  The pin is configured with
 * INPUT_PULLUP so an open circuit reads HIGH (buckle open) and a closed
 * circuit reads LOW (buckle fastened).
 */
bool isBuckleFastened() {
  return (digitalRead(PIN_BUCKLE) == LOW);
}

// ─────────────────────────────────────────────────────────────────────────────
/**
 * sendGPSAlert()
 *
 * Prints the current GPS coordinates (if a valid fix is available) together
 * with a reason string to Serial.  In a production system this would dispatch
 * an SMS/MQTT/HTTP message.
 */
void sendGPSAlert(const char* reason) {
  if (gps.location.isValid()) {
    Serial.printf("[GPS ALERT] %s | Lat: %.6f, Lng: %.6f\n",
                  reason,
                  gps.location.lat(),
                  gps.location.lng());
  } else {
    Serial.printf("[GPS ALERT] %s | Location unavailable\n", reason);
  }
}
