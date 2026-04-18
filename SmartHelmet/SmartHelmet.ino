/*
 * Smart Helmet – ESP32 firmware
 *
 * Features
 *   - Alcohol (MQ-3) + helmet-buckle ignition interlock
 *   - Hands-free navigation via GPS (TinyGPSPlus)
 *   - Speed alerts
 *   - Automated GPS crash notification (MPU-6050 impact detection)
 */

#include <Wire.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ---------------------------------------------------------------------------
// Pin definitions
// ---------------------------------------------------------------------------
static const int PIN_MQ3         = 34;   // MQ-3 alcohol sensor  – ADC input
static const int PIN_BUCKLE      = 25;   // Helmet buckle switch – digital input (LOW = buckled)
static const int PIN_RELAY       = 26;   // Ignition relay       – digital output (HIGH = allow ignition)

// MQ-3 raw ADC threshold (0–4095 on ESP32 12-bit ADC).
// Values above this level indicate alcohol is detected.
static const int ALCOHOL_THRESHOLD = 400;

// ---------------------------------------------------------------------------
// Peripheral objects
// ---------------------------------------------------------------------------
TinyGPSPlus      gps;
Adafruit_MPU6050 mpu;

HardwareSerial   gpsSerial(1);  // UART1 for GPS module (RX=GPIO16, TX=GPIO17)

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
void checkIgnitionInterlock();
void handleGPS();
void checkSpeed();
void checkCrash();

// ===========================================================================
// setup
// ===========================================================================
void setup() {
  Serial.begin(115200);

  // Ignition relay – default OFF (safe state)
  pinMode(PIN_RELAY,  OUTPUT);
  digitalWrite(PIN_RELAY, LOW);

  // Buckle switch – internal pull-up so the pin reads HIGH when unbuckled
  // and LOW when the buckle closes the circuit to GND
  pinMode(PIN_BUCKLE, INPUT_PULLUP);

  // MQ-3 is an analog input; no pinMode needed on most ESP32 boards,
  // but setting it explicitly keeps intent clear
  pinMode(PIN_MQ3, INPUT);

  // GPS serial
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);

  // IMU
  if (!mpu.begin()) {
    Serial.println("MPU-6050 not found – crash detection disabled");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  Serial.println("Smart Helmet ready");
}

// ===========================================================================
// loop
// ===========================================================================
void loop() {
  checkIgnitionInterlock();
  handleGPS();
  checkSpeed();
  checkCrash();
  delay(200);
}

// ===========================================================================
// checkIgnitionInterlock
//
// Reads the MQ-3 alcohol sensor and the helmet-buckle digital pin.
// The relay that controls the vehicle ignition is switched:
//   ON  – only when buckle is fastened AND no alcohol is detected
//   OFF – if the buckle is open OR alcohol is detected above threshold
// ===========================================================================
void checkIgnitionInterlock() {
  // --- 1. Read MQ-3 alcohol sensor (12-bit ADC: 0–4095) ---
  int alcoholLevel = analogRead(PIN_MQ3);

  // --- 2. Read buckle switch ---
  // Pin is pulled HIGH internally; buckle closing the circuit pulls it LOW
  bool buckled = (digitalRead(PIN_BUCKLE) == LOW);

  // --- 3. Evaluate conditions ---
  bool alcoholDetected = (alcoholLevel > ALCOHOL_THRESHOLD);

  if (buckled && !alcoholDetected) {
    // Safe to ride: helmet is on and rider is sober – enable ignition
    digitalWrite(PIN_RELAY, HIGH);
    Serial.println("Interlock: PASS – ignition enabled");
  } else {
    // Unsafe: disable ignition and report reason
    digitalWrite(PIN_RELAY, LOW);
    if (!buckled) {
      Serial.println("Interlock: FAIL – helmet buckle not fastened");
    }
    if (alcoholDetected) {
      Serial.print("Interlock: FAIL – alcohol detected (ADC=");
      Serial.print(alcoholLevel);
      Serial.println(")");
    }
  }
}

// ===========================================================================
// handleGPS  – feed NMEA sentences into TinyGPSPlus
// ===========================================================================
void handleGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
}

// ===========================================================================
// checkSpeed  – alert if speed exceeds limit
// ===========================================================================
void checkSpeed() {
  if (gps.speed.isUpdated()) {
    double speedKmh = gps.speed.kmph();
    if (speedKmh > 80.0) {
      Serial.print("Speed alert: ");
      Serial.print(speedKmh);
      Serial.println(" km/h");
    }
  }
}

// ===========================================================================
// checkCrash  – detect sudden deceleration via MPU-6050
// ===========================================================================
void checkCrash() {
  sensors_event_t accel, gyro, temp;
  if (!mpu.getEvent(&accel, &gyro, &temp)) {
    return;
  }

  // Compute magnitude of acceleration vector
  float ax = accel.acceleration.x;
  float ay = accel.acceleration.y;
  float az = accel.acceleration.z;
  float magnitude = sqrt(ax * ax + ay * ay + az * az);

  // Threshold: ~3 g (29.4 m/s²) indicates a crash-level impact
  if (magnitude > 29.4f) {
    Serial.print("CRASH DETECTED – acceleration magnitude: ");
    Serial.print(magnitude);
    Serial.println(" m/s²");
    if (gps.location.isValid()) {
      Serial.print("  Location: ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(", ");
      Serial.println(gps.location.lng(), 6);
    }
  }
}
