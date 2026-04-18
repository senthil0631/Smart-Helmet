/**
 * Smart Helmet Firmware for ESP32
 *
 * Features:
 *  1. Ignition Interlock  – MQ-3 alcohol sensor + strap-buckle switch → relay
 *  2. Speed Alert         – Neo-6M GPS module → piezo buzzer
 *  3. Crash SOS           – MPU6050 IMU → SIM800L GSM (SMS + GPS coordinates)
 *
 * Hardware connections (all GPIO numbers refer to the ESP32 dev-board):
 *
 *  Sensor / Actuator       | ESP32 Pin
 *  ------------------------|----------
 *  MQ-3 analog out         | GPIO 34  (ADC1_CH6 – input only)
 *  Strap-buckle switch     | GPIO 27  (active-HIGH when fastened)
 *  Ignition relay          | GPIO 26  (HIGH = ignition enabled)
 *  Piezo buzzer            | GPIO 25
 *  GPS TX  → ESP32 RX2     | GPIO 16  (UART2 RX)
 *  GPS RX  ← ESP32 TX2     | GPIO 17  (UART2 TX)
 *  GSM TX  → ESP32 RX1     | GPIO  4  (UART1 RX) – SIM800L → ESP32
 *  GSM RX  ← ESP32 TX1     | GPIO  2  (UART1 TX) – ESP32 → SIM800L
 *  MPU6050 SDA             | GPIO 21  (I2C SDA)
 *  MPU6050 SCL             | GPIO 22  (I2C SCL)
 *
 * Required libraries (install via Arduino Library Manager):
 *  - TinyGPSPlus   by Mikal Hart
 *  - Adafruit MPU6050 + Adafruit Unified Sensor + Adafruit BusIO
 */

// ---------------------------------------------------------------------------
// Library inclusions
// ---------------------------------------------------------------------------
#include <Wire.h>               // I2C bus (MPU6050)
#include <Adafruit_MPU6050.h>   // IMU driver
#include <Adafruit_Sensor.h>    // Unified sensor abstraction
#include <TinyGPSPlus.h>        // GPS NMEA parser
#include <HardwareSerial.h>     // ESP32 hardware UART

// ---------------------------------------------------------------------------
// Pin definitions
// ---------------------------------------------------------------------------

// --- Ignition Interlock ---
#define PIN_MQ3_ANALOG    34    // MQ-3 alcohol sensor (ADC)
#define PIN_STRAP_SWITCH  27    // Strap-buckle switch (digital, active-HIGH)
#define PIN_RELAY         26    // Ignition relay (active-HIGH)

// --- Speed Alert ---
#define PIN_BUZZER        25    // Piezo buzzer (active-HIGH)

// GPS UART (UART2)
#define PIN_GPS_RX        16
#define PIN_GPS_TX        17

// GSM UART (UART1)
#define PIN_GSM_RX         4
#define PIN_GSM_TX         2

// MPU6050 uses default ESP32 I2C pins: SDA = 21, SCL = 22

// ---------------------------------------------------------------------------
// Configuration constants
// ---------------------------------------------------------------------------

// Alcohol threshold – ADC counts (0-4095 for 12-bit ESP32 ADC).
// Tune this value after calibrating your MQ-3 sensor.
#define ALCOHOL_THRESHOLD   1500

// Speed limit in km/h above which the buzzer triggers.
#define SPEED_LIMIT_KMPH    80.0

// Crash detection: net-acceleration threshold in m/s² (above 1g ≈ 9.8 m/s²
// means a sudden jolt on top of gravity).
#define CRASH_ACCEL_THRESHOLD   20.0   // m/s²  (~2 g)

// Emergency contact number for the SMS alert.
// *** IMPORTANT: Replace with the actual emergency contact number before deployment. ***
// Must be in international E.164 format, e.g. "+12025551234".
#define EMERGENCY_NUMBER    "+12025551234"

// Baud rates
#define SERIAL_MONITOR_BAUD 115200
#define GPS_BAUD            9600
#define GSM_BAUD            9600

// GSM timing constants (milliseconds)
// Increase GSM_NETWORK_REGISTRATION_DELAY_MS if the module fails to register
// in areas with poor signal.
#define GSM_NETWORK_REGISTRATION_DELAY_MS  5000UL
#define GSM_CMD_RESPONSE_DELAY_MS           500UL
#define GSM_SMS_SEND_DELAY_MS              3000UL

// Debug output rate-limiting: serial logs are printed at most once every this
// many milliseconds to avoid flooding the serial monitor.
#define DEBUG_PRINT_INTERVAL_MS            1000UL

// ---------------------------------------------------------------------------
// Global objects
// ---------------------------------------------------------------------------
TinyGPSPlus       gps;
Adafruit_MPU6050  mpu;

// ESP32 hardware serial ports
HardwareSerial    gpsSerial(2);   // UART2 for GPS
HardwareSerial    gsmSerial(1);   // UART1 for GSM (SIM800L)

// ---------------------------------------------------------------------------
// Forward declarations of feature functions
// ---------------------------------------------------------------------------
void checkIgnitionInterlock();
void checkSpeedAlert();
void checkCrashSOS();

// Helper utilities
void enableIgnition(bool enable);
void triggerBuzzer(bool on);
void sendSmsAlert(float latitude, float longitude);
void sendSmsAlertNoFix();
bool readGpsData();

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
    // --- Debug serial monitor ---
    Serial.begin(SERIAL_MONITOR_BAUD);
    Serial.println(F("[SmartHelmet] Booting..."));

    // --- GPIO configuration ---
    pinMode(PIN_MQ3_ANALOG,   INPUT);
    pinMode(PIN_STRAP_SWITCH, INPUT);
    pinMode(PIN_RELAY,        OUTPUT);
    pinMode(PIN_BUZZER,       OUTPUT);

    // Safe defaults: ignition OFF, buzzer OFF
    enableIgnition(false);
    triggerBuzzer(false);

    // --- GPS (UART2) ---
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    Serial.println(F("[GPS] Serial started"));

    // --- GSM / SIM800L (UART1) ---
    gsmSerial.begin(GSM_BAUD, SERIAL_8N1, PIN_GSM_RX, PIN_GSM_TX);
    Serial.println(F("[GSM] Serial started"));
    delay(GSM_NETWORK_REGISTRATION_DELAY_MS);  // Allow SIM800L to register on network

    // --- MPU6050 (I2C) ---
    Wire.begin();  // SDA = 21, SCL = 22 on ESP32
    if (!mpu.begin()) {
        Serial.println(F("[MPU6050] ERROR: Sensor not found! Check wiring."));
        // Non-fatal: crash detection will be disabled but other features work.
    } else {
        // Configure accelerometer range and bandwidth for impact detection.
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        Serial.println(F("[MPU6050] Initialised"));
    }

    Serial.println(F("[SmartHelmet] Setup complete. Entering main loop."));
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
    // Feed all available GPS bytes into the TinyGPS++ parser.
    readGpsData();

    // --- Feature 1: Ignition Interlock ---
    checkIgnitionInterlock();

    // --- Feature 2: Speed Alert ---
    checkSpeedAlert();

    // --- Feature 3: Crash SOS ---
    checkCrashSOS();

    delay(100);  // Small delay to prevent a tight loop
}

// ---------------------------------------------------------------------------
// Feature 1 – Ignition Interlock
//
// Logic: Ignition relay is enabled ONLY when:
//   (a) The strap-buckle switch is fastened (digital HIGH), AND
//   (b) The MQ-3 alcohol reading is below ALCOHOL_THRESHOLD.
// ---------------------------------------------------------------------------
void checkIgnitionInterlock() {
    // Read strap-buckle state (HIGH = fastened)
    bool strapFastened = (digitalRead(PIN_STRAP_SWITCH) == HIGH);

    // Read MQ-3 ADC value (12-bit: 0–4095)
    int alcoholLevel = analogRead(PIN_MQ3_ANALOG);

    bool alcoholSafe = (alcoholLevel < ALCOHOL_THRESHOLD);

    bool ignitionAllowed = strapFastened && alcoholSafe;
    enableIgnition(ignitionAllowed);

    // Rate-limited debug output to avoid flooding the serial monitor.
    static unsigned long lastPrintTime = 0;
    unsigned long now = millis();
    if ((now - lastPrintTime) >= DEBUG_PRINT_INTERVAL_MS) {
        lastPrintTime = now;
        Serial.print(F("[Interlock] Strap: "));
        Serial.print(strapFastened ? "ON " : "OFF");
        Serial.print(F("  Alcohol ADC: "));
        Serial.print(alcoholLevel);
        Serial.print(F("  Ignition: "));
        Serial.println(ignitionAllowed ? "ENABLED" : "DISABLED");
    }
}

// ---------------------------------------------------------------------------
// Feature 2 – Speed Alert
//
// Logic: If GPS has a valid speed fix and speed exceeds SPEED_LIMIT_KMPH,
//        activate the buzzer; otherwise silence it.
// ---------------------------------------------------------------------------
void checkSpeedAlert() {
    // Rate-limited serial output
    static unsigned long lastPrintTime = 0;
    unsigned long now = millis();
    bool doPrint = (now - lastPrintTime) >= DEBUG_PRINT_INTERVAL_MS;

    if (gps.speed.isValid()) {
        double speedKmph = gps.speed.kmph();

        if (speedKmph > SPEED_LIMIT_KMPH) {
            triggerBuzzer(true);
            if (doPrint) {
                lastPrintTime = now;
                Serial.print(F("[Speed] ALERT! Speed: "));
                Serial.print(speedKmph);
                Serial.println(F(" km/h – Buzzer ON"));
            }
        } else {
            triggerBuzzer(false);
            if (doPrint) {
                lastPrintTime = now;
                Serial.print(F("[Speed] OK: "));
                Serial.print(speedKmph);
                Serial.println(F(" km/h"));
            }
        }
    } else {
        // No valid GPS fix yet – keep buzzer off.
        triggerBuzzer(false);
        if (doPrint) {
            lastPrintTime = now;
            Serial.println(F("[Speed] Waiting for GPS fix..."));
        }
    }
}

// ---------------------------------------------------------------------------
// Feature 3 – Crash SOS
//
// Logic: Read linear acceleration from the MPU6050.  If the net acceleration
//        vector magnitude exceeds CRASH_ACCEL_THRESHOLD, treat it as a crash.
//        Send an SMS via the SIM800L GSM module containing the last known
//        GPS coordinates.
//
// A simple re-arm cooldown (CRASH_COOLDOWN_MS) prevents repeated alerts.
// ---------------------------------------------------------------------------
void checkCrashSOS() {
    static const unsigned long CRASH_COOLDOWN_MS = 60000UL;  // 60 s
    static unsigned long lastCrashTime = 0;

    // Fetch accelerometer data
    sensors_event_t accelEvent, gyroEvent, tempEvent;
    mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);

    float ax = accelEvent.acceleration.x;
    float ay = accelEvent.acceleration.y;
    float az = accelEvent.acceleration.z;

    // Compute net acceleration magnitude
    float netAccel = sqrt(ax * ax + ay * ay + az * az);

    if (netAccel > CRASH_ACCEL_THRESHOLD) {
        unsigned long now = millis();
        if ((now - lastCrashTime) > CRASH_COOLDOWN_MS) {
            lastCrashTime = now;
            Serial.print(F("[Crash] IMPACT detected! Accel: "));
            Serial.print(netAccel);
            Serial.println(F(" m/s²  → Sending SOS SMS..."));

            if (gps.location.isValid()) {
                // Send SMS with the actual GPS coordinates.
                sendSmsAlert((float)gps.location.lat(), (float)gps.location.lng());
            } else {
                // GPS fix not yet available – send alert without coordinates.
                sendSmsAlertNoFix();
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Helper: enableIgnition
// ---------------------------------------------------------------------------
void enableIgnition(bool enable) {
    digitalWrite(PIN_RELAY, enable ? HIGH : LOW);
}

// ---------------------------------------------------------------------------
// Helper: triggerBuzzer
// ---------------------------------------------------------------------------
void triggerBuzzer(bool on) {
    digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
}

// ---------------------------------------------------------------------------
// Helper: readGpsData
// Feeds bytes from the GPS serial port into the TinyGPS++ parser.
// Returns true if at least one new sentence was encoded.
// ---------------------------------------------------------------------------
bool readGpsData() {
    bool updated = false;
    while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
            updated = true;
        }
    }
    return updated;
}

// ---------------------------------------------------------------------------
// Helper: sendSmsAlert
// Sends an SMS via the SIM800L module using AT commands.
// ---------------------------------------------------------------------------
void sendSmsAlert(float latitude, float longitude) {
    // Switch to text mode
    gsmSerial.println(F("AT+CMGF=1"));
    delay(GSM_CMD_RESPONSE_DELAY_MS);

    // Set recipient number
    gsmSerial.print(F("AT+CMGS=\""));
    gsmSerial.print(EMERGENCY_NUMBER);
    gsmSerial.println(F("\""));
    delay(GSM_CMD_RESPONSE_DELAY_MS);

    // Compose message body
    gsmSerial.print(F("CRASH ALERT! Smart Helmet detected an impact. "));
    gsmSerial.print(F("Last known location: "));
    gsmSerial.print(F("https://maps.google.com/?q="));
    gsmSerial.print(latitude, 6);
    gsmSerial.print(F(","));
    gsmSerial.print(longitude, 6);

    // End message with Ctrl+Z (ASCII 26)
    gsmSerial.write(26);
    delay(GSM_SMS_SEND_DELAY_MS);  // Wait for the module to transmit

    Serial.println(F("[GSM] SOS SMS sent."));
}

// ---------------------------------------------------------------------------
// Helper: sendSmsAlertNoFix
// Sends a crash alert SMS when no GPS fix is available.
// ---------------------------------------------------------------------------
void sendSmsAlertNoFix() {
    gsmSerial.println(F("AT+CMGF=1"));
    delay(GSM_CMD_RESPONSE_DELAY_MS);

    gsmSerial.print(F("AT+CMGS=\""));
    gsmSerial.print(EMERGENCY_NUMBER);
    gsmSerial.println(F("\""));
    delay(GSM_CMD_RESPONSE_DELAY_MS);

    gsmSerial.print(F("CRASH ALERT! Smart Helmet detected an impact. "));
    gsmSerial.print(F("GPS location is currently unavailable."));

    gsmSerial.write(26);
    delay(GSM_SMS_SEND_DELAY_MS);

    Serial.println(F("[GSM] SOS SMS sent (no GPS fix)."));
}
