// ============================================================
//  LIBRARY INCLUDES
// ============================================================
#include <Arduino.h>
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <math.h>

// ============================================================
//  PIN DEFINITIONS
// ============================================================
#define PIN_DHT22            4     // DHT11 data pin (digital, 1-wire)
#define PIN_LDR              32    // LDR photocell analog input (ADC1_CH4)
#define PIN_POT_BRIGHTNESS   33    // Potentiometer for brightness (ADC1_CH5)
#define PIN_POT_PATIENT      34    // aLKDmc;ladmlckm Potentiometer simulating patient sensor (ADC1_CH6, input-only)
#define PIN_PIR              5     // PIR motion sensor (interrupt-capable)
#define PIN_ROOM_LED         2     // Automatic room lighting LED
#define PIN_BRIGHTNESS_LED   25    // PWM-controlled brightness LED
#define PIN_RGB_RED          26    // RGB LED red channel (PWM)
#define PIN_RGB_GREEN        16    // RGB LED green channel (PWM)
#define PIN_RGB_BLUE         27    // RGB LED blue channel (PWM)
#define PIN_LED_GREEN        13    // Green status LED (normal operation)
#define PIN_LED_YELLOW       12    // Yellow status LED (warning state)
#define PIN_LED_RED          17    // Red alert LED (critical condition)
#define PIN_BUZZER           23    // Passive piezo buzzer (PWM)
#define PIN_BUTTON           15    // Push button (active LOW, internal pull-up)

// ============================================================
//  OLED DISPLAY CONFIGURATION
// ============================================================
#define SCREEN_WIDTH         128   // OLED width in pixels
#define SCREEN_HEIGHT        64    // OLED height in pixels
#define OLED_RESET           -1    // Reset (-1 = shared with Arduino reset)
#define OLED_ADDRESS         0x3C  // I2C address for SSD1306

// ============================================================
//  DHT11 SENSOR CONFIGURATION
// ============================================================
#define DHT_TYPE             DHT22

// ============================================================
//  WI-FI CONFIGURATION (for cloud transmission - Section B)
// ============================================================
// Replace with actual credentials before deploying
#define WIFI_SSID            "YourNetworkSSID"
#define WIFI_PASSWORD        "YourNetworkPassword"
#define SERVER_URL           "http://your-server.com/api/patient-data"

// ============================================================
//  SYSTEM THRESHOLDS AND CONSTANTS
// ============================================================

// --- Room Environmental Thresholds ---
#define TEMP_HIGH_THRESHOLD  26.0  // Degrees C - activate HVAC
#define TEMP_LOW_THRESHOLD   22.0  // Degrees C - deactivate HVAC
#define TEMP_ALERT_FEVER     37.5  // Degrees C - patient fever alert
#define TEMP_SAMPLE_COUNT    5     // Samples per 10-second window
#define HUMIDITY_LOW         30.0  // % RH - dry air alert
#define HUMIDITY_HIGH        70.0  // % RH - humid air alert

// --- Light Level Thresholds ---
#define LIGHT_ON_THRESHOLD   20    // % - auto-light ON below this
#define LIGHT_OFF_THRESHOLD  30    // % - auto-light OFF above this

// --- Patient Physiological Thresholds ---
// Potentiometer 0-4095 mapped to 0-200 BPM for simulation
#define HR_NORMAL_MIN        60    // BPM - normal heart rate minimum
#define HR_NORMAL_MAX        100   // BPM - normal heart rate maximum
#define HR_ALERT_LOW         50    // BPM - bradycardia alert
#define HR_ALERT_HIGH        120   // BPM - tachycardia alert
#define SPO2_ALERT_LOW       94    // % - low blood oxygen alert

// --- Timing ---
#define SLEEP_TIMEOUT_MS     30000 // 30 seconds inactivity before sleep
#define MOTION_ALERT_MS      10000 // 10 seconds RGB alert duration
#define WIFI_SEND_INTERVAL   30000 // 30 seconds between cloud uploads

// --- Security ---
#define PASSWORD_CORRECT     "nurse123"
#define MAX_FAILED_ATTEMPTS  3     // Lock after this many wrong tries

// --- Buzzer Tones ---
#define TONE_WARNING_HZ      500   // Hz - warning tone
#define TONE_CRITICAL_HZ     2000  // Hz - critical alert tone
#define TONE_OK_HZ           1000  // Hz - all-clear confirmation tone

// ============================================================
//  OBJECT INSTANTIATION
// ============================================================
DHT dht(PIN_DHT22, DHT_TYPE);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ============================================================
//  GLOBAL VARIABLES
// ============================================================

// --- CW2: Room Temperature Statistics ---
float g_tempSamples[TEMP_SAMPLE_COUNT]; // Circular buffer of temperature readings
int   g_tempIndex     = 0;     // Current index in circular buffer
float g_tempMin       = 999.0; // Minimum temperature recorded this session
float g_tempMax       = -999.0;// Maximum temperature recorded this session
float g_tempAvg       = 0.0;   // Rolling average temperature
float g_tempStdDev    = 0.0;   // Standard deviation of temperature readings
float g_humidity      = 0.0;   // Current humidity reading (%)
unsigned long g_lastTempRead = 0; // Timestamp of last DHT11 read

// --- CW1: Automatic Lighting ---
float g_lightPercent  = 0.0;   // Current ambient light level (0-100%)
bool  g_roomLampOn    = false;  // Current auto-lamp state

// --- CW3: Brightness Dashboard ---
int   g_potBright     = 0;     // Brightness potentiometer ADC value
int   g_brightnessVal = 0;     // Mapped PWM brightness (0-255)

// --- Patient Physiological (Section B) ---
int   g_patientRawADC = 0;     // ADC reading from patient sensor potentiometer
float g_heartRate     = 0.0;   // Simulated heart rate (BPM)
bool  g_patientAlert  = false;  // Is a patient condition alert active

// --- CW6: Intrusion Detection ---
volatile bool g_motionFlag = false;    // Set by PIR ISR
bool  g_motionAlertActive  = false;    // Is motion alert running
unsigned long g_motionStart = 0;       // Motion alert start timestamp

// --- CW4: Password Security ---
bool  g_isAuthenticated    = false;    // Is staff logged in
int   g_failedAttempts     = 0;        // Failed login counter
bool  g_accessLocked       = false;    // Locked after max failed attempts

// --- CW5: Power Saving ---
unsigned long g_lastActivity = 0;      // Last activity timestamp

// --- Wi-Fi / Cloud (Section B) ---
bool  g_wifiConnected      = false;    // Wi-Fi connection state
unsigned long g_lastWifiSend = 0;      // Timestamp of last cloud upload

// --- System State ---
bool  g_monitoringActive   = true;     // Is monitoring running
String g_systemStatus      = "ACTIVE"; // Status string for display

// ============================================================
//  FUNCTION DECLARATIONS (Prototypes)
// ============================================================
void     setupPins();
void     setupOLED();
void     setupDHT();
void     setupWiFi();
void     printWelcomeBanner();
void     printMenu();

// Classwork tasks
void     cw1_AutomaticLighting();
void     cw2_TemperatureProcessing();
float    calculateStdDev(float* samples, int count, float avg);
void     cw3_BrightnessDashboard();
void     cw4_PasswordSecurity();
uint32_t djb2Hash(const String& str);
void     cw5_PowerSaving();
void     enterDeepSleep();
void     IRAM_ATTR isr_PIRMotion();
void     cw6_IntrusionDetection();
void     flashRGBRandom();

// Section B: Patient physiological
void     sectionB_PatientMonitoring();
void     triggerBuzzerAlert(int frequency, int duration);
void     checkPatientThresholds();

// Wi-Fi / cloud
void     sendDataToCloud();

// Output
void     updateOLED();
void     printSerialReadings();
void     handleSerialInput();
void     setStatusLEDs();

// ============================================================
//  SETUP - Executes once on boot or wake from deep sleep
// ============================================================
/**
 * @brief Arduino setup function. Initialises all hardware,
 *        sensors, display, Wi-Fi, and serial communication.
 *        Called once when the ESP32 starts or wakes from deep sleep.
 */
void setup() {
  Serial.begin(115200);
  delay(500);

  setupPins();    // Configure all GPIO pins and initial states
  setupOLED();    // Initialise SSD1306 display over I2C
  setupDHT();     // Initialise DHT11 sensor

  // Initialise temperature circular buffer
  for (int i = 0; i < TEMP_SAMPLE_COUNT; i++) {
    g_tempSamples[i] = 0.0;
  }

  // Attach PIR interrupt on rising edge (CW6)
  attachInterrupt(digitalPinToInterrupt(PIN_PIR),
                  isr_PIRMotion,
                  RISING);

  // Attempt Wi-Fi connection for cloud transmission (Section B)
  // Non-blocking: system continues if Wi-Fi unavailable
  setupWiFi();

  // Record startup timestamp
  g_lastActivity  = millis();
  g_lastTempRead  = millis();
  g_lastWifiSend  = millis();

  printWelcomeBanner();
  printMenu();

  // Show startup screen on OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Patient Room Monitor");
  display.println("Healthcare IoT System");
  display.println("Initialising...");
  display.display();
  delay(2000);
}

// ============================================================
//  MAIN LOOP - Executes repeatedly
// ============================================================

void loop() {
  // Process any incoming Serial Monitor commands
  handleSerialInput();

  if (g_monitoringActive) {
    cw1_AutomaticLighting();      // CW1: LDR-based auto lighting
    cw2_TemperatureProcessing();  // CW2: DHT11 stats + HVAC control
    cw3_BrightnessDashboard();    // CW3: Potentiometer brightness
    cw6_IntrusionDetection();     // CW6: PIR motion alert handler
    sectionB_PatientMonitoring(); // Section B: Patient vitals + alerts

    // Update OLED and serial output every 1 second
    static unsigned long lastDisplay = 0;
    if (millis() - lastDisplay >= 1000) {
      lastDisplay = millis();
      setStatusLEDs();
      updateOLED();
      printSerialReadings();
    }

    // Send data to cloud every 30 seconds (Section B - Wi-Fi)
    if (g_wifiConnected &&
        millis() - g_lastWifiSend >= WIFI_SEND_INTERVAL) {
      sendDataToCloud();
      g_lastWifiSend = millis();
    }
  }

  // CW5: Check inactivity timeout -> deep sleep
  cw5_PowerSaving();

  delay(10); // Prevent watchdog reset
}

// ============================================================
//  INITIALISATION FUNCTIONS
// ============================================================

void setupPins() {
  // Digital outputs
  pinMode(PIN_ROOM_LED,       OUTPUT);
  pinMode(PIN_BRIGHTNESS_LED, OUTPUT);
  pinMode(PIN_RGB_RED,        OUTPUT);
  pinMode(PIN_RGB_GREEN,      OUTPUT);
  pinMode(PIN_RGB_BLUE,       OUTPUT);
  pinMode(PIN_LED_GREEN,      OUTPUT);
  pinMode(PIN_LED_YELLOW,     OUTPUT);
  pinMode(PIN_LED_RED,        OUTPUT);
  pinMode(PIN_BUZZER,         OUTPUT);

  // Digital inputs
  pinMode(PIN_PIR,    INPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP); // Active LOW

  // Initialise all outputs to OFF/LOW
  digitalWrite(PIN_ROOM_LED,   LOW);
  digitalWrite(PIN_RGB_RED,    LOW);
  digitalWrite(PIN_RGB_GREEN,  LOW);
  digitalWrite(PIN_RGB_BLUE,   LOW);
  digitalWrite(PIN_LED_GREEN,  LOW);
  digitalWrite(PIN_LED_YELLOW, LOW);
  digitalWrite(PIN_LED_RED,    LOW);
  digitalWrite(PIN_BUZZER,     LOW);

  Serial.println("[SETUP] GPIO pins configured.");
}

void setupOLED() {
  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("[ERROR] OLED not found. Check I2C wiring.");
  } else {
    display.clearDisplay();
    display.display();
    Serial.println("[SETUP] OLED display initialised.");
  }
}


void setupDHT() {
  dht.begin();
  Serial.println("[SETUP] DHT22 sensor initialised.");
}


void setupWiFi() {
  Serial.print("[WIFI] Connecting to: ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Wait up to 10 seconds for connection
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    g_wifiConnected = true;
    Serial.println();
    Serial.print("[WIFI] Connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    g_wifiConnected = false;
    Serial.println();
    Serial.println("[WIFI] Connection failed. Offline mode active.");
    Serial.println("[WIFI] Local monitoring continues without cloud upload.");
  }
}


void printWelcomeBanner() {
  Serial.println();
  Serial.println("============================================");
  Serial.println("  IoT PATIENT ROOM MONITORING SYSTEM");
  Serial.println("  Integrated Healthcare Safety Station");
  Serial.println("  Computing Things 5FTC2094 - Project 2");
  Serial.println("============================================");
}

void printMenu() {
  Serial.println("-------- STAFF COMMAND MENU --------");
  Serial.println("  1 - Start monitoring");
  Serial.println("  2 - Stop monitoring");
  Serial.println("  3 - View current readings");
  Serial.println("  4 - View patient vitals");
  Serial.println("  5 - View temperature statistics");
  Serial.println("  6 - View light level");
  Serial.println("  7 - Show this menu");
  Serial.println("  L - Staff login (password)");
  Serial.println("  S - Enter deep sleep");
  Serial.println("  W - Send data to cloud now");
  Serial.println("------------------------------------");
}

// ============================================================
//  CW1: AUTOMATIC LIGHTING
// ============================================================


void cw1_AutomaticLighting() {
  // Read LDR ADC value (0 = dark, 4095 = bright)
  int ldrRaw = analogRead(PIN_LDR);

  // Convert to light percentage: higher ADC = more light
  g_lightPercent = (ldrRaw / 4095.0) * 100.0;

  // Hysteresis control: ON below 20%, OFF above 30%
  if (!g_roomLampOn && g_lightPercent < LIGHT_ON_THRESHOLD) {
    g_roomLampOn = true;
    digitalWrite(PIN_ROOM_LED, HIGH);
    Serial.println("[LIGHT] Room dark - Auto-lamp ON. (Fall risk reduced)");
    g_lastActivity = millis();
  }
  else if (g_roomLampOn && g_lightPercent > LIGHT_OFF_THRESHOLD) {
    g_roomLampOn = false;
    digitalWrite(PIN_ROOM_LED, LOW);
    Serial.println("[LIGHT] Sufficient daylight - Auto-lamp OFF.");
    g_lastActivity = millis();
  }
}

// ============================================================
//  CW2: REAL-TIME DATA PROCESSING
// ============================================================

void cw2_TemperatureProcessing() {
  // Enforce 2-second minimum between DHT11 reads
  if (millis() - g_lastTempRead < 2000) return;
  g_lastTempRead = millis();

  // Attempt to read from DHT11
  float temperature = dht.readTemperature();
  float humidity    = dht.readHumidity();

  // --- Fault Tolerance: Sensor failure detection ---
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("[TEMP] WARNING: DHT22 read failed!");
    Serial.println("[TEMP] Fault tolerance: using last valid reading.");
    // Safe fallback: continue with previous values (no crash)
    return;
  }

  // Store humidity globally for display
  g_humidity = humidity;

  // Store temperature in circular buffer
  g_tempSamples[g_tempIndex] = temperature;
  g_tempIndex = (g_tempIndex + 1) % TEMP_SAMPLE_COUNT;

  // Update running minimum and maximum
  if (temperature < g_tempMin) g_tempMin = temperature;
  if (temperature > g_tempMax) g_tempMax = temperature;

  // Calculate rolling average
  float sum = 0.0;
  for (int i = 0; i < TEMP_SAMPLE_COUNT; i++) {
    sum += g_tempSamples[i];
  }
  g_tempAvg = sum / TEMP_SAMPLE_COUNT;

  // Calculate standard deviation
  g_tempStdDev = calculateStdDev(g_tempSamples, TEMP_SAMPLE_COUNT, g_tempAvg);

  // --- Print statistics every 10 seconds (5 samples * 2s) ---
  static int sampleCount = 0;
  sampleCount++;
  if (sampleCount >= TEMP_SAMPLE_COUNT) {
    sampleCount = 0;
    Serial.println("------ ROOM TEMPERATURE STATISTICS (10s) ------");
    Serial.print("  Min:      "); Serial.print(g_tempMin, 1);    Serial.println(" C");
    Serial.print("  Max:      "); Serial.print(g_tempMax, 1);    Serial.println(" C");
    Serial.print("  Average:  "); Serial.print(g_tempAvg, 1);    Serial.println(" C");
    Serial.print("  StdDev:   "); Serial.print(g_tempStdDev, 2); Serial.println(" C");
    Serial.print("  Humidity: "); Serial.print(g_humidity, 1);   Serial.println(" %");
    Serial.println("------------------------------------------------");
  }

  // --- HVAC control based on rolling average ---
  static bool fanOn = false;
  if (!fanOn && g_tempAvg > TEMP_HIGH_THRESHOLD) {
    fanOn = true;
    Serial.print("[HVAC] Temp ");
    Serial.print(g_tempAvg, 1);
    Serial.println("C - Fan/AC activated for patient comfort.");
    triggerBuzzerAlert(TONE_WARNING_HZ, 200); // Short tone for staff awareness
  }
  else if (fanOn && g_tempAvg < TEMP_LOW_THRESHOLD) {
    fanOn = false;
    Serial.println("[HVAC] Temperature normalised - Fan/AC deactivated.");
  }

  // --- Humidity alerts ---
  if (g_humidity < HUMIDITY_LOW) {
    Serial.println("[AIR] WARNING: Humidity low - risk of respiratory discomfort.");
  }
  else if (g_humidity > HUMIDITY_HIGH) {
    Serial.println("[AIR] WARNING: Humidity high - discomfort risk for patients.");
  }
}

float calculateStdDev(float* samples, int count, float avg) {
  float sumSq = 0.0;
  for (int i = 0; i < count; i++) {
    float diff = samples[i] - avg;
    sumSq += diff * diff;
  }
  return sqrt(sumSq / count);
}

// ============================================================
//  CW3: CONTROLS AND DASHBOARD
// ============================================================

void cw3_BrightnessDashboard() {
  // Read brightness potentiometer (0-4095)
  g_potBright = analogRead(PIN_POT_BRIGHTNESS);

  // Map ADC range to PWM duty cycle (0-255)
  g_brightnessVal = map(g_potBright, 0, 4095, 0, 255);

  // Apply PWM to the brightness LED
  analogWrite(PIN_BRIGHTNESS_LED, g_brightnessVal);

  // Calculate duty cycle percentage
  float dutyCyclePct = (g_brightnessVal / 255.0) * 100.0;

  // --- Serial Plotter output (comma-separated for graph display) ---
  // Open Serial Plotter (Tools -> Serial Plotter) to see live graph
  Serial.print("LightADC:");
  Serial.print(g_potBright);
  Serial.print(",DutyCycle:");
  Serial.println(dutyCyclePct, 1);
}

// ============================================================
//  CW4: PASSWORD SECURITY
// ============================================================

void cw4_PasswordSecurity() {
  Serial.println();

  // Check if access is locked from failed attempts
  if (g_accessLocked) {
    Serial.println("[AUTH] SYSTEM LOCKED: Too many failed attempts.");
    Serial.println("[AUTH] Contact system administrator to reset.");
    triggerBuzzerAlert(TONE_CRITICAL_HZ, 500);
    return;
  }

  Serial.println("[AUTH] Staff authentication required.");
  Serial.println("[AUTH] Enter password:");

  // Wait up to 30 seconds for password input
  String password = "";
  unsigned long startWait = millis();
  while (millis() - startWait < 30000) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') break;
      password += c;
    }
    delay(10);
  }
  password.trim();

  // Compute djb2 hash of entered and correct passwords
  uint32_t inputHash   = djb2Hash(password);
  uint32_t correctHash = djb2Hash(String(PASSWORD_CORRECT));

  // Display hash values for demonstration
  Serial.print("[AUTH] Hash of your input:   0x");
  Serial.println(inputHash, HEX);
  Serial.print("[AUTH] Expected hash:         0x");
  Serial.println(correctHash, HEX);

  if (inputHash == correctHash) {
    // Correct password
    g_isAuthenticated = true;
    g_failedAttempts  = 0;
    Serial.println("[AUTH] Access GRANTED. Welcome, authorised staff.");
    triggerBuzzerAlert(TONE_OK_HZ, 100); // Short confirmation beep
    g_lastActivity = millis();
  }
  else {
    // Wrong password
    g_isAuthenticated = false;
    g_failedAttempts++;
    Serial.print("[AUTH] Access DENIED. Failed attempt ");
    Serial.print(g_failedAttempts);
    Serial.print(" of ");
    Serial.println(MAX_FAILED_ATTEMPTS);

    // Flash red LED for failed authentication
    for (int i = 0; i < 3; i++) {
      digitalWrite(PIN_LED_RED, HIGH); delay(200);
      digitalWrite(PIN_LED_RED, LOW);  delay(200);
    }
    triggerBuzzerAlert(TONE_CRITICAL_HZ, 300);

    // Lock after maximum failed attempts
    if (g_failedAttempts >= MAX_FAILED_ATTEMPTS) {
      g_accessLocked = true;
      Serial.println("[AUTH] LOCKED: Maximum failed attempts reached!");
      Serial.println("[AUTH] Unauthorised access logged.");
    }
  }
}

uint32_t djb2Hash(const String& str) {
  uint32_t hash = 5381; // djb2 magic seed value
  for (int i = 0; i < str.length(); i++) {
    hash = ((hash << 5) + hash) + (uint8_t)str[i]; // hash * 33 + c
  }
  return hash;
}

// ============================================================
//  CW5: POWER SAVING (DEEP SLEEP)
// ============================================================

void cw5_PowerSaving() {
  static unsigned long btnHoldStart  = 0;
  static bool          btnWasPressed = false;

  // Check button state (active LOW)
  if (digitalRead(PIN_BUTTON) == LOW) {
    g_lastActivity = millis(); // Button press = activity
    if (!btnWasPressed) {
      btnWasPressed = true;
      btnHoldStart  = millis();
    }

    // 3-second hold = maintenance/emergency override mode
    if (millis() - btnHoldStart >= 3000) {
      Serial.println("[POWER] Button held 3s - MAINTENANCE MODE active.");
      Serial.println("[POWER] All monitoring paused. Release to resume.");
      g_monitoringActive = false;
      while (digitalRead(PIN_BUTTON) == LOW) { delay(100); }
      g_monitoringActive = true;
      g_systemStatus = "ACTIVE";
      Serial.println("[POWER] Maintenance mode ended. Monitoring resumed.");
      btnWasPressed  = false;
      g_lastActivity = millis();
    }
  }
  else {
    btnWasPressed = false;
    btnHoldStart  = 0;
  }

  // Check inactivity timeout
  if (millis() - g_lastActivity > SLEEP_TIMEOUT_MS) {
    enterDeepSleep();
  }
}

void enterDeepSleep() {
  Serial.println();
  Serial.println("[SLEEP] System inactive for 30 seconds.");
  Serial.println("[SLEEP] Entering deep sleep to conserve battery.");
  Serial.println("[SLEEP] Touch T3 (GPIO15) to wake - simulates nurse panel touch.");

  // Display sleep screen on OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 16);
  display.println("  POWER SAVING MODE");
  display.println("  Touch panel to wake");
  display.println("  (Nurse touch sensor)");
  display.display();
  delay(1500);

  // Power down all outputs
  digitalWrite(PIN_LED_GREEN,  LOW);
  digitalWrite(PIN_LED_YELLOW, LOW);
  digitalWrite(PIN_LED_RED,    LOW);
  digitalWrite(PIN_ROOM_LED,   LOW);
  display.clearDisplay();
  display.display();

  // Configure touch pin T3 as wake-up source
  // System wakes when capacitive touch value drops below threshold 40
  touchAttachInterrupt(T3, [](){}, 40);
  esp_sleep_enable_touchpad_wakeup();

  // Enter deep sleep - halts here until touch wake-up
  esp_deep_sleep_start();
}

// ============================================================
//  CW6: INTRUSION DETECTION (INTERRUPT-DRIVEN)
// ============================================================

void IRAM_ATTR isr_PIRMotion() {
  g_motionFlag = true; // Set flag for main loop to handle
}


void cw6_IntrusionDetection() {
  // Check if ISR set the motion flag
  if (g_motionFlag) {
    g_motionFlag = false; // Clear flag immediately
    if (!g_motionAlertActive) {
      g_motionAlertActive = true;
      g_motionStart       = millis();

      // Staff notification via Serial Monitor
      Serial.println();
      Serial.println("!!! SECURITY ALERT !!!");
      Serial.println("[PIR] ALERT: Motion detected in restricted area!");
      Serial.println("[PIR] Possible unauthorized entry or patient fall risk.");
      Serial.println("[PIR] RGB alert active for 10 seconds.");

      // Audible alert for nursing staff (Section B: alarm mechanism)
      triggerBuzzerAlert(TONE_CRITICAL_HZ, 500);

      g_lastActivity = millis(); // Motion = system activity
    }
  }

  // Handle active motion alert - flash RGB random colors
  if (g_motionAlertActive) {
    if (millis() - g_motionStart < MOTION_ALERT_MS) {
      flashRGBRandom();
      digitalWrite(PIN_LED_RED, HIGH);
    }
    else {
      // Alert period ended
      g_motionAlertActive = false;
      digitalWrite(PIN_RGB_RED,   LOW);
      digitalWrite(PIN_RGB_GREEN, LOW);
      digitalWrite(PIN_RGB_BLUE,  LOW);
      digitalWrite(PIN_LED_RED,   LOW);
      Serial.println("[PIR] Motion alert period ended.");
    }
  }
}


void flashRGBRandom() {
  static unsigned long lastFlash = 0;
  if (millis() - lastFlash < 500) return; // Limit to every 500ms
  lastFlash = millis();

  // Generate random values for each RGB channel (0-255)
  analogWrite(PIN_RGB_RED,   random(0, 256));
  analogWrite(PIN_RGB_GREEN, random(0, 256));
  analogWrite(PIN_RGB_BLUE,  random(0, 256));
}

// ============================================================
//  SECTION B: PATIENT PHYSIOLOGICAL MONITORING
// ============================================================

void sectionB_PatientMonitoring() {
  static unsigned long lastPatientRead = 0;

  // Read patient sensor every 2 seconds
  if (millis() - lastPatientRead < 2000) return;
  lastPatientRead = millis();

  // Read simulated physiological sensor (potentiometer)
  // GPIO34 is input-only (no internal pull-up) - correct for ADC use
  g_patientRawADC = analogRead(PIN_POT_PATIENT);

  // --- Fault Tolerance: Detect disconnected/stuck sensor ---
  // A stuck-at-max or stuck-at-zero reading indicates sensor failure
  if (g_patientRawADC == 0 || g_patientRawADC >= 4094) {
    Serial.println("[PATIENT] WARNING: Patient sensor may be disconnected!");
    Serial.println("[PATIENT] Fault tolerance: Alert staff to check sensor.");
    triggerBuzzerAlert(TONE_WARNING_HZ, 300);
    // Safe fallback: log warning and continue without crashing
    return;
  }

  // Map ADC reading (0-4095) to simulated heart rate range (40-200 BPM)
  // In real deployment: replace with actual MAX30102 BPM calculation
  g_heartRate = map(g_patientRawADC, 0, 4095, 40, 200);

  // Check patient vital thresholds and trigger alerts if needed
  checkPatientThresholds();

  // Log patient data to Serial Monitor (staff can check trends)
  static int logCount = 0;
  logCount++;
  if (logCount >= 5) { // Log every 10 seconds
    logCount = 0;
    Serial.println("------ PATIENT VITALS LOG ------");
    Serial.print("  Heart Rate (sim): ");
    Serial.print(g_heartRate, 0);
    Serial.println(" BPM");
    Serial.print("  Room Temp (body proxy): ");
    Serial.print(g_tempAvg, 1);
    Serial.println(" C");
    Serial.print("  Patient ADC Raw:  ");
    Serial.println(g_patientRawADC);
    Serial.println("--------------------------------");
  }
}

void checkPatientThresholds() {
  bool alertTriggered = false;

  // --- Heart Rate: Bradycardia (too slow) ---
  if (g_heartRate < HR_ALERT_LOW) {
    Serial.println("!!! PATIENT ALERT: BRADYCARDIA !!!");
    Serial.print("[PATIENT] Heart rate critically LOW: ");
    Serial.print(g_heartRate, 0);
    Serial.println(" BPM (< 50 BPM). Nurse response required!");
    triggerBuzzerAlert(TONE_CRITICAL_HZ, 1000);
    digitalWrite(PIN_LED_RED, HIGH);
    g_patientAlert = true;
    alertTriggered = true;
    g_lastActivity = millis();
  }

  // --- Heart Rate: Tachycardia (too fast) ---
  else if (g_heartRate > HR_ALERT_HIGH) {
    Serial.println("!!! PATIENT ALERT: TACHYCARDIA !!!");
    Serial.print("[PATIENT] Heart rate critically HIGH: ");
    Serial.print(g_heartRate, 0);
    Serial.println(" BPM (> 120 BPM). Immediate attention required!");
    triggerBuzzerAlert(TONE_CRITICAL_HZ, 1000);
    digitalWrite(PIN_LED_RED, HIGH);
    g_patientAlert = true;
    alertTriggered = true;
    g_lastActivity = millis();
  }

  // --- Room Temperature: Possible Fever proxy ---
  else if (g_tempAvg > TEMP_ALERT_FEVER) {
    Serial.println("!!! PATIENT ALERT: HIGH TEMPERATURE !!!");
    Serial.print("[PATIENT] Possible fever indicated: ");
    Serial.print(g_tempAvg, 1);
    Serial.println(" C. Check patient body temperature.");
    triggerBuzzerAlert(TONE_WARNING_HZ, 500);
    digitalWrite(PIN_LED_YELLOW, HIGH);
    alertTriggered = true;
  }

  // --- Normal vitals ---
  if (!alertTriggered) {
    g_patientAlert = false;
    digitalWrite(PIN_LED_RED,    LOW);
    digitalWrite(PIN_LED_YELLOW, LOW);
  }
}

// ============================================================
//  WI-FI CLOUD TRANSMISSION (SECTION B)
// ============================================================

void sendDataToCloud() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WIFI] Not connected - skipping cloud upload.");
    g_wifiConnected = false;
    return;
  }

  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type", "application/json");

  // Build JSON payload with all sensor readings
  String jsonPayload = "{";
  jsonPayload += "\"room_temp\":"    + String(g_tempAvg, 1)      + ",";
  jsonPayload += "\"room_humidity\":" + String(g_humidity, 1)    + ",";
  jsonPayload += "\"light_pct\":"    + String(g_lightPercent, 1) + ",";
  jsonPayload += "\"heart_rate\":"   + String(g_heartRate, 0)    + ",";
  jsonPayload += "\"motion_alert\":" + String(g_motionAlertActive ? "true" : "false") + ",";
  jsonPayload += "\"patient_alert\":" + String(g_patientAlert ? "true" : "false")  + ",";
  jsonPayload += "\"brightness_pct\":" + String((g_brightnessVal/255.0)*100.0, 0);
  jsonPayload += "}";

  // Send HTTP POST
  int httpCode = http.POST(jsonPayload);

  if (httpCode > 0) {
    Serial.print("[WIFI] Cloud upload successful. HTTP code: ");
    Serial.println(httpCode);
  } else {
    Serial.print("[WIFI] Cloud upload failed. Error: ");
    Serial.println(http.errorToString(httpCode));
  }

  http.end();
}

// ============================================================
//  BUZZER ALERT (SECTION B: ALARM MECHANISM)
// ============================================================

void triggerBuzzerAlert(int frequency, int duration) {
  // Use tone() to drive passive buzzer with PWM signal
  tone(PIN_BUZZER, frequency, duration);
  delay(duration + 50); // Wait for tone to complete
  noTone(PIN_BUZZER);
}

// ============================================================
//  OUTPUT FUNCTIONS
// ============================================================

void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Line 1: Room temperature and humidity
  display.setCursor(0, 0);
  display.print("T:");
  display.print(g_tempAvg, 1);
  display.print("C H:");
  display.print(g_humidity, 0);
  display.print("%");

  // Line 2: Light level and lamp state
  display.setCursor(0, 10);
  display.print("Light:");
  display.print(g_lightPercent, 0);
  display.print("% Lamp:");
  display.print(g_roomLampOn ? "ON" : "OFF");

  // Line 3: Simulated patient heart rate
  display.setCursor(0, 20);
  display.print("HR:");
  display.print(g_heartRate, 0);
  display.print("BPM");
  if (g_patientAlert) {
    display.print(" !ALERT!");
  }

  // Line 4: Brightness level
  display.setCursor(0, 30);
  display.print("Brightness:");
  display.print((g_brightnessVal / 255.0) * 100.0, 0);
  display.print("%");

  // Line 5: Motion / security status
  display.setCursor(0, 42);
  if (g_motionAlertActive) {
    display.print("!!! MOTION ALERT !!!");
  } else {
    display.print("Motion: Clear");
  }

  // Line 6: Overall system status
  display.setCursor(0, 54);
  display.print("Status:");
  display.print(g_monitoringActive ? "ACTIVE" : "STOPPED");
  if (g_wifiConnected) display.print(" WiFi");

  display.display(); // Push buffer to OLED
}

void printSerialReadings() {
  static unsigned long lastFull = 0;
  if (millis() - lastFull < 5000) return; // Print every 5 seconds
  lastFull = millis();

  Serial.println();
  Serial.println("====== PATIENT ROOM MONITORING REPORT ======");
  Serial.println("--- ROOM ENVIRONMENTAL CONDITIONS ---");
  Serial.print("  Temperature:  "); Serial.print(g_tempAvg, 1);      Serial.println(" C (avg)");
  Serial.print("  Humidity:     "); Serial.print(g_humidity, 1);     Serial.println(" %");
  Serial.print("  Light Level:  "); Serial.print(g_lightPercent, 1); Serial.println(" %");
  Serial.print("  Room Lamp:    "); Serial.println(g_roomLampOn ? "ON (auto)" : "OFF");
  Serial.print("  Brightness:   "); Serial.print((g_brightnessVal/255.0)*100.0, 0); Serial.println(" %");
  Serial.println("--- PATIENT PHYSIOLOGICAL ---");
  Serial.print("  Heart Rate:   "); Serial.print(g_heartRate, 0);    Serial.println(" BPM (simulated)");
  Serial.print("  Patient Alert:"); Serial.println(g_patientAlert ? " ACTIVE" : " None");
  Serial.println("--- SECURITY STATUS ---");
  Serial.print("  Motion Alert: "); Serial.println(g_motionAlertActive ? "ACTIVE" : "Clear");
  Serial.print("  Auth Status:  "); Serial.println(g_isAuthenticated ? "Staff logged in" : "Not authenticated");
  Serial.print("  Lock Status:  "); Serial.println(g_accessLocked ? "LOCKED" : "Unlocked");
  Serial.println("--- SYSTEM ---");
  Serial.print("  Monitoring:   "); Serial.println(g_monitoringActive ? "ACTIVE" : "STOPPED");
  Serial.print("  Wi-Fi:        "); Serial.println(g_wifiConnected ? "Connected" : "Offline");
  Serial.println("=============================================");
}

void setStatusLEDs() {
  bool critical = g_patientAlert || g_accessLocked;
  bool warning  = (g_tempAvg > TEMP_HIGH_THRESHOLD) ||
                  (g_humidity < HUMIDITY_LOW) ||
                  (g_humidity > HUMIDITY_HIGH);

  // Green: active and no alerts
  digitalWrite(PIN_LED_GREEN, (g_monitoringActive && !critical && !warning) ? HIGH : LOW);

  // Yellow: warning conditions (not overriding motion alert red)
  if (!g_motionAlertActive) {
    digitalWrite(PIN_LED_YELLOW, warning ? HIGH : LOW);
  }

  // Red: critical conditions (not overriding motion alert)
  if (!g_motionAlertActive) {
    digitalWrite(PIN_LED_RED, critical ? HIGH : LOW);
  }
}

// ============================================================
//  SERIAL INPUT HANDLER
// ============================================================

void handleSerialInput() {
  if (!Serial.available()) return;

  char cmd = Serial.read();
  g_lastActivity = millis(); // Any input resets inactivity timer

  switch (cmd) {
    case '1':
      g_monitoringActive = true;
      g_systemStatus = "ACTIVE";
      Serial.println("[CMD] Monitoring STARTED.");
      break;

    case '2':
      g_monitoringActive = false;
      g_systemStatus = "STOPPED";
      Serial.println("[CMD] Monitoring STOPPED.");
      break;

    case '3':
      // Immediate full readings dump
      Serial.println();
      Serial.println("====== CURRENT READINGS ======");
      Serial.print("Room Temp:    "); Serial.print(g_tempAvg, 1);      Serial.println(" C");
      Serial.print("Humidity:     "); Serial.print(g_humidity, 1);     Serial.println(" %");
      Serial.print("Light Level:  "); Serial.print(g_lightPercent, 1); Serial.println(" %");
      Serial.print("Heart Rate:   "); Serial.print(g_heartRate, 0);    Serial.println(" BPM");
      Serial.print("Brightness:   "); Serial.print((g_brightnessVal/255.0)*100.0, 0); Serial.println(" %");
      Serial.println("==============================");
      break;

    case '4':
      Serial.println("[CMD] Patient Vitals:");
      Serial.print("  Simulated HR: "); Serial.print(g_heartRate, 0); Serial.println(" BPM");
      Serial.print("  Alert Active: "); Serial.println(g_patientAlert ? "YES" : "No");
      Serial.print("  Room Temp:    "); Serial.print(g_tempAvg, 1); Serial.println(" C");
      break;

    case '5':
      Serial.println("[CMD] Temperature Statistics:");
      Serial.print("  Min:    "); Serial.print(g_tempMin, 1);    Serial.println(" C");
      Serial.print("  Max:    "); Serial.print(g_tempMax, 1);    Serial.println(" C");
      Serial.print("  Avg:    "); Serial.print(g_tempAvg, 1);    Serial.println(" C");
      Serial.print("  StdDev: "); Serial.print(g_tempStdDev, 2); Serial.println(" C");
      Serial.print("  Humidity: "); Serial.print(g_humidity, 1); Serial.println(" %");
      break;

    case '6':
      Serial.print("[CMD] Light Level: ");
      Serial.print(g_lightPercent, 1);
      Serial.println(" %");
      Serial.print("[CMD] Room Lamp:   ");
      Serial.println(g_roomLampOn ? "ON (auto-activated)" : "OFF");
      break;

    case '7':
      printMenu();
      break;

    case 'L':
    case 'l':
      cw4_PasswordSecurity(); // Trigger password authentication
      break;

    case 'S':
    case 's':
      Serial.println("[CMD] Manual sleep command received.");
      enterDeepSleep();
      break;

    case 'W':
    case 'w':
      Serial.println("[CMD] Manual cloud upload requested.");
      sendDataToCloud();
      break;

    default:
      if (cmd != '\n' && cmd != '\r') {
        Serial.print("[CMD] Unknown command: '");
        Serial.print(cmd);
        Serial.println("'. Enter 7 for menu.");
      }
      break;
  }
}