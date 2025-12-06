/**
 * SISTEM KENDALI HYBRID (FUZZY & PID) 
 * * Deskripsi:
 * Kode ini membandingkan kinerja kontrol Fuzzy Logic vs PID Adaptif (Gain Scheduling).
 * - Fuzzy: Menggunakan metode Sugeno (5 membership function).
 * - PID: Menggunakan fitur Gain Scheduling (respon cepat) + Feedforward (anti-stuck).
 * * Hardware: ESP32, DS18B20, Sensor Turbidity (ADS1115), L298N Driver.
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <esp_arduino_version.h>

// =========================================================================
//                  SETTING JARINGAN & MQTT
// =========================================================================

struct WiFiCredentials {
  const char *ssid;
  const char *password;
};

WiFiCredentials wifiNetworks[] = {
    {"Private u52", "12345678"}, 
    {"iPhone 2", "bobo2002"}    
};
const int NUM_WIFI_NETWORKS = sizeof(wifiNetworks) / sizeof(wifiNetworks[0]);

const char *MQTT_BROKER = "broker.hivemq.com";
const int MQTT_PORT = 1883;
const char *MQTT_TOPIC_DATA = "unhas/informatika/aquarium/data";
const char *MQTT_TOPIC_MODE = "unhas/informatika/aquarium/mode";
const char *MQTT_CLIENT_ID = "esp32-research-aquarium";

// =========================================================================
//                  PIN & VARIABEL GLOBAL
// =========================================================================

const int SENSOR_SUHU_PIN = 4;

// Driver L298N (Pemanas & Pompa)
const int HEATER_ENA = 16; const int HEATER_IN1 = 17; const int HEATER_IN2 = 18;
const int PUMP_ENB = 27;   const int PUMP_IN3 = 25;   const int PUMP_IN4 = 26;

// Mode Kontrol
enum ControlMode { FUZZY, PID };
ControlMode kontrolAktif = FUZZY;

// Setpoint default
float suhuSetpoint = 28.0f;
float turbiditySetpoint = 15.0f;

// Parameter PID (Default Tuning - Mode Smooth)
double Kp_suhu = 8.0, Ki_suhu = 0.3, Kd_suhu = 6.0;
double Kp_keruh = 5.0, Ki_keruh = 0.2, Kd_keruh = 2.0; 

// Variabel penyimpan nilai integral & error sebelumnya
double integralSumSuhu = 0.0, lastErrorSuhu = 0.0;
double integralSumKeruh = 0.0, lastErrorKeruh = 0.0;

// Kalibrasi ADC Turbidity (Nilai Default)
int NILAI_ADC_JERNIH = 20100;
int NILAI_ADC_KERUH = 3550;

// Timer
unsigned long lastTimeSuhu = 0;
unsigned long lastTimeKeruh = 0;
unsigned long waktuTerakhirKirim = 0;
const long intervalKirim = 1000;      
unsigned long lastWiFiCheck = 0;

// variabel wifiCheckInterval 
const long wifiCheckInterval = 5000; 

// Setting PWM
const int PWM_FREQ = 1000;
const int PWM_RESOLUTION = 8;
const int PWM_MIN_FISIK = 180;   
const int PWM_START_LOGIKA = 5;

// Objek Sensor & Komunikasi
WiFiClient espClient;
PubSubClient mqttClient(espClient);
OneWire oneWire(SENSOR_SUHU_PIN);
DallasTemperature sensors(&oneWire);
Adafruit_ADS1115 ads;

// Variabel Filter Sensor & Last Values
float suhuTerfilter = 0.0;
const float ALPHA = 0.2; 

// variabel suhuTerakhir & turbidityTerakhir
float suhuTerakhir = 25.0f;
int turbidityTerakhir = 0; 

// =========================================================================
//                  FUNGSI BANTUAN (HELPER)
// =========================================================================

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// =========================================================================
//                  KONTROL MOTOR L298N
// =========================================================================

void setHeaterSpeed(int pwmValue) {
  pwmValue = constrain(pwmValue, 0, 255);
  if (pwmValue > 0) {
    digitalWrite(HEATER_IN1, HIGH); digitalWrite(HEATER_IN2, LOW);
  } else {
    digitalWrite(HEATER_IN1, LOW); digitalWrite(HEATER_IN2, LOW);
  }
  
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(HEATER_ENA, pwmValue);
  #else
    ledcWrite(0, pwmValue); 
  #endif
}

void setPumpSpeed(int pwmValue) {
  pwmValue = constrain(pwmValue, 0, 255);
  int finalOutput = 0;

  if (pwmValue < PWM_START_LOGIKA) {
    finalOutput = 0;
  } else {
    finalOutput = map(pwmValue, PWM_START_LOGIKA, 255, PWM_MIN_FISIK, 255);
  }

  if (finalOutput > 0) {
    digitalWrite(PUMP_IN3, HIGH); digitalWrite(PUMP_IN4, LOW);
  } else {
    digitalWrite(PUMP_IN3, LOW); digitalWrite(PUMP_IN4, LOW);
  }

  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(PUMP_ENB, finalOutput);
  #else
    ledcWrite(1, finalOutput); 
  #endif
}

void setupL298N() {
  pinMode(HEATER_IN1, OUTPUT); pinMode(HEATER_IN2, OUTPUT);
  pinMode(PUMP_IN3, OUTPUT); pinMode(PUMP_IN4, OUTPUT);

  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttach(HEATER_ENA, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PUMP_ENB, PWM_FREQ, PWM_RESOLUTION);
  #else
    ledcSetup(0, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(HEATER_ENA, 0);
    ledcSetup(1, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(PUMP_ENB, 1);
  #endif
}

// =========================================================================
//                  LOGIKA FUZZY (SUGENO)
// =========================================================================

// --- Fuzzy Suhu ---
float membershipSangatDingin(float error) {
  if (error <= 3.5f) return 0.0f;
  if (error >= 5.0f) return 1.0f;
  return (error - 3.5f) / 1.5f;
}
float membershipDingin(float error) {
  if (error <= 1.5f || error >= 4.5f) return 0.0f;
  if (error >= 2.5f && error <= 3.5f) return 1.0f;
  if (error > 1.5f && error < 2.5f) return (error - 1.5f) / 1.0f;
  return (4.5f - error) / 1.0f; 
}
float membershipSesuai(float error) { 
  if (error <= -1.0f || error >= 2.0f) return 0.0f;
  if (error >= -0.3f && error <= 0.3f) return 1.0f;
  if (error > -1.0f && error < -0.3f) return (error + 1.0f) / 0.7f;
  return (2.0f - error) / 1.7f;
}
float membershipPanas(float error) {
  if (error <= -3.5f || error >= -0.5f) return 0.0f;
  if (error >= -2.5f && error <= -1.0f) return 1.0f;
  if (error > -3.5f && error < -2.5f) return (error + 3.5f) / 1.0f;
  return (-0.5f - error) / 0.5f;
}
float membershipSangatPanas(float error) {
  if (error >= -3.0f) return 0.0f;
  if (error <= -4.5f) return 1.0f;
  return (-3.0f - error) / 1.5f;
}

float hitungFuzzySuhu(float errorSuhu) {
  float mu_sangatDingin = membershipSangatDingin(errorSuhu);
  float mu_dingin = membershipDingin(errorSuhu);
  float mu_sesuai = membershipSesuai(errorSuhu);
  float mu_panas = membershipPanas(errorSuhu);
  float mu_sangatPanas = membershipSangatPanas(errorSuhu);

  float numerator = (mu_sangatDingin * 95.0f) + (mu_dingin * 75.0f) + 
                    (mu_sesuai * 25.0f) + (mu_panas * 5.0f) + (mu_sangatPanas * 0.0f);
  float denominator = mu_sangatDingin + mu_dingin + mu_sesuai + mu_panas + mu_sangatPanas;

  if (denominator < 0.01f) return 25.0f; 
  return numerator / denominator;
}

// --- Fuzzy Turbidity ---
float membershipSangatJernih(float error) {
  if (error <= -7.0f) return 1.0f;
  if (error <= -5.0f) return (-5.0f - error) / 2.0f;
  return 0.0f;
}
float membershipJernih(float error) {
  if (error <= -7.0f || error >= -1.0f) return 0.0f;
  if (error >= -4.0f && error <= -2.0f) return 1.0f;
  if (error > -7.0f && error < -4.0f) return (error + 7.0f) / 3.0f;
  return (-1.0f - error) / 1.0f;
}
float membershipSesuaiKeruh(float error) {
  if (error <= -2.5f || error >= 2.5f) return 0.0f;
  if (error >= -0.5f && error <= 0.5f) return 1.0f;
  if (error > -2.5f && error < -0.5f) return (error + 2.5f) / 2.0f;
  return (2.5f - error) / 2.0f;
}
float membershipKeruh(float error) {
  if (error <= 1.0f || error >= 10.0f) return 0.0f;
  if (error >= 4.0f && error <= 7.0f) return 1.0f;
  if (error > 1.0f && error < 4.0f) return (error - 1.0f) / 3.0f;
  return (10.0f - error) / 3.0f;
}
float membershipSangatKeruh(float error) {
  if (error <= 8.0f) return 0.0f;
  if (error >= 12.0f) return 1.0f;
  return (error - 8.0f) / 4.0f;
}

float hitungFuzzyKeruh(float errorKeruh) {
  float mu_sangatJernih = membershipSangatJernih(errorKeruh);
  float mu_jernih = membershipJernih(errorKeruh);
  float mu_sesuai = membershipSesuaiKeruh(errorKeruh);
  float mu_keruh = membershipKeruh(errorKeruh);
  float mu_sangatKeruh = membershipSangatKeruh(errorKeruh);

  float numerator = (mu_sangatJernih * 0.0f) + (mu_jernih * 20.0f) + 
                    (mu_sesuai * 50.0f) + (mu_keruh * 90.0f) + (mu_sangatKeruh * 100.0f);
  float denominator = mu_sangatJernih + mu_jernih + mu_sesuai + mu_keruh + mu_sangatKeruh;

  if (denominator < 0.01f) return 50.0f; 
  return numerator / denominator;
}

// =========================================================================
//                      KONTROL PID (ADVANCED)
// =========================================================================

double hitungPIDSuhu(float errorSuhu) {
  unsigned long now = millis();
  double dt = (double)(now - lastTimeSuhu) / 1000.0;
  if (dt < 0.001) dt = 0.001; 

  double P = Kp_suhu * errorSuhu;

  integralSumSuhu += errorSuhu * dt;
  if (integralSumSuhu > 20.0) integralSumSuhu = 20.0;
  if (integralSumSuhu < -20.0) integralSumSuhu = -20.0;
  
  if ((errorSuhu > 0 && lastErrorSuhu < 0) || (errorSuhu < 0 && lastErrorSuhu > 0)) {
    integralSumSuhu *= 0.5;
  }
  double I = Ki_suhu * integralSumSuhu;

  double rawDerivative = (errorSuhu - lastErrorSuhu) / dt;
  static double lastDerivSuhu = 0.0;
  double derivative = 0.3 * rawDerivative + 0.7 * lastDerivSuhu; 
  lastDerivSuhu = derivative;
  double D = Kd_suhu * derivative;

  lastErrorSuhu = errorSuhu;
  lastTimeSuhu = now;

  return constrain(P + I + D, 0.0, 100.0);
}

double hitungPIDKeruh(float errorKeruh) {
  unsigned long now = millis();
  double dt = (double)(now - lastTimeKeruh) / 1000.0;
  if (dt < 0.001) dt = 0.001;

  double dynamicKp;
  double dynamicKd;
  
  if (abs(errorKeruh) > 2.0) {
    dynamicKp = 35.0; // Mode Turbo
    dynamicKd = 0.0;  
    integralSumKeruh = 0; 
  } else {
    dynamicKp = Kp_keruh; // Mode Smooth
    dynamicKd = Kd_keruh; 
  }

  double P = dynamicKp * errorKeruh;

  integralSumKeruh += errorKeruh * dt;
  integralSumKeruh = constrain(integralSumKeruh, -20.0, 20.0); 
  double I = Ki_keruh * integralSumKeruh;

  double rawDerivative = (errorKeruh - lastErrorKeruh) / dt;
  static double lastDerivKeruh = 0.0;
  double derivative = 0.3 * rawDerivative + 0.7 * lastDerivKeruh; 
  lastDerivKeruh = derivative;
  double D = dynamicKd * derivative;

  double feedForward = 50.0; 

  double output = P + I + D + feedForward;

  float aktualTurbidity = errorKeruh + turbiditySetpoint; 
  float targetMatiTotal = 9.0; 

  if (aktualTurbidity <= targetMatiTotal) {
    output = 0.0;          
    integralSumKeruh = 0;  
  }

  lastErrorKeruh = errorKeruh;
  lastTimeKeruh = now;

  return constrain(output, 0.0, 100.0);
}

void resetPID() {
  integralSumSuhu = 0; lastErrorSuhu = 0;
  integralSumKeruh = 0; lastErrorKeruh = 0;
  lastTimeSuhu = millis(); lastTimeKeruh = millis();
}

// =========================================================================
//                  KONEKSI WIFI & MQTT
// =========================================================================

void setup_wifi() {
  delay(10);
  Serial.println("\n[WiFi] Mencoba koneksi...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  for (int network = 0; network < NUM_WIFI_NETWORKS; network++) {
    Serial.printf("\n[WiFi] Mencoba network: %s\n", wifiNetworks[network].ssid);
    WiFi.begin(wifiNetworks[network].ssid, wifiNetworks[network].password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500); Serial.print("."); attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("\n[WiFi] Terhubung! IP: %s\n", WiFi.localIP().toString().c_str());
      return;
    }
  }
  Serial.println("\n[WiFi] Gagal semua network. Restart ESP...");
  ESP.restart();
}

void callback(char *topic, byte *payload, unsigned int length) {
  // Gunakan DynamicJsonDocument agar aman dari Stack Overflow
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, payload, length);

  if (error) {
    Serial.printf("[MQTT ERROR] Gagal parse JSON: %s\n", error.c_str());
    return;
  }

  // --- 1. UPDATE MODE KONTROL ---
  if (doc.containsKey("kontrol_aktif")) {
    String mode = doc["kontrol_aktif"].as<String>();
    if (mode == "Fuzzy") kontrolAktif = FUZZY;
    else kontrolAktif = PID;
    resetPID(); 
    Serial.println("\n========================================");
    Serial.printf("[MODE] Ganti Mode Kontrol ke: %s\n", mode.c_str());
    Serial.println("========================================");
  }

  // --- 2. UPDATE SETPOINT ---
  if (doc.containsKey("suhu_setpoint")) {
    suhuSetpoint = doc["suhu_setpoint"];
    Serial.printf("[SETPOINT] Target Suhu Baru: %.2f C\n", suhuSetpoint);
  }
  if (doc.containsKey("keruh_setpoint")) {
    turbiditySetpoint = doc["keruh_setpoint"];
    Serial.printf("[SETPOINT] Target Kekeruhan Baru: %.2f %%\n", turbiditySetpoint);
  }

  // --- 3. UPDATE TUNING PID ---
  bool tuningUpdated = false;
  
  // PID Suhu
  if (doc.containsKey("kp_suhu")) { Kp_suhu = doc["kp_suhu"]; tuningUpdated = true; }
  if (doc.containsKey("ki_suhu")) { Ki_suhu = doc["ki_suhu"]; tuningUpdated = true; }
  if (doc.containsKey("kd_suhu")) { Kd_suhu = doc["kd_suhu"]; tuningUpdated = true; }

  // PID Keruh
  if (doc.containsKey("kp_keruh")) { Kp_keruh = doc["kp_keruh"]; tuningUpdated = true; }
  if (doc.containsKey("ki_keruh")) { Ki_keruh = doc["ki_keruh"]; tuningUpdated = true; }
  if (doc.containsKey("kd_keruh")) { Kd_keruh = doc["kd_keruh"]; tuningUpdated = true; }

  if (tuningUpdated) {
    Serial.println("\n----------- PID PARAMETER BERHASIL DI-UPDATE -----------");
    // PERBAIKAN: Menghapus %s dan timeStr yang bikin crash
    Serial.printf("[PID SUHU ] Kp: %.2f | Ki: %.2f | Kd: %.2f\n", Kp_suhu, Ki_suhu, Kd_suhu);
    Serial.printf("[PID KERUH] Kp: %.2f | Ki: %.2f | Kd: %.2f\n", Kp_keruh, Ki_keruh, Kd_keruh);
    Serial.println("--------------------------------------------------------");
  }

  // --- 4. UPDATE KALIBRASI ---
  bool calibUpdated = false;
  if (doc.containsKey("adc_jernih")) { 
    NILAI_ADC_JERNIH = doc["adc_jernih"]; 
    calibUpdated = true; 
  }
  if (doc.containsKey("adc_keruh")) { 
    NILAI_ADC_KERUH = doc["adc_keruh"]; 
    calibUpdated = true; 
  }

  if (calibUpdated) {
    Serial.println("\n!!!!!!!!!! CALIBRATION UPDATED !!!!!!!!!!");
    // PERBAIKAN: Menghapus %s dan timeStr
    Serial.printf("[CALIB] ADC Jernih (0%%)   : %d\n", NILAI_ADC_JERNIH);
    Serial.printf("[CALIB] ADC Keruh (100%%)  : %d\n", NILAI_ADC_KERUH);
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }
}

bool reconnect_mqtt() {
  if (mqttClient.connect(MQTT_CLIENT_ID)) {
    mqttClient.subscribe(MQTT_TOPIC_MODE, 1); 
    return true;
  }
  return false;
}

// =========================================================================
//                  PEMBACAAN SENSOR
// =========================================================================

float bacaSuhuDS18B20() {
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  if (tempC == -127.00f || isnan(tempC)) {
    return (suhuTerfilter == 0.0) ? 28.0 : suhuTerfilter;
  }

  if (suhuTerfilter == 0.0) suhuTerfilter = tempC;
  else suhuTerfilter = (ALPHA * tempC) + ((1.0 - ALPHA) * suhuTerfilter);
  
  suhuTerakhir = suhuTerfilter; 
  return suhuTerfilter;
}

int bacaTurbidity() {
  long totalADC = 0;
  for (int i = 0; i < 20; i++) {
    int16_t val = ads.readADC_SingleEnded(0);
    if (val < 0) val = 0;
    if (val > 32767) val = 32767;
    totalADC += val;
    delay(2);
  }
  int avgADC = totalADC / 20;
  turbidityTerakhir = avgADC; // Update nilai ADC global
  return avgADC;
}

float konversiTurbidityKePersen(int adcValue) {
  float persen = mapFloat((float)adcValue, (float)NILAI_ADC_KERUH, (float)NILAI_ADC_JERNIH, 100.0, 0.0);
  return constrain(persen, 0.0f, 100.0f);
}

// =========================================================================
//                  SETUP & LOOP UTAMA
// =========================================================================

void setup() {
  Serial.begin(115200);
  
  Wire.begin();
  if (!ads.begin()) {
    Serial.println("[ERR] ADS1115 Tidak Terdeteksi!");
    while (1);
  }
  
  setupL298N();
  sensors.begin();
  resetPID();
  setup_wifi();
  
  mqttClient.setBufferSize(512); 
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(callback);
  
  Serial.println("\n=== SISTEM SIAP: RISET KENDALI HYBRID ===");
}

void loop() {
  unsigned long now = millis();

  if (now - lastWiFiCheck >= wifiCheckInterval) {
    lastWiFiCheck = now;
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.disconnect(); WiFi.reconnect();
    }
  }

  if (!mqttClient.connected()) reconnect_mqtt();
  mqttClient.loop();

  if (now - waktuTerakhirKirim >= intervalKirim) {
    waktuTerakhirKirim = now;

    // 1. Baca Sensor
    float suhuAktual = bacaSuhuDS18B20();
    int turbidityADC = bacaTurbidity();
    float turbidityPersen = konversiTurbidityKePersen(turbidityADC);

    // 2. Hitung Error
    float errorSuhu = suhuSetpoint - suhuAktual;
    float errorKeruh = turbidityPersen - turbiditySetpoint;

    // 3. Hitung Output Kontrol
    double outSuhu, outKeruh;
    if (kontrolAktif == FUZZY) {
      outSuhu = hitungFuzzySuhu(errorSuhu);
      outKeruh = hitungFuzzyKeruh(errorKeruh);
    } else {
      outSuhu = hitungPIDSuhu(errorSuhu);
      outKeruh = hitungPIDKeruh(errorKeruh);
    }

    // 4. Eksekusi ke Motor
    int pwmSuhu = constrain((int)(outSuhu * 2.55), 0, 255);
    int pwmKeruh = constrain((int)(outKeruh * 2.55), 0, 255);

    setHeaterSpeed(pwmSuhu);
    setPumpSpeed(pwmKeruh);

    // 5. Kirim Telemetri ke Dashboard (MQTT)
    if (mqttClient.connected()) {
      StaticJsonDocument<512> doc;

      // --- Basic Data (Nama Key sesuai kode lama) ---
      doc["timestamp_ms"] = now;  
      doc["suhu"] = round(suhuAktual * 100) / 100.0;
      doc["turbidity_persen"] = round(turbidityPersen * 100) / 100.0;
      doc["turbidity_adc"] = turbidityADC;
      doc["kontrol_aktif"] = (kontrolAktif == FUZZY) ? "Fuzzy" : "PID"; // Kembali ke "kontrol_aktif"
      
      // Kirim nilai 0-100% (bukan PWM 0-255) agar enak dibaca di grafik
      doc["pwm_heater"] = round(outSuhu * 100) / 100.0; 
      doc["pwm_pompa"] = round(outKeruh * 100) / 100.0;

      // --- Research Data ---
      doc["error_suhu"] = round(errorSuhu * 1000) / 1000.0;
      doc["error_keruh"] = round(errorKeruh * 1000) / 1000.0;
      doc["setpoint_suhu"] = suhuSetpoint;
      doc["setpoint_keruh"] = turbiditySetpoint;

      // --- Data Tambahan (Optional - Fitur Baru) ---
      // Ini tidak akan merusak dashboard lama, cuma nambah info jika ingin dipakai
      doc["feedforward_active"] = (abs(errorKeruh) < 3.0 && turbidityPersen > 9.0);
      
      char buffer[512];
      serializeJson(doc, buffer);
      mqttClient.publish(MQTT_TOPIC_DATA, buffer, false); 
    }

    // 6. Debug Lengkap di Serial Monitor 
    unsigned long s = now / 1000;      // Total detik
    unsigned long m = s / 60;          // Total menit
    unsigned long h = m / 60;          // Total jam

    Serial.println("\n-------------------------------------------------------------");
    Serial.printf("[%02lu:%02lu:%02lu] [SYSTEM] Mode: %s | WiFi: %s (%d dBm)\n", 
      (h % 24), (m % 60), (s % 60), 
      (kontrolAktif == FUZZY) ? "FUZZY" : "PID (ADAPTIVE)", 
      WiFi.status() == WL_CONNECTED ? "ONLINE" : "OFFLINE", 
      WiFi.RSSI()
    );
    
    Serial.printf("[TURBIDITY] Current: %.2f%% (Set: %.1f%%) | Error: %.2f\n", 
      turbidityPersen, turbiditySetpoint, errorKeruh
    );
    Serial.printf("            ADC Val: %d | Calib: [Jernih:%d - Keruh:%d]\n", 
      turbidityADC, NILAI_ADC_JERNIH, NILAI_ADC_KERUH
    );
    Serial.printf("            Output : %.1f%% (PWM: %d) | Feedforward: %s\n", 
      outKeruh, pwmKeruh, 
      (abs(errorKeruh) < 3.0 && turbidityPersen > 9.0) ? "ON" : "OFF"
    );

    Serial.printf("[TEMP]      Current: %.2f°C (Set: %.1f°C) | Error: %.2f\n", 
      suhuAktual, suhuSetpoint, errorSuhu
    );
    Serial.printf("            Output : %.1f%% (PWM: %d)\n", outSuhu, pwmSuhu);
    Serial.println("-------------------------------------------------------------");
  }
}