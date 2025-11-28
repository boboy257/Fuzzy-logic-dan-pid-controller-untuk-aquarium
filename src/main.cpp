#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <esp_arduino_version.h>

// =========================================================================
//                  CONFIGURATION
// =========================================================================
// WiFi Configuration - Support Multiple Networks
struct WiFiCredentials
{
  const char *ssid;
  const char *password;
};

// Definisikan Multiple WiFi (WiFi pertama akan dicoba terlebih dahulu)
WiFiCredentials wifiNetworks[] = {
    {"Private u52", "12345678"},     // WiFi Utama
    {"iPhone 2", "bobo2002"} // WiFi Cadangan (ganti sesuai kebutuhan)
    // Tambahkan WiFi lain jika diperlukan:
};
const int NUM_WIFI_NETWORKS = sizeof(wifiNetworks) / sizeof(wifiNetworks[0]);

//MQTT Configuration
const char *MQTT_BROKER = "broker.hivemq.com";
const int MQTT_PORT = 1883;
const char *MQTT_TOPIC_DATA = "unhas/informatika/aquarium/data";
const char *MQTT_TOPIC_MODE = "unhas/informatika/aquarium/mode";
const char *MQTT_CLIENT_ID = "esp32-research-aquarium";

// Pin Configuration untuk L298N
const int SENSOR_SUHU_PIN = 4;

// L298N Motor A - PTC Heater 12V DC
const int HEATER_ENA = 16; // Enable A (PWM)
const int HEATER_IN1 = 17; // Input 1
const int HEATER_IN2 = 18; // Input 2

// L298N Motor B - mini Pump 12V DC
const int PUMP_ENB = 27; // Enable B (PWM)
const int PUMP_IN3 = 25; // Input 3
const int PUMP_IN4 = 26; // Input 4

// Control Modes
enum ControlMode
{
  FUZZY,
  PID
};
ControlMode kontrolAktif = FUZZY;

float suhuSetpoint = 28.0f;
float turbiditySetpoint = 10.0f;

// PID Parameters - Temperature
double Kp_suhu = 8.0;
double Ki_suhu = 0.3;
double Kd_suhu = 6.0;
double integralSumSuhu = 0.0;
double lastErrorSuhu = 0.0;

// PID Parameters - Turbidity
double Kp_keruh = 5.0;
double Ki_keruh = 0.2;
double Kd_keruh = 2.0;
double integralSumKeruh = 0.0;
double lastErrorKeruh = 0.0;

// Sensor Calibration - Tubidity
int NILAI_ADC_JERNIH = 9475;
int NILAI_ADC_KERUH = 3550;

// Timing
unsigned long lastTimeSuhu = 0;
unsigned long lastTimeKeruh = 0;
unsigned long waktuTerakhirKirim = 0;
const long intervalKirim = 1000; // 1 second for research

// WiFi Monitoring
unsigned long lastWiFiCheck = 0;
const long wifiCheckInterval = 5000;

// PWM Configuration
const int PWM_CHANNEL_HEATER = 0;
const int PWM_CHANNEL_PUMP = 1;
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;

// Tuning Fisik Pompa (Anti-Stiction & Maintenance Flow)
const int PWM_MIN_FISIK = 180; // Batas minimal tenaga agar pompa 5W berputar
const int PWM_START_LOGIKA = 5;

// Global Objects
WiFiClient espClient;
PubSubClient mqttClient(espClient);
OneWire oneWire(SENSOR_SUHU_PIN);
DallasTemperature sensors(&oneWire);
Adafruit_ADS1115 ads;

// Last Known Values
float suhuTerakhir = 25.0f;
int turbidityTerakhir = 0;

float suhuTerfilter = 0.0;
const float ALPHA = 0.2;

// [BARU] Fungsi Helper untuk konversi desimal yang presisi
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// =========================================================================
//          L298N MOTOR CONTROL FUNCTIONS
// =========================================================================

void setHeaterSpeed(int pwmValue)
{
  pwmValue = constrain(pwmValue, 0, 255);

  if (pwmValue > 0)
  {
    digitalWrite(HEATER_IN1, HIGH);
    digitalWrite(HEATER_IN2, LOW);
  }
  else
  {
    digitalWrite(HEATER_IN1, LOW);
    digitalWrite(HEATER_IN2, LOW);
  }

// Support ESP32 Core v2.x and v3.x
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(HEATER_ENA, pwmValue);
#else
  ledcWrite(PWM_CHANNEL_HEATER, pwmValue);
#endif
}

void setPumpSpeed(int pwmValue)
{
  pwmValue = constrain(pwmValue, 0, 255);
  int finalOutput = 0;

  if (pwmValue < PWM_START_LOGIKA)
  {
    finalOutput = 0;
  }
  else
  {
    // Mapping dari Range Logika (5-255) ke Range Fisik (180-255)
    finalOutput = map(pwmValue, PWM_START_LOGIKA, 255, PWM_MIN_FISIK, 255);
  }

  if (finalOutput > 0)
  {
    digitalWrite(PUMP_IN3, HIGH);
    digitalWrite(PUMP_IN4, LOW);
  }
  else
  {
    digitalWrite(PUMP_IN3, LOW);
    digitalWrite(PUMP_IN4, LOW);
  }

// Support ESP32 Core v2.x and v3.x
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(PUMP_ENB, finalOutput);
#else
  ledcWrite(PWM_CHANNEL_PUMP, finalOutput);
#endif
}

void setupL298N()
{
  pinMode(HEATER_IN1, OUTPUT);
  pinMode(HEATER_IN2, OUTPUT);
  pinMode(PUMP_IN3, OUTPUT);
  pinMode(PUMP_IN4, OUTPUT);

// KONFIGURASI PWM UNIVERSAL
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttach(HEATER_ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PUMP_ENB, PWM_FREQ, PWM_RESOLUTION);
#else
  ledcSetup(PWM_CHANNEL_HEATER, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(HEATER_ENA, PWM_CHANNEL_HEATER);
  ledcSetup(PWM_CHANNEL_PUMP, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PUMP_ENB, PWM_CHANNEL_PUMP);
#endif

  setHeaterSpeed(0);
  setPumpSpeed(0);
  Serial.println("[OK] L298N driver initialized (Universal Fix + Remapping)");
}

// =========================================================================
//          OPTIMIZED FUZZY LOGIC - TEMPERATURE (5 MEMBERSHIP FUNCTIONS)
// =========================================================================
// Dirancang untuk respons cepat dengan stabilitas tinggi
// Asumsi: setpoint dinamis (misal 33.5°C), error = setpoint - suhuAktual

// 1. SANGAT DINGIN - Butuh pemanasan maksimum
// Aktif: error > 3.5 (suhuAktual < setpoint - 3.5)
// Puncak: error >= 5.0
float membershipSangatDingin(float error)
{
  if (error <= 3.5f)
    return 0.0f;
  if (error >= 5.0f)
    return 1.0f;
  return (error - 3.5f) / 1.5f; // Naik cepat dari 3.5 ke 5.0
}

// 2. DINGIN - Butuh pemanasan tinggi
// Aktif: 1.5 < error < 4.5
// Puncak: 2.5 <= error <= 3.5 (ZONA KRITIS untuk kasus Anda!)
float membershipDingin(float error)
{
  if (error <= 1.5f || error >= 4.5f)
    return 0.0f;
  if (error >= 2.5f && error <= 3.5f)
    return 1.0f; // Puncak di zona error Anda
  if (error > 1.5f && error < 2.5f)
    return (error - 1.5f) / 1.0f;
  if (error > 3.5f && error < 4.5f)
    return (4.5f - error) / 1.0f;
  return 0.0f;
}

// 3. SESUAI - Zona setpoint
// PILIHAN A: Puncak ±0.3°C (Agresif, respons cepat)
// PILIHAN B: Puncak ±0.5°C (Seimbang, lebih stabil)
// Anda bisa pilih salah satu sesuai kebutuhan

// PILIHAN A: Puncak Sempit ±0.3°C (DIREKOMENDASIKAN untuk kasus Anda)
// Aktif: -1.0 < error < 2.0
// Puncak: -0.3 <= error <= 0.3
float membershipSesuai(float error)
{
  if (error <= -1.0f || error >= 2.0f)
    return 0.0f;
  if (error >= -0.3f && error <= 0.3f)
    return 1.0f; // Zona stabil sangat sempit, sistem akan berusaha keras stay di sini
  if (error > -1.0f && error < -0.3f)
    return (error + 1.0f) / 0.7f;
  if (error > 0.3f && error < 2.0f)
    return (2.0f - error) / 1.7f;
  return 0.0f;
}

/* PILIHAN B: Jika ingin lebih konservatif, gunakan ini:
// Aktif: -1.2 < error < 2.0
// Puncak: -0.5 <= error <= 0.5
float membershipSesuai(float error)
{
  if (error <= -1.2f || error >= 2.0f)
    return 0.0f;
  if (error >= -0.5f && error <= 0.5f)
    return 1.0f; // Zona stabil lebih lebar
  if (error > -1.2f && error < -0.5f)
    return (error + 1.2f) / 0.7f;
  if (error > 0.5f && error < 2.0f)
    return (2.0f - error) / 1.5f;
  return 0.0f;
}
*/

// 4. PANAS - Mulai kurangi pemanasan
// Aktif: -3.5 < error < -0.5
// Puncak: -2.5 <= error <= -1.0
float membershipPanas(float error)
{
  if (error <= -3.5f || error >= -0.5f)
    return 0.0f;
  if (error >= -2.5f && error <= -1.0f)
    return 1.0f;
  if (error > -3.5f && error < -2.5f)
    return (error + 3.5f) / 1.0f;
  if (error > -1.0f && error < -0.5f)
    return (-0.5f - error) / 0.5f;
  return 0.0f;
}

// 5. SANGAT PANAS - Matikan heater
// Aktif: error < -3.0 (suhuAktual > setpoint + 3.0)
// Puncak: error <= -4.5
float membershipSangatPanas(float error)
{
  if (error >= -3.0f)
    return 0.0f;
  if (error <= -4.5f)
    return 1.0f;
  return (-3.0f - error) / 1.5f;
}

// DEFUZZIFIKASI - Output yang lebih agresif untuk pemanasan
float hitungFuzzySuhu(float errorSuhu)
{
  float mu_sangatDingin = membershipSangatDingin(errorSuhu);
  float mu_dingin = membershipDingin(errorSuhu);
  float mu_sesuai = membershipSesuai(errorSuhu);
  float mu_panas = membershipPanas(errorSuhu);
  float mu_sangatPanas = membershipSangatPanas(errorSuhu);

  // Output PWM% yang lebih agresif:
  // - Sangat Dingin: 95% (maksimum penuh)
  // - Dingin: 75% (tinggi untuk kasus Anda dengan error 3.5)
  // - Sesuai: 25% (maintenance level, tetap menjaga suhu)
  // - Panas: 5% (minimal, hampir mati)
  // - Sangat Panas: 0% (mati total)

  float numerator = (mu_sangatDingin * 95.0f) +
                    (mu_dingin * 75.0f) + // PENINGKATAN dari 60% ke 75%
                    (mu_sesuai * 25.0f) +
                    (mu_panas * 5.0f) +
                    (mu_sangatPanas * 0.0f);

  float denominator = mu_sangatDingin + mu_dingin + mu_sesuai + mu_panas + mu_sangatPanas;

  if (denominator < 0.01f)
  {
    return 25.0f; // Maintenance level default
  }

  return numerator / denominator;
}

// =========================================================================
//          OPTIMIZED FUZZY LOGIC - TURBIDITY (5 MEMBERSHIP FUNCTIONS)
// =========================================================================
// Mengikuti pola yang sama dengan peningkatan agresivitas

// 1. SANGAT JERNIH - Air terlalu bersih, pompa bisa dikurangi
// Aktif: error <= -5.0 (turbidityPersen <= 5%)
// Puncak: error <= -7.0
float membershipSangatJernih(float error)
{
  if (error <= -7.0f)
    return 1.0f;
  if (error <= -5.0f)
    return (-5.0f - error) / 2.0f;
  return 0.0f;
}

// 2. JERNIH - Air bersih, pompa minimal
// Aktif: -7.0 < error < -1.0
// Puncak: -4.0 <= error <= -2.0
float membershipJernih(float error)
{
  if (error <= -7.0f || error >= -1.0f)
    return 0.0f;
  if (error >= -4.0f && error <= -2.0f)
    return 1.0f;
  if (error > -7.0f && error < -4.0f)
    return (error + 7.0f) / 3.0f;
  if (error > -2.0f && error < -1.0f)
    return (-1.0f - error) / 1.0f;
  return 0.0f;
}

// 3. SESUAI - Zona setpoint kekeruhan (DIPERSEMPIT)
// Aktif: -2.5 < error < 2.5
// Puncak: -0.5 <= error <= 0.5
float membershipSesuaiKeruh(float error)
{
  if (error <= -2.5f || error >= 2.5f)
    return 0.0f;
  if (error >= -0.5f && error <= 0.5f)
    return 1.0f;
  if (error > -2.5f && error < -0.5f)
    return (error + 2.5f) / 2.0f;
  if (error > 0.5f && error < 2.5f)
    return (2.5f - error) / 2.0f;
  return 0.0f;
}

// 4. KERUH - Air mulai keruh, pompa perlu ditingkatkan
// Aktif: 1.0 < error < 10.0
// Puncak: 4.0 <= error <= 7.0
float membershipKeruh(float error)
{
  if (error <= 1.0f || error >= 10.0f)
    return 0.0f;
  if (error >= 4.0f && error <= 7.0f)
    return 1.0f;
  if (error > 1.0f && error < 4.0f)
    return (error - 1.0f) / 3.0f;
  if (error > 7.0f && error < 10.0f)
    return (10.0f - error) / 3.0f;
  return 0.0f;
}

// 5. SANGAT KERUH - Air sangat keruh, pompa maksimum
// Aktif: error >= 8.0
// Puncak: error >= 12.0
float membershipSangatKeruh(float error)
{
  if (error <= 8.0f)
    return 0.0f;
  if (error >= 12.0f)
    return 1.0f;
  return (error - 8.0f) / 4.0f;
}

// DEFUZZIFIKASI - Output yang lebih responsif
float hitungFuzzyKeruh(float errorKeruh)
{
  float mu_sangatJernih = membershipSangatJernih(errorKeruh);
  float mu_jernih = membershipJernih(errorKeruh);
  float mu_sesuai = membershipSesuaiKeruh(errorKeruh);
  float mu_keruh = membershipKeruh(errorKeruh);
  float mu_sangatKeruh = membershipSangatKeruh(errorKeruh);

  // Output PWM% yang lebih agresif:
  float numerator = (mu_sangatJernih * 0.0f) +
                    (mu_jernih * 20.0f) +
                    (mu_sesuai * 45.0f) +
                    (mu_keruh * 70.0f) +      // PENINGKATAN dari 60% ke 70%
                    (mu_sangatKeruh * 95.0f); // PENINGKATAN dari 85% ke 95%

  float denominator = mu_sangatJernih + mu_jernih + mu_sesuai + mu_keruh + mu_sangatKeruh;

  if (denominator < 0.01f)
  {
    return 45.0f;
  }

  return numerator / denominator;
}

// =========================================================================
//          IMPROVED PID CONTROL - TEMPERATURE
// =========================================================================
double hitungPIDSuhu(float errorSuhu)
{
  unsigned long now = millis();
  double elapsedTime = (double)(now - lastTimeSuhu);
  if (elapsedTime < 1)
    elapsedTime = 1;
  double dt = elapsedTime / 1000.0;

  // PROPORTIONAL
  double P = Kp_suhu * errorSuhu;

  // INTEGRAL dengan anti-windup yang lebih baik
  integralSumSuhu += errorSuhu * dt;

  // Anti-windup dengan batas lebih ketat
  if (integralSumSuhu > 20.0)
    integralSumSuhu = 20.0;
  if (integralSumSuhu < -20.0)
    integralSumSuhu = -20.0;

  // Reset 50% saat crossing setpoint
  if ((errorSuhu > 0 && lastErrorSuhu < 0) || (errorSuhu < 0 && lastErrorSuhu > 0))
  {
    integralSumSuhu *= 0.5;
  }

  double I = Ki_suhu * integralSumSuhu;

  // DERIVATIVE dengan low-pass filter
  static double lastDerivative = 0.0;
  double derivative = (errorSuhu - lastErrorSuhu) / dt;
  derivative = 0.3 * derivative + 0.7 * lastDerivative;
  lastDerivative = derivative;

  double D = Kd_suhu * derivative;

  double output = P + I + D;

  lastErrorSuhu = errorSuhu;
  lastTimeSuhu = now;

  return constrain(output, 0.0, 100.0);
}

void resetPIDSuhu()
{
  integralSumSuhu = 0.0;
  lastErrorSuhu = 0.0;
  lastTimeSuhu = millis();
}

// =========================================================================
//          IMPROVED PID CONTROL - TURBIDITY
// =========================================================================
double hitungPIDKeruh(float errorKeruh)
{
  unsigned long now = millis();
  double elapsedTime = (double)(now - lastTimeKeruh);
  if (elapsedTime < 1)
    elapsedTime = 1;
  double dt = elapsedTime / 1000.0;

  double P = Kp_keruh * errorKeruh;

  integralSumKeruh += errorKeruh * dt;
  if (integralSumKeruh > 30.0)
    integralSumKeruh = 30.0;
  if (integralSumKeruh < -30.0)
    integralSumKeruh = -30.0;

  if ((errorKeruh > 0 && lastErrorKeruh < 0) || (errorKeruh < 0 && lastErrorKeruh > 0))
  {
    integralSumKeruh *= 0.5;
  }

  double I = Ki_keruh * integralSumKeruh;

  static double lastDerivativeKeruh = 0.0;
  double derivative = (errorKeruh - lastErrorKeruh) / dt;
  derivative = 0.3 * derivative + 0.7 * lastDerivativeKeruh;
  lastDerivativeKeruh = derivative;

  double D = Kd_keruh * derivative;

  double output = P + I + D;

  lastErrorKeruh = errorKeruh;
  lastTimeKeruh = now;

  return constrain(output, 0.0, 100.0);
}

void resetPIDKeruh()
{
  integralSumKeruh = 0.0;
  lastErrorKeruh = 0.0;
  lastTimeKeruh = millis();
}

// =========================================================================
//                WIFI & MQTT
// =========================================================================
void setup_wifi()
{
  delay(10);
  Serial.println("\n[WiFi] Starting multi-network connection...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  // Coba connect ke setiap WiFi yang tersedia
  for (int network = 0; network < NUM_WIFI_NETWORKS; network++)
  {
    Serial.printf("\n[WiFi] Trying network %d: %s\n", network + 1, wifiNetworks[network].ssid);
    WiFi.begin(wifiNetworks[network].ssid, wifiNetworks[network].password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) // 20 attempts = 10 detik
    {
      delay(500);
      Serial.print(".");
      attempts++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("\n[WiFi] ✅ Connected!");
      Serial.printf("[WiFi] Network: %s\n", wifiNetworks[network].ssid);
      Serial.printf("[WiFi] IP Address: %s\n", WiFi.localIP().toString().c_str());
      Serial.printf("[WiFi] Signal Strength: %d dBm\n", WiFi.RSSI());
      return; // Berhasil connect, keluar dari fungsi
    }
    else
    {
      Serial.printf("\n[WiFi] ❌ Failed to connect to %s\n", wifiNetworks[network].ssid);
      WiFi.disconnect();
      delay(1000);
    }
  }

  // Jika semua WiFi gagal
  Serial.println("\n[WiFi] ⚠️ ALL NETWORKS FAILED!");
  Serial.println("[WiFi] Restarting ESP32 in 5 seconds...");
  delay(5000);
  ESP.restart();
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.println("\n========================================");
  Serial.println("[MQTT] ✅ Message received!");
  Serial.print("[MQTT] Topic: ");
  Serial.println(topic);
  Serial.print("[MQTT] Payload length: ");
  Serial.println(length);
  Serial.print("[MQTT] Payload: ");

  // Print raw payload
  for (unsigned int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (strcmp(topic, MQTT_TOPIC_MODE) != 0)
  {
    Serial.println("[MQTT] ❌ Wrong topic, ignoring...");
    Serial.println("========================================\n");
    return;
  }

  StaticJsonDocument<512> doc; // Tingkatkan dari 400 ke 512
  char payloadStr[length + 1];
  memcpy(payloadStr, payload, length);
  payloadStr[length] = '\0';

  DeserializationError error = deserializeJson(doc, payloadStr);

  if (error)
  {
    Serial.print("[MQTT] ❌ JSON Parse Error: ");
    Serial.println(error.c_str());
    Serial.println("========================================\n");
    return;
  }

  Serial.println("[MQTT] ✅ JSON parsed successfully!");

  // Control Mode
  if (doc.containsKey("kontrol_aktif"))
  {
    String mode = doc["kontrol_aktif"].as<String>();
    if (mode == "Fuzzy")
    {
      kontrolAktif = FUZZY;
      resetPIDSuhu();
      resetPIDKeruh();
      Serial.println("[MODE] Changed to: FUZZY");
    }
    else if (mode == "PID")
    {
      kontrolAktif = PID;
      Serial.println("[MODE] Changed to: PID");
    }
  }

  // Setpoints
  if (doc.containsKey("suhu_setpoint"))
  {
    suhuSetpoint = doc["suhu_setpoint"];
    Serial.print("[SETPOINT] Suhu updated: ");
    Serial.println(suhuSetpoint);
  }
  if (doc.containsKey("keruh_setpoint"))
  {
    turbiditySetpoint = doc["keruh_setpoint"];
    Serial.print("[SETPOINT] Kekeruhan updated: ");
    Serial.println(turbiditySetpoint);
  }

  // PID Parameters
  if (doc.containsKey("kp_suhu"))
  {
    Kp_suhu = doc["kp_suhu"];
    Serial.print("[PID] Kp_suhu updated: ");
    Serial.println(Kp_suhu);
  }
  if (doc.containsKey("ki_suhu"))
  {
    Ki_suhu = doc["ki_suhu"];
    Serial.print("[PID] Ki_suhu updated: ");
    Serial.println(Ki_suhu);
  }
  if (doc.containsKey("kd_suhu"))
  {
    Kd_suhu = doc["kd_suhu"];
    Serial.print("[PID] Kd_suhu updated: ");
    Serial.println(Kd_suhu);
  }
  if (doc.containsKey("kp_keruh"))
  {
    Kp_keruh = doc["kp_keruh"];
    Serial.print("[PID] Kp_keruh updated: ");
    Serial.println(Kp_keruh);
  }
  if (doc.containsKey("ki_keruh"))
  {
    Ki_keruh = doc["ki_keruh"];
    Serial.print("[PID] Ki_keruh updated: ");
    Serial.println(Ki_keruh);
  }
  if (doc.containsKey("kd_keruh"))
  {
    Kd_keruh = doc["kd_keruh"];
    Serial.print("[PID] Kd_keruh updated: ");
    Serial.println(Kd_keruh);
  }

  // ========== KALIBRASI ADC - PERBAIKAN DI SINI ==========
  bool adcUpdated = false;

  if (doc.containsKey("adc_jernih"))
  {
    int newValue = doc["adc_jernih"];
    Serial.print("[CALIBRATION] ADC Jernih - Old: ");
    Serial.print(NILAI_ADC_JERNIH);
    Serial.print(" → New: ");
    Serial.println(newValue);

    NILAI_ADC_JERNIH = newValue;
    adcUpdated = true;
  }

  if (doc.containsKey("adc_keruh"))
  {
    int newValue = doc["adc_keruh"];
    Serial.print("[CALIBRATION] ADC Keruh - Old: ");
    Serial.print(NILAI_ADC_KERUH);
    Serial.print(" → New: ");
    Serial.println(newValue);

    NILAI_ADC_KERUH = newValue;
    adcUpdated = true;
  }

  if (adcUpdated)
  {
    Serial.println("[CALIBRATION] ✅ ADC values updated successfully!");
    Serial.print("[CALIBRATION] Current values - Jernih: ");
    Serial.print(NILAI_ADC_JERNIH);
    Serial.print(", Keruh: ");
    Serial.println(NILAI_ADC_KERUH);
  }
  // =======================================================

  Serial.println("[MQTT] ✅ All updates completed!");
  Serial.println("========================================\n");
}

bool reconnect_mqtt()
{
  int attempts = 0;
  while (!mqttClient.connected() && attempts < 3)
  {
    Serial.print("[MQTT] Connecting... ");
    if (mqttClient.connect(MQTT_CLIENT_ID))
    {
      Serial.println("OK!");
      mqttClient.subscribe(MQTT_TOPIC_MODE, 1);
      return true;
    }
    Serial.println("FAIL");
    attempts++;
    delay(2000);
  }
  return false;
}

// =========================================================================
//                SENSOR FUNCTIONS
// =========================================================================
float bacaSuhuDS18B20()
{
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  // 1. Safety Check: Jika sensor error/copot
  if (tempC == -127.00f || isnan(tempC))
  {
    // Jika belum pernah ada data valid, kembalikan 0 atau nilai aman
    if (suhuTerfilter == 0.0)
      return 28.0;
    return suhuTerfilter; // Pakai nilai terakhir yang valid
  }

  // 2. Inisialisasi Awal (Anti-Startup Bug)
  // Jika ini pembacaan pertama (suhuTerfilter masih 0), langsung pakai nilai sensor
  if (suhuTerfilter == 0.0)
  {
    suhuTerfilter = tempC;
  }
  else
  {
    // 3. Rumus EMA Filter
    // NilaiBaru = (Faktor * DataMentah) + ((1 - Faktor) * DataLama)
    suhuTerfilter = (ALPHA * tempC) + ((1.0 - ALPHA) * suhuTerfilter);
  }

  // Update variabel global suhuTerakhir untuk keperluan lain jika ada
  suhuTerakhir = suhuTerfilter;

  return suhuTerfilter;
}

int bacaTurbidity()
{
  long totalADC = 0;
  int jumlahSampel = 20; // Ambil 20 sampel data

  for (int i = 0; i < jumlahSampel; i++)
  {
    int16_t val = ads.readADC_SingleEnded(0);

    // Safety check per sampel
    if (val < 0)
      val = 0;
    if (val > 32767)
      val = 32767;

    totalADC += val;
    delay(2); // Jeda dikit biar ADC nafas (total delay cuma 40ms)
  }

  // Hitung rata-rata
  int rataRata = totalADC / jumlahSampel;

  // Simpan ke variabel global
  turbidityTerakhir = rataRata;

  return rataRata;
}

float konversiTurbidityKePersen(int adcValue)
{
  // Logika: Nilai ADC Keruh -> 100%, Nilai ADC Jernih -> 0%
  float persen = mapFloat((float)adcValue, (float)NILAI_ADC_KERUH, (float)NILAI_ADC_JERNIH, 100.0, 0.0);

  return constrain(persen, 0.0f, 100.0f);
}

// =========================================================================
//                          MQTT PUBLISH
// =========================================================================
void kirimDataMQTT(float suhu, float turbPersen, double pwmSuhu, double pwmKeruh,
                   float errSuhu, float errKeruh, int turbADC)
{
  StaticJsonDocument<512> doc;

  // Basic Data
  doc["timestamp_ms"] = millis();
  doc["suhu"] = round(suhu * 100) / 100.0;
  doc["turbidity_persen"] = round(turbPersen * 100) / 100.0;
  doc["turbidity_adc"] = turbADC;
  doc["kontrol_aktif"] = (kontrolAktif == FUZZY) ? "Fuzzy" : "PID";
  doc["pwm_heater"] = round(pwmSuhu * 100) / 100.0;
  doc["pwm_pompa"] = round(pwmKeruh * 100) / 100.0;

  // Research Data
  doc["error_suhu"] = round(errSuhu * 1000) / 1000.0;
  doc["error_keruh"] = round(errKeruh * 1000) / 1000.0;
  doc["setpoint_suhu"] = suhuSetpoint;
  doc["setpoint_keruh"] = turbiditySetpoint;

  char buffer[512];
  serializeJson(doc, buffer);
  mqttClient.publish(MQTT_TOPIC_DATA, buffer, false);
}

// =========================================================================
//                            SETUP
// =========================================================================
void setup()
{
  Serial.begin(115200);
  Serial.println("\n=== ESP32 Research Control System with L298N ===");

  Wire.begin();
  if (!ads.begin())
  {
    Serial.println("[ERROR] ADS1115 not found!");
    while (1)
      delay(1000);
  }
  Serial.println("[OK] ADS1115 initialized");

  // Initialize L298N Motor Driver
  setupL298N();

  // Initialize DS18B20
  sensors.begin();
  Serial.println("[OK] DS18B20 sensor initialized");

  lastTimeSuhu = millis();
  lastTimeKeruh = millis();
  resetPIDSuhu();
  resetPIDKeruh();

  setup_wifi();
  mqttClient.setBufferSize(512); //size data 
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(callback);
  mqttClient.setKeepAlive(60);

  // Paksa connect MQTT Sekarang
  Serial.println("[Setup] Connecting to MQTT...");
  if (reconnect_mqtt())
  {
    Serial.println("[Setup] MQTT Connected & Subscribed!");
    // Beri waktu sebentar untuk menerima pesan retain
    for (int i = 0; i < 10; i++)
    {
      mqttClient.loop();
      delay(50);
    }
  }

  Serial.println("=== System Ready for Research ===\n");
}

  // =========================================================================
  //                       FUNGSI WiFi MONITORING 
  // =========================================================================
  void checkWiFiConnection()
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("\n[WiFi] ⚠️ Connection lost! Attempting to reconnect...");
      WiFi.disconnect();
      WiFi.reconnect();
    }
  }
  // =========================================================================
  //                              MAIN LOOP
  // =========================================================================
  void loop()
  {
    unsigned long now = millis();

    // ========== WiFi Connection Monitor (Check every 5 seconds) ==========
    if (now - lastWiFiCheck >= wifiCheckInterval)
    {
      lastWiFiCheck = now;
      checkWiFiConnection();
    }

    // ========== MQTT Connection ==========
    if (!mqttClient.connected())
    {
      reconnect_mqtt();
    }
    mqttClient.loop();

    // Data Collection Loop
    if (now - waktuTerakhirKirim >= intervalKirim)
    {
      waktuTerakhirKirim = now;

      // Read Sensors
      float suhuAktual = bacaSuhuDS18B20();
      int turbidityADC = bacaTurbidity();
      float turbidityPersen = konversiTurbidityKePersen(turbidityADC);

      // Calculate Errors
      float errorSuhu = suhuSetpoint - suhuAktual;
      float errorKeruh = turbidityPersen - turbiditySetpoint;

      // Control Outputs
      double dayaOutputSuhu = (kontrolAktif == FUZZY) ? hitungFuzzySuhu(errorSuhu) : hitungPIDSuhu(errorSuhu);
      double dayaOutputKeruh = (kontrolAktif == FUZZY) ? hitungFuzzyKeruh(errorKeruh) : hitungPIDKeruh(errorKeruh);

      int pwmSuhu = constrain((int)(dayaOutputSuhu * 2.55), 0, 255);
      int pwmKeruh = constrain((int)(dayaOutputKeruh * 2.55), 0, 255);

      // Control L298N Motors
      setHeaterSpeed(pwmSuhu);
      setPumpSpeed(pwmKeruh);

      // Send Data
      if (mqttClient.connected())
      {
        kirimDataMQTT(suhuAktual, turbidityPersen, dayaOutputSuhu, dayaOutputKeruh,
                      errorSuhu, errorKeruh, turbidityADC);
      }

      // Debug Print
      Serial.printf("[%lu] [%s] T:%.2f/%.1f [%.1f/%.1f/%.1f] E:%.2f PWM:%-3d | K:%.1f/%.1f [%.1f/%.1f/%.1f] E:%.1f PWM:%-3d | ADC:%d (J:%d K:%d) | WiFi:%s\n",
                    millis() / 1000,
                    (kontrolAktif == FUZZY) ? "FUZZY" : "PID  ", // 1. Tampilkan Mode
                    suhuAktual, suhuSetpoint,
                    Kp_suhu, Ki_suhu, Kd_suhu, // 2. Tampilkan PID Suhu
                    errorSuhu, pwmSuhu,
                    turbidityPersen, turbiditySetpoint,
                    Kp_keruh, Ki_keruh, Kd_keruh, // 3. Tampilkan PID Keruh
                    errorKeruh, pwmKeruh,
                    turbidityADC, NILAI_ADC_JERNIH, NILAI_ADC_KERUH, // 4. DATA ADC DIKEMBALIKAN
                    WiFi.status() == WL_CONNECTED ? "OK" : "LOST");
    }
  }