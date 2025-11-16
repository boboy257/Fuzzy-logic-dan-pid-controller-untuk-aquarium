#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// =========================================================================
//                  CONFIGURATION
// =========================================================================
const char *WIFI_SSID = "Private u52";
const char *WIFI_PASSWORD = "12345678";
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

// L298N Motor B - Diaphragm Pump 12V DC
const int PUMP_ENB = 19; // Enable B (PWM)
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

// Sensor Calibration
const int NILAI_ADC_JERNIH = 9475;
const int NILAI_ADC_KERUH = 3550;

// Timing
unsigned long lastTimeSuhu = 0;
unsigned long lastTimeKeruh = 0;
unsigned long waktuTerakhirKirim = 0;
const long intervalKirim = 1000; // 1 second for research

// PWM Configuration
const int PWM_CHANNEL_HEATER = 0;
const int PWM_CHANNEL_PUMP = 1;
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;

// Global Objects
WiFiClient espClient;
PubSubClient mqttClient(espClient);
OneWire oneWire(SENSOR_SUHU_PIN);
DallasTemperature sensors(&oneWire);
Adafruit_ADS1115 ads;

// Last Known Values
float suhuTerakhir = 25.0f;
int turbidityTerakhir = 0;

// =========================================================================
//          L298N MOTOR CONTROL FUNCTIONS
// =========================================================================

void setHeaterSpeed(int pwmValue)
{
  // Constrain PWM value
  pwmValue = constrain(pwmValue, 0, 255);

  if (pwmValue == 0)
  {
    // Motor OFF
    digitalWrite(HEATER_IN1, LOW);
    digitalWrite(HEATER_IN2, LOW);
    ledcWrite(PWM_CHANNEL_HEATER, 0);
  }
  else
  {
    // Motor ON - Forward direction
    digitalWrite(HEATER_IN1, HIGH);
    digitalWrite(HEATER_IN2, LOW);
    ledcWrite(PWM_CHANNEL_HEATER, pwmValue);
  }
}

void setPumpSpeed(int pwmValue)
{
  // Constrain PWM value
  pwmValue = constrain(pwmValue, 0, 255);

  if (pwmValue == 0)
  {
    // Motor OFF
    digitalWrite(PUMP_IN3, LOW);
    digitalWrite(PUMP_IN4, LOW);
    ledcWrite(PWM_CHANNEL_PUMP, 0);
  }
  else
  {
    // Motor ON - Forward direction
    digitalWrite(PUMP_IN3, HIGH);
    digitalWrite(PUMP_IN4, LOW);
    ledcWrite(PWM_CHANNEL_PUMP, pwmValue);
  }
}

void setupL298N()
{
  // Setup Heater pins (Motor A)
  pinMode(HEATER_IN1, OUTPUT);
  pinMode(HEATER_IN2, OUTPUT);
  ledcSetup(PWM_CHANNEL_HEATER, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(HEATER_ENA, PWM_CHANNEL_HEATER);

  // Setup Pump pins (Motor B)
  pinMode(PUMP_IN3, OUTPUT);
  pinMode(PUMP_IN4, OUTPUT);
  ledcSetup(PWM_CHANNEL_PUMP, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PUMP_ENB, PWM_CHANNEL_PUMP);

  // Initialize both motors to OFF
  setHeaterSpeed(0);
  setPumpSpeed(0);

  Serial.println("[OK] L298N driver initialized");
}

// =========================================================================
//          IMPROVED FUZZY LOGIC - TEMPERATURE (5 MEMBERSHIP FUNCTIONS)
// =========================================================================

// Fungsi Keanggotaan untuk Error Suhu (errorSuhu = suhuSetpoint - suhuAktual)
// Asumsi suhuSetpoint = 28.0f
// Rentang error yang relevan: sekitar -6 hingga +6 (artinya suhu aktual antara 22°C hingga 34°C, area operasional utama)

// Fungsi: Sangat Dingin
// Rentang Aktif: Error > 4.0 (artinya suhuAktual < 24.0°C)
// Puncak: Error >= 6.0 (artinya suhuAktual <= 22.0°C) <-- Lebih realistis untuk akuarium
float membershipSangatDingin(float error)
{
  if (error <= 4.0f)
    return 0.0f; // Turun
  if (error >= 6.0f)
    return 1.0f;                // Puncak
  return (error - 4.0f) / 2.0f; // Naik: dari 0.0 di error=4.0 ke 1.0 di error=6.0
}

// Fungsi: Dingin
// Rentang Aktif: 1.0 < Error < 5.0 (artinya 23.0°C < suhuAktual < 27.0°C)
// Puncak: 2.0 <= Error <= 4.0 (artinya 24.0°C <= suhuAktual <= 26.0°C)
float membershipDingin(float error)
{
  if (error <= 1.0f || error >= 5.0f)
    return 0.0f; // Luar rentang aktif
  if (error >= 2.0f && error <= 4.0f)
    return 1.0f; // Puncak
  if (error > 1.0f && error < 2.0f)
    return (error - 1.0f) / 1.0f; // Naik: dari 0.0 di error=1.0 ke 1.0 di error=2.0
  if (error > 4.0f && error < 5.0f)
    return (5.0f - error) / 1.0f; // Turun: dari 1.0 di error=4.0 ke 0.0 di error=5.0
  return 0.0f;
}

// Fungsi: Sesuai (ini adalah fungsi yang menunjukkan nilai sesuai dengan setpoint)
// Rentang Aktif: -3.0 < Error < 3.0 (artinya 25.0°C < suhuAktual < 31.0°C)
// Puncak: -1.0 <= Error <= 1.0 (artinya 27.0°C <= suhuAktual <= 29.0°C) <-- Diperlebar dari ±0.5 menjadi ±1.0
float membershipSesuai(float error)
{
  if (error <= -3.0f || error >= 3.0f)
    return 0.0f; // Luar rentang aktif
  if (error >= -1.0f && error <= 1.0f)
    return 1.0f; // Puncak
  if (error > -3.0f && error < -1.0f)
    return (error + 3.0f) / 2.0f; // Naik: dari 0.0 di error=-3.0 ke 1.0 di error=-1.0
  if (error > 1.0f && error < 3.0f)
    return (3.0f - error) / 2.0f; // Turun: dari 1.0 di error=1.0 ke 0.0 di error=3.0
  return 0.0f;
}

// Fungsi: Panas
// Rentang Aktif: -5.0 < Error < -1.0 (artinya 29.0°C < suhuAktual < 33.0°C)
// Puncak: -4.0 <= Error <= -2.0 (artinya 30.0°C <= suhuAktual <= 32.0°C)
float membershipPanas(float error)
{
  if (error <= -5.0f || error >= -1.0f)
    return 0.0f; // Luar rentang aktif
  if (error >= -4.0f && error <= -2.0f)
    return 1.0f; // Puncak
  if (error > -5.0f && error < -4.0f)
    return (error + 5.0f) / 1.0f; // Naik: dari 0.0 di error=-5.0 ke 1.0 di error=-4.0
  if (error > -2.0f && error < -1.0f)
    return (-1.0f - error) / 1.0f; // Turun: dari 1.0 di error=-2.0 ke 0.0 di error=-1.0
  return 0.0f;
}

// Fungsi: Sangat Panas
// Rentang Aktif: Error < -4.0 (artinya suhuAktual > 32.0°C)
// Puncak: Error <= -6.0 (artinya suhuAktual >= 34.0°C) <-- Lebih realistis untuk akuarium
float membershipSangatPanas(float error)
{
  if (error >= -4.0f)
    return 0.0f; // Turun
  if (error <= -6.0f)
    return 1.0f;                 // Puncak
  return (-4.0f - error) / 2.0f; // Naik: dari 0.0 di error=-4.0 ke 1.0 di error=-6.0
}

// Fungsi utama Fuzzy Logic untuk Suhu
// Pastikan juga fungsi ini menggunakan nama fungsi baru yang sudah disesuaikan
float hitungFuzzySuhu(float errorSuhu)
{
  float mu_sangatDingin = membershipSangatDingin(errorSuhu);
  float mu_dingin = membershipDingin(errorSuhu);
  float mu_sesuai = membershipSesuai(errorSuhu);
  float mu_panas = membershipPanas(errorSuhu);
  float mu_sangatPanas = membershipSangatPanas(errorSuhu);

  // --- Defuzzifikasi dengan Metode Centroid ---
  // Asumsikan output crisp berdasarkan tingkat kebutuhan pemanasan untuk heater
  // Output: 85% (heater maks), 60% (heater cepat), 30% (heater sedang), 10% (heater pelan), 0% (heater mati)
  float numerator = (mu_sangatDingin * 85.0f) + // Jika sangat dingin, heater maks
                    (mu_dingin * 60.0f) +       // Jika dingin, heater cepat
                    (mu_sesuai * 30.0f) +       // Jika sesuai, heater sedang (untuk menjaga)
                    (mu_panas * 10.0f) +        // Jika panas, heater pelan (mungkin hanya untuk sirkulasi kecil?)
                    (mu_sangatPanas * 0.0f);    // Jika sangat panas, heater mati

  float denominator = mu_sangatDingin + mu_dingin + mu_sesuai + mu_panas + mu_sangatPanas;

  if (denominator < 0.01f)
  {
    // Jika semua membership = 0 (kemungkinan kecil, tapi aman)
    // Kembalikan nilai default, misalnya nilai saat error nol (sesuai)
    return 30.0f; // Nilai default saat di setpoint
  }

  return numerator / denominator; // Nilai output fuzzy akhir (0.0 - 85.0)
}

// =========================================================================
//          IMPROVED FUZZY LOGIC - TURBIDITY (5 MEMBERSHIP FUNCTIONS)
// =========================================================================

// Fungsi Keanggotaan untuk Error Kekeruhan (errorKeruh = turbidityPersen - turbiditySetpoint)
// Asumsi setpoint = 10.0f
// Rentang error yang relevan: sekitar -10 hingga +15 (artinya nilai aktual antara 0% hingga 25%)

// Fungsi: Sangat Jernih
// Rentang Aktif: Error <= -6.0 (artinya turbidityPersen <= 4%)
// Puncak: Error <= -8.0 (artinya turbidityPersen <= 2%)
float membershipSangatJernih(float error)
{
  if (error <= -8.0f)
    return 1.0f; // Puncak
  if (error <= -6.0f)
    return (-6.0f - error) / 2.0f; // Naik
  return 0.0f;                     // Turun
}

// Fungsi: Jernih
// Rentang Aktif: -8.0 < Error < 0.0 (artinya 2% < turbidityPersen < 10%)
// Puncak: -4.0 <= Error <= -2.0 (artinya 6% <= turbidityPersen <= 8%)
float membershipJernih(float error)
{
  if (error <= -8.0f || error >= 0.0f)
    return 0.0f;
  if (error >= -4.0f && error <= -2.0f)
    return 1.0f; // Puncak
  if (error > -8.0f && error < -4.0f)
    return (error + 8.0f) / 4.0f; // Naik
  if (error > -2.0f && error < 0.0f)
    return (0.0f - error) / 2.0f; // Turun
  return 0.0f;
}

// Fungsi: Sesuai (ini adalah fungsi yang menunjukkan nilai sesuai dengan setpoint)
// Rentang Aktif: -4.0 < Error < 4.0 (artinya 6% < turbidityPersen < 14%)
// Puncak: -1.0 <= Error <= 1.0 (artinya 9% <= turbidityPersen <= 11%)
float membershipSesuaiKeruh(float error)
{
  if (error <= -4.0f || error >= 4.0f)
    return 0.0f;
  if (error >= -1.0f && error <= 1.0f)
    return 1.0f; // Puncak
  if (error > -4.0f && error < -1.0f)
    return (error + 4.0f) / 3.0f; // Naik
  if (error > 1.0f && error < 4.0f)
    return (4.0f - error) / 3.0f; // Turun
  return 0.0f;
}

// Fungsi: Keruh
// Rentang Aktif: 1.0 < Error < 12.0 (artinya 11% < turbidityPersen < 22%)
// Puncak: 5.0 <= Error <= 8.0 (artinya 15% <= turbidityPersen <= 18%)
float membershipKeruh(float error)
{
  if (error <= 1.0f || error >= 12.0f)
    return 0.0f;
  if (error >= 5.0f && error <= 8.0f)
    return 1.0f; // Puncak
  if (error > 1.0f && error < 5.0f)
    return (error - 1.0f) / 4.0f; // Naik
  if (error > 8.0f && error < 12.0f)
    return (12.0f - error) / 4.0f; // Turun
  return 0.0f;
}

// Fungsi: Sangat Keruh
// Rentang Aktif: Error >= 9.0 (artinya turbidityPersen >= 19%)
// Puncak: Error >= 15.0 (artinya turbidityPersen >= 25%)
float membershipSangatKeruh(float error)
{
  if (error <= 9.0f)
    return 0.0f; // Turun
  if (error >= 15.0f)
    return 1.0f;                // Puncak
  return (error - 9.0f) / 6.0f; // Naik
}

// Fungsi utama Fuzzy Logic untuk Kekeruhan
// Pastikan juga fungsi ini menggunakan nama fungsi baru yang sudah disesuaikan
float hitungFuzzyKeruh(float errorKeruh)
{
  float mu_sangatJernih = membershipSangatJernih(errorKeruh);
  float mu_jernih = membershipJernih(errorKeruh);
  float mu_sesuai = membershipSesuaiKeruh(errorKeruh);
  float mu_keruh = membershipKeruh(errorKeruh);
  float mu_sangatKeruh = membershipSangatKeruh(errorKeruh);

  // --- Defuzzifikasi dengan Metode Centroid ---
  // Asumsikan output crisp berdasarkan tingkat kekeruhan yang diinginkan untuk pompa
  // Output: 0% (pompa mati), 15% (pompa pelan), 30% (pompa sedang), 60% (pompa cepat), 85% (pompa maks)
  float numerator = (mu_sangatJernih * 0.0f) + // Jika sangat jernih, pompa mati
                    (mu_jernih * 15.0f) +      // Jika jernih, pompa pelan
                    (mu_sesuai * 30.0f) +      // Jika sesuai, pompa sedang (untuk menjaga sirkulasi)
                    (mu_keruh * 60.0f) +       // Jika keruh, pompa cepat
                    (mu_sangatKeruh * 85.0f);  // Jika sangat keruh, pompa maks

  float denominator = mu_sangatJernih + mu_jernih + mu_sesuai + mu_keruh + mu_sangatKeruh;

  if (denominator < 0.01f)
  {
    // Jika semua membership = 0 (kemungkinan kecil, tapi aman)
    // Kembalikan nilai default, misalnya nilai saat error nol (sesuai)
    return 30.0f; // Nilai default saat di setpoint
  }

  return numerator / denominator; // Nilai output fuzzy akhir (0.0 - 85.0)
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
  Serial.println("\n[WiFi] Connecting...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40)
  {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\n[WiFi] Connected! IP: " + WiFi.localIP().toString());
  }
  else
  {
    Serial.println("\n[WiFi] FAILED! Restarting...");
    ESP.restart();
  }
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.println("[DEBUG] ESP32 received MQTT message:");
  Serial.println((char *)payload);
  if (strcmp(topic, MQTT_TOPIC_MODE) != 0)
    return;

  StaticJsonDocument<400> doc;
  char payloadStr[length + 1];
  memcpy(payloadStr, payload, length);
  payloadStr[length] = '\0';

  if (deserializeJson(doc, payloadStr))
    return;

  // Control Mode
  if (doc.containsKey("kontrol_aktif"))
  {
    String mode = doc["kontrol_aktif"].as<String>();
    if (mode == "Fuzzy")
    {
      kontrolAktif = FUZZY;
      resetPIDSuhu();
      resetPIDKeruh();
    }
    else if (mode == "PID")
    {
      kontrolAktif = PID;
    }
    Serial.println("[DEBUG] Mode changed to: " + mode);
  }

  // Setpoints
  if (doc.containsKey("suhu_setpoint"))
    suhuSetpoint = doc["suhu_setpoint"];
  if (doc.containsKey("keruh_setpoint"))
    turbiditySetpoint = doc["keruh_setpoint"];

  // PID Parameters
  if (doc.containsKey("kp_suhu"))
    Kp_suhu = doc["kp_suhu"];
  if (doc.containsKey("ki_suhu"))
    Ki_suhu = doc["ki_suhu"];
  if (doc.containsKey("kd_suhu"))
    Kd_suhu = doc["kd_suhu"];
  if (doc.containsKey("kp_keruh"))
    Kp_keruh = doc["kp_keruh"];
  if (doc.containsKey("ki_keruh"))
    Ki_keruh = doc["ki_keruh"];
  if (doc.containsKey("kd_keruh"))
    Kd_keruh = doc["kd_keruh"];

  Serial.println("[DEBUG] ESP32 finished processing MQTT message");
  Serial.println("[DEBUG] Suhu setpoint updated: " + String(suhuSetpoint));
  Serial.println("[DEBUG] Keruh setpoint updated: " + String(turbiditySetpoint));
  Serial.println("[DEBUG] After update: kontrolAktif = " + String(kontrolAktif));
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
  if (tempC == -127.00f || isnan(tempC))
    return suhuTerakhir;
  suhuTerakhir = tempC;
  return tempC;
}

int bacaTurbidity()
{
  int16_t adcValue = ads.readADC_SingleEnded(0);
  if (adcValue < 0 || adcValue > 32767)
    return turbidityTerakhir;
  turbidityTerakhir = adcValue;
  return adcValue;
}

float konversiTurbidityKePersen(int adcValue)
{
  float persen;
  if (NILAI_ADC_JERNIH > NILAI_ADC_KERUH)
  {
    persen = map(adcValue, NILAI_ADC_KERUH, NILAI_ADC_JERNIH, 100, 0);
  }
  else
  {
    persen = map(adcValue, NILAI_ADC_JERNIH, NILAI_ADC_KERUH, 0, 100);
  }
  return constrain(persen, 0.0f, 100.0f);
}

// =========================================================================
//                MQTT PUBLISH
// =========================================================================
void kirimDataMQTT(float suhu, float turbPersen, double pwmSuhu, double pwmKeruh,
                   float errSuhu, float errKeruh)
{
  StaticJsonDocument<512> doc;

  // Basic Data
  doc["timestamp_ms"] = millis();
  doc["suhu"] = round(suhu * 100) / 100.0;
  doc["turbidity_persen"] = round(turbPersen * 100) / 100.0;
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
//                SETUP
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
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(callback);
  mqttClient.setKeepAlive(60);

  Serial.println("=== System Ready for Research ===\n");
}

// =========================================================================
//                MAIN LOOP
// =========================================================================
void loop()
{
  if (!mqttClient.connected())
  {
    reconnect_mqtt();
  }
  mqttClient.loop();

  unsigned long now = millis();

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
                    errorSuhu, errorKeruh);
    }

    // Debug Print
    Serial.printf("[%lu] T:%.2f/%.1f E:%.2f PWM:%d | K:%.1f/%.1f E:%.1f PWM:%d\n",
                  millis() / 1000, suhuAktual, suhuSetpoint, errorSuhu, pwmSuhu,
                  turbidityPersen, turbiditySetpoint, errorKeruh, pwmKeruh);
  }
}