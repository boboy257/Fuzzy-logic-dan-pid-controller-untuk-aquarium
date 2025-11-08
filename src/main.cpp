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
// const char *MQTT_TOPIC_METRICS = ...; // DIHAPUS
const char *MQTT_CLIENT_ID = "esp32-research-aquarium";

// Pin Configuration
const int SENSOR_SUHU_PIN = 4;
const int HEATER_PIN = 16;
const int FILTER_PUMP_PIN = 17;

// Control Modes
enum ControlMode
{
  FUZZY,
  PID
};
ControlMode kontrolAktif = FUZZY;

// Experiment Control
bool experimentRunning = false;
unsigned long experimentStartTime = 0;
unsigned long experimentDuration = 600000; // 10 minutes default
String experimentID = "";

// Setpoints
float suhuSetpoint = 28.0f;
float turbiditySetpoint = 10.0f;

// PID Parameters - Temperature
double Kp_suhu = 25.0;
double Ki_suhu = 1.5;
double Kd_suhu = 4.0;
double integralSumSuhu = 0.0;
double lastErrorSuhu = 0.0;

// PID Parameters - Turbidity
double Kp_keruh = 10.0;
double Ki_keruh = 0.5;
double Kd_keruh = 1.0;
double integralSumKeruh = 0.0;
double lastErrorKeruh = 0.0;

// --- SEMUA VARIABEL PERFORMANCE METRICS DIHAPUS ---

// Sensor Calibration
const int NILAI_ADC_JERNIH = 9475;
const int NILAI_ADC_KERUH = 3550;

// Timing
unsigned long lastTimeSuhu = 0;
unsigned long lastTimeKeruh = 0;
unsigned long waktuTerakhirKirim = 0;
// unsigned long waktuTerakhirMetrics = 0; // DIHAPUS
const long intervalKirim = 1000; // 1 second for research
// const long intervalMetrics = 5000; // DIHAPUS

// PWM Configuration
const int PWM_CHANNEL_SUHU = 0;
const int PWM_CHANNEL_KERUH = 1;
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
//                FUZZY LOGIC - TEMPERATURE
// =========================================================================
float membershipPanasSuhu(float error)
{
  if (error <= -8.0f || error >= -2.0f)
    return 0.0f;
  if (error >= -6.0f && error <= -4.0f)
    return 1.0f;
  if (error > -8.0f && error < -6.0f)
    return (error + 8.0f) / 2.0f;
  if (error > -4.0f && error < -2.0f)
    return (error + 2.0f) / -2.0f;
  return 0.0f;
}

float membershipSesuaiSuhu(float error)
{
  if (error <= -2.0f || error >= 2.0f)
    return 0.0f;
  if (error > -2.0f && error <= 0.0f)
    return (error + 2.0f) / 2.0f;
  if (error > 0.0f && error < 2.0f)
    return (2.0f - error) / 2.0f;
  return 0.0f;
}

float membershipDinginSuhu(float error)
{
  if (error <= 2.0f || error >= 8.0f)
    return 0.0f;
  if (error >= 4.0f && error <= 6.0f)
    return 1.0f;
  if (error > 2.0f && error < 4.0f)
    return (error - 2.0f) / 2.0f;
  if (error > 6.0f && error < 8.0f)
    return (8.0f - error) / 2.0f;
  return 0.0f;
}

float hitungFuzzySuhu(float errorSuhu)
{
  float nilaiPanas = membershipPanasSuhu(errorSuhu);
  float nilaiSesuai = membershipSesuaiSuhu(errorSuhu);
  float nilaiDingin = membershipDinginSuhu(errorSuhu);

  float kekuatanTinggi = nilaiDingin;
  float kekuatanRendah = max(nilaiSesuai, nilaiPanas);

  float numerator = (kekuatanRendah * 10.0f) + (kekuatanTinggi * 90.0f);
  float denominator = kekuatanRendah + kekuatanTinggi;

  return (denominator > 0.0f) ? (numerator / denominator) : 0.0f;
}

// =========================================================================
//                FUZZY LOGIC - TURBIDITY
// =========================================================================
float membershipJernihKeruh(float error)
{
  if (error <= -20.0f || error >= -5.0f)
    return 0.0f;
  if (error >= -15.0f && error <= -10.0f)
    return 1.0f;
  if (error > -20.0f && error < -15.0f)
    return (error + 20.0f) / 5.0f;
  if (error > -10.0f && error < -5.0f)
    return (error + 5.0f) / -5.0f;
  return 0.0f;
}

float membershipSesuaiKeruh(float error)
{
  if (error <= -5.0f || error >= 5.0f)
    return 0.0f;
  if (error > -5.0f && error <= 0.0f)
    return (error + 5.0f) / 5.0f;
  if (error > 0.0f && error < 5.0f)
    return (5.0f - error) / 5.0f;
  return 0.0f;
}

float membershipKeruh(float error)
{
  if (error <= 5.0f || error >= 40.0f)
    return 0.0f;
  if (error >= 10.0f && error <= 20.0f)
    return 1.0f;
  if (error > 5.0f && error < 10.0f)
    return (error - 5.0f) / 5.0f;
  if (error > 20.0f && error < 40.0f)
    return (40.0f - error) / 20.0f;
  return 0.0f;
}

float hitungFuzzyKeruh(float errorKeruh)
{
  float nilaiJernih = membershipJernihKeruh(errorKeruh);
  float nilaiSesuai = membershipSesuaiKeruh(errorKeruh);
  float nilaiKeruh = membershipKeruh(errorKeruh);

  float kekuatanTinggi = nilaiKeruh;
  float kekuatanRendah = max(nilaiSesuai, nilaiJernih);

  float numerator = (kekuatanRendah * 10.0f) + (kekuatanTinggi * 90.0f);
  float denominator = kekuatanRendah + kekuatanTinggi;

  return (denominator > 0.0f) ? (numerator / denominator) : 0.0f;
}

// =========================================================================
//                PID CONTROL - TEMPERATURE
// =========================================================================
double hitungPIDSuhu(float errorSuhu)
{
  unsigned long now = millis();
  double elapsedTime = (double)(now - lastTimeSuhu);
  if (elapsedTime < 1)
    elapsedTime = 1;

  integralSumSuhu += errorSuhu * elapsedTime / 1000.0;
  integralSumSuhu = constrain(integralSumSuhu, -100.0, 100.0); // Anti-windup

  double derivative = (errorSuhu - lastErrorSuhu) / (elapsedTime / 1000.0);
  double output = Kp_suhu * errorSuhu + Ki_suhu * integralSumSuhu + Kd_suhu * derivative;

  lastErrorSuhu = errorSuhu;
  lastTimeSuhu = now;

  return constrain(output, 0.0, 100.0);
}

void resetPIDSuhu()
{
  integralSumSuhu = 0.0;
  lastErrorSuhu = 0.0;
}

// =========================================================================
//                PID CONTROL - TURBIDITY
// =========================================================================
double hitungPIDKeruh(float errorKeruh)
{
  unsigned long now = millis();
  double elapsedTime = (double)(now - lastTimeKeruh);
  if (elapsedTime < 1)
    elapsedTime = 1;

  integralSumKeruh += errorKeruh * elapsedTime / 1000.0;
  integralSumKeruh = constrain(integralSumKeruh, -100.0, 100.0); // Anti-windup

  double derivative = (errorKeruh - lastErrorKeruh) / (elapsedTime / 1000.0);
  double output = Kp_keruh * errorKeruh + Ki_keruh * integralSumKeruh + Kd_keruh * derivative;

  lastErrorKeruh = errorKeruh;
  lastTimeKeruh = now;

  return constrain(output, 0.0, 100.0);
}

void resetPIDKeruh()
{
  integralSumKeruh = 0.0;
  lastErrorKeruh = 0.0;
}

// =========================================================================
//                PERFORMANCE METRICS CALCULATION
// =========================================================================
// --- SELURUH BLOK FUNGSI updateTempMetrics(), updateTurbMetrics(), ---
// --- dan resetMetrics() DIHAPUS ---

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
    Serial.println("[DEBUG] Mode changed to: " + mode); // Tambahkan log
  }

  // Experiment Control
  if (doc.containsKey("experiment_start") && doc["experiment_start"])
  {
    experimentRunning = true;
    experimentStartTime = millis();
    experimentID = doc.containsKey("experiment_id") ? doc["experiment_id"].as<String>() : String(millis());
    experimentDuration = doc.containsKey("duration") ? doc["duration"].as<unsigned long>() : 600000;
    // resetMetrics(); // DIHAPUS
    Serial.println("[EXP] Started: " + experimentID);
  }

  if (doc.containsKey("experiment_stop") && doc["experiment_stop"])
  {
    experimentRunning = false;
    Serial.println("[EXP] Stopped: " + experimentID);
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

  // TAMBAHKAN LOG INI DI AKHIR FUNGSI
  Serial.println("[DEBUG] ESP32 finished processing MQTT message");
  Serial.println("[DEBUG] Suhu setpoint updated: " + String(suhuSetpoint));       // Tambahkan log
  Serial.println("[DEBUG] Keruh setpoint updated: " + String(turbiditySetpoint)); // Tambahkan log
  Serial.println("[DEBUG] Kp suhu updated: " + String(Kp_suhu));                  // Tambahkan log
  Serial.println("[DEBUG] Ki suhu updated: " + String(Ki_suhu));                  // Tambahkan log
  Serial.println("[DEBUG] Kd suhu updated: " + String(Kd_suhu));                  // Tambahkan log
  Serial.println("[DEBUG] Kp keruh updated: " + String(Kp_keruh));                // Tambahkan log
  Serial.println("[DEBUG] Ki keruh updated: " + String(Ki_keruh));                // Tambahkan log
  Serial.println("[DEBUG] Kd keruh updated: " + String(Kd_keruh));                // Tambahkan log

  Serial.println("[DEBUG] After update: kontrolAktif = " + String(kontrolAktif));
  Serial.println("[DEBUG] After update: suhuSetpoint = " + String(suhuSetpoint));
  Serial.println("[DEBUG] After update: turbiditySetpoint = " + String(turbiditySetpoint));
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

  // Experiment Info
  doc["experiment_running"] = experimentRunning;
  if (experimentRunning)
  {
    doc["experiment_id"] = experimentID;
    doc["experiment_elapsed_s"] = (millis() - experimentStartTime) / 1000;
  }

  char buffer[512];
  serializeJson(doc, buffer);
  // Serial.println(buffer); // <-- TAMBAHKAN LOG INI UNTUK MELIHAT ISI DATA
  mqttClient.publish(MQTT_TOPIC_DATA, buffer, false);
}

// --- FUNGSI kirimMetricsMQTT() DIHAPUS ---

// =========================================================================
//                SETUP
// =========================================================================
void setup()
{
  Serial.begin(115200);
  Serial.println("\n=== ESP32 Research Control System ===");

  Wire.begin();
  if (!ads.begin())
  {
    Serial.println("[ERROR] ADS1115 not found!");
    while (1)
      delay(1000);
  }

  ledcSetup(PWM_CHANNEL_SUHU, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(HEATER_PIN, PWM_CHANNEL_SUHU);
  ledcSetup(PWM_CHANNEL_KERUH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(FILTER_PUMP_PIN, PWM_CHANNEL_KERUH);

  sensors.begin();

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

  // Check experiment timeout
  if (experimentRunning &&
      (millis() - experimentStartTime) > experimentDuration)
  {
    experimentRunning = false;
    Serial.println("[EXP] Timeout - Auto stopped");
  }

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
    float errorKeruh = turbidityPersen - turbiditySetpoint; // DIBALIK: error = aktual - setpoint

    // Control Outputs
    double dayaOutputSuhu = (kontrolAktif == FUZZY) ? hitungFuzzySuhu(errorSuhu) : hitungPIDSuhu(errorSuhu);
    double dayaOutputKeruh = (kontrolAktif == FUZZY) ? hitungFuzzyKeruh(errorKeruh) : hitungPIDKeruh(errorKeruh); // Gunakan errorKeruh

    int pwmSuhu = constrain((int)(dayaOutputSuhu * 2.55), 0, 255);
    int pwmKeruh = constrain((int)(dayaOutputKeruh * 2.55), 0, 255);

    ledcWrite(PWM_CHANNEL_SUHU, pwmSuhu);
    ledcWrite(PWM_CHANNEL_KERUH, pwmKeruh);

    // Update Metrics
    // updateTempMetrics(suhuAktual);     // DIHAPUS
    // updateTurbMetrics(turbidityPersen); // DIHAPUS

    // Send Data
    if (mqttClient.connected())
    {
      // Serial.println("[DEBUG] Sending data to MQTT:"); // <-- TAMBAHKAN LOG INI
      kirimDataMQTT(suhuAktual, turbidityPersen, dayaOutputSuhu, dayaOutputKeruh,
                    errorSuhu, errorKeruh);
    }

    // Debug Print
    Serial.printf("[%lu] T:%.2f/%.1f E:%.2f PWM:%d | K:%.1f/%.1f E:%.1f PWM:%d\n",
                  millis() / 1000, suhuAktual, suhuSetpoint, errorSuhu, pwmSuhu,
                  turbidityPersen, turbiditySetpoint, errorKeruh, pwmKeruh);
  }

  // Metrics Publishing Loop
  // --- BLOK 'if (now - waktuTerakhirMetrics >= intervalMetrics)' DIHAPUS ---
}