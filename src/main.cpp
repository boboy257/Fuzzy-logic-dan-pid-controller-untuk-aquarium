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
//          IMPROVED FUZZY LOGIC - TEMPERATURE (5 MEMBERSHIP FUNCTIONS)
// =========================================================================

float membershipSangatDingin(float error)
{
  if (error <= 4.0f)
    return 0.0f;
  if (error >= 8.0f)
    return 1.0f;
  return (error - 4.0f) / 4.0f;
}

float membershipDingin(float error)
{
  if (error <= 1.0f || error >= 7.0f)
    return 0.0f;
  if (error >= 3.0f && error <= 5.0f)
    return 1.0f;
  if (error > 1.0f && error < 3.0f)
    return (error - 1.0f) / 2.0f;
  if (error > 5.0f && error < 7.0f)
    return (7.0f - error) / 2.0f;
  return 0.0f;
}

float membershipSesuai(float error)
{
  if (error <= -2.0f || error >= 2.0f)
    return 0.0f;
  if (error >= -0.5f && error <= 0.5f)
    return 1.0f;
  if (error > -2.0f && error < -0.5f)
    return (error + 2.0f) / 1.5f;
  if (error > 0.5f && error < 2.0f)
    return (2.0f - error) / 1.5f;
  return 0.0f;
}

float membershipPanas(float error)
{
  if (error <= -7.0f || error >= -1.0f)
    return 0.0f;
  if (error >= -5.0f && error <= -3.0f)
    return 1.0f;
  if (error > -7.0f && error < -5.0f)
    return (error + 7.0f) / 2.0f;
  if (error > -3.0f && error < -1.0f)
    return (-1.0f - error) / 2.0f;
  return 0.0f;
}

float membershipSangatPanas(float error)
{
  if (error >= -4.0f)
    return 0.0f;
  if (error <= -8.0f)
    return 1.0f;
  return (-4.0f - error) / 4.0f;
}

float hitungFuzzySuhu(float errorSuhu)
{
  float mu_sangatDingin = membershipSangatDingin(errorSuhu);
  float mu_dingin = membershipDingin(errorSuhu);
  float mu_sesuai = membershipSesuai(errorSuhu);
  float mu_panas = membershipPanas(errorSuhu);
  float mu_sangatPanas = membershipSangatPanas(errorSuhu);

  // Fuzzy Rules: 5 level output untuk kontrol yang lebih halus
  float numerator = (mu_sangatDingin * 85.0f) +
                    (mu_dingin * 60.0f) +
                    (mu_sesuai * 30.0f) +
                    (mu_panas * 10.0f) +
                    (mu_sangatPanas * 0.0f);

  float denominator = mu_sangatDingin + mu_dingin + mu_sesuai + mu_panas + mu_sangatPanas;

  if (denominator < 0.01f)
    return 30.0f;

  return numerator / denominator;
}

// =========================================================================
//          IMPROVED FUZZY LOGIC - TURBIDITY (5 MEMBERSHIP FUNCTIONS)
// =========================================================================

float membershipSangatJernih(float error)
{
  if (error >= -10.0f)
    return 0.0f;
  if (error <= -20.0f)
    return 1.0f;
  return (-10.0f - error) / 10.0f;
}

float membershipJernih(float error)
{
  if (error <= -20.0f || error >= 0.0f)
    return 0.0f;
  if (error >= -12.0f && error <= -8.0f)
    return 1.0f;
  if (error > -20.0f && error < -12.0f)
    return (error + 20.0f) / 8.0f;
  if (error > -8.0f && error < 0.0f)
    return (-error) / 8.0f;
  return 0.0f;
}

float membershipSesuaiKeruh(float error)
{
  if (error <= -8.0f || error >= 8.0f)
    return 0.0f;
  if (error >= -2.0f && error <= 2.0f)
    return 1.0f;
  if (error > -8.0f && error < -2.0f)
    return (error + 8.0f) / 6.0f;
  if (error > 2.0f && error < 8.0f)
    return (8.0f - error) / 6.0f;
  return 0.0f;
}

float membershipKeruh(float error)
{
  if (error <= 3.0f || error >= 30.0f)
    return 0.0f;
  if (error >= 10.0f && error <= 20.0f)
    return 1.0f;
  if (error > 3.0f && error < 10.0f)
    return (error - 3.0f) / 7.0f;
  if (error > 20.0f && error < 30.0f)
    return (30.0f - error) / 10.0f;
  return 0.0f;
}

float membershipSangatKeruh(float error)
{
  if (error <= 20.0f)
    return 0.0f;
  if (error >= 35.0f)
    return 1.0f;
  return (error - 20.0f) / 15.0f;
}

float hitungFuzzyKeruh(float errorKeruh)
{
  float mu_sangatJernih = membershipSangatJernih(errorKeruh);
  float mu_jernih = membershipJernih(errorKeruh);
  float mu_sesuai = membershipSesuaiKeruh(errorKeruh);
  float mu_keruh = membershipKeruh(errorKeruh);
  float mu_sangatKeruh = membershipSangatKeruh(errorKeruh);

  float numerator = (mu_sangatJernih * 0.0f) +
                    (mu_jernih * 15.0f) +
                    (mu_sesuai * 30.0f) +
                    (mu_keruh * 60.0f) +
                    (mu_sangatKeruh * 85.0f);

  float denominator = mu_sangatJernih + mu_jernih + mu_sesuai + mu_keruh + mu_sangatKeruh;

  if (denominator < 0.01f)
    return 30.0f;

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
  Serial.println("\n=== ESP32 Research Control System ===");

  Wire.begin();
  if (!ads.begin())
  {
    Serial.println("[ERROR] ADS1115 not found!");
    while (1)
      delay(1000);
  }
  Serial.println("[OK] ADS1115 initialized");

  // Initialize PWM
  ledcSetup(PWM_CHANNEL_SUHU, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(HEATER_PIN, PWM_CHANNEL_SUHU);
  ledcSetup(PWM_CHANNEL_KERUH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(FILTER_PUMP_PIN, PWM_CHANNEL_KERUH);
  Serial.println("[OK] PWM channels configured");

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

    ledcWrite(PWM_CHANNEL_SUHU, pwmSuhu);
    ledcWrite(PWM_CHANNEL_KERUH, pwmKeruh);

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