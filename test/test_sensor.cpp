#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h> // 🔹 Tambahkan ini

// =========================================================================
//                       KONFIGURASI WAJIB
// =========================================================================
const char *ssid = "iPhone 2";
const char *password = "bobo2002";

const char *mqtt_broker = "broker.hivemq.com";
const int mqtt_port = 1883;
const char *mqtt_topic_data = "unhas/informatika/aquarium/data";
const char *mqtt_topic_mode = "unhas/informatika/aquarium/mode";

String kontrolAktif = "Fuzzy"; // Pilihan: "Fuzzy" atau "PID"

const int SENSOR_SUHU_PIN = 4;
const int HEATER_PIN = 16;
float suhuSetpoint = 28.0;

// --- PENGATURAN PID ---
double Kp = 25;
double Ki = 1.5;
double Kd = 4;
// =========================================================================

// Inisialisasi library & variabel global
WiFiClient espClient;
PubSubClient client(espClient);
OneWire oneWire(SENSOR_SUHU_PIN);
DallasTemperature sensors(&oneWire);

// 🔹 Inisialisasi ADS1115
Adafruit_ADS1115 ads; // Default address 0x48

float suhuTerakhir = 25.0;
int turbidityTerakhir = 0;
float turbidityPersenTerakhir = 0.0;
unsigned long waktuTerakhirKirim = 0;
const long intervalKirim = 5000;

// Variabel untuk PID
double integralSum = 0;
double lastError = 0;
unsigned long lastTime = 0;

// Variabel untuk PWM
const int PWM_CHANNEL = 0;
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;

bool debugMode = true;

// =========================================================================
//            FUNGSI-FUNGSI UNTUK FUZZY LOGIC
// =========================================================================
float membershipPanas(float error)
{
  if (error <= -8 || error >= -2)
    return 0;
  if (error >= -6 && error <= -4)
    return 1;
  if (error > -8 && error < -6)
    return (error - (-8)) / (-6 - (-8));
  if (error > -4 && error < -2)
    return (-2 - error) / (-2 - (-4));
  return 0;
}

float membershipSesuai(float error)
{
  if (error <= -2 || error >= 2)
    return 0;
  if (error > -2 && error <= 0)
    return (error - (-2)) / (0 - (-2));
  if (error > 0 && error < 2)
    return (2 - error) / (2 - 0);
  return 0;
}

float membershipDingin(float error)
{
  if (error <= 2 || error >= 8)
    return 0;
  if (error >= 4 && error <= 6)
    return 1;
  if (error > 2 && error < 4)
    return (error - 2) / (4 - 2);
  if (error > 6 && error < 8)
    return (8 - error) / (8 - 6);
  return 0;
}

float hitungFuzzy(float error)
{
  float nilaiPanas = membershipPanas(error);
  float nilaiSesuai = membershipSesuai(error);
  float nilaiDingin = membershipDingin(error);

  float kekuatanTinggi = nilaiDingin;
  float kekuatanRendah = max(nilaiSesuai, nilaiPanas);
  float kekuatanSedang = 0;

  float numerator = (kekuatanRendah * 10) + (kekuatanSedang * 50) + (kekuatanTinggi * 90);
  float denominator = kekuatanRendah + kekuatanSedang + kekuatanTinggi;

  if (denominator > 0)
  {
    return numerator / denominator;
  }
  return 0;
}

// =========================================================================
//                         FUNGSI UNTUK PID
// =========================================================================
double hitungPID(float error)
{
  unsigned long now = millis();
  double elapsedTime = (double)(now - lastTime);
  if (elapsedTime < 1)
    elapsedTime = 1;

  integralSum += error * elapsedTime / 1000.0;
  double derivative = (error - lastError) / (elapsedTime / 1000.0);

  double output = Kp * error + Ki * integralSum + Kd * derivative;

  lastError = error;
  lastTime = now;

  if (output > 100)
    output = 100;
  if (output < 0)
    output = 0;

  return output;
}

void resetPID()
{
  integralSum = 0;
  lastError = 0;
}

// =========================================================================
//                         FUNGSI KOMUNIKASI
// =========================================================================
void setup_wifi()
{
  delay(10);
  Serial.println();
  Serial.print("Menghubungkan ke WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi terhubung!");
  Serial.print("Alamat IP: ");
  Serial.println(WiFi.localIP());
}

void callback(char *topic, byte *payload, unsigned int length)
{
  if (strcmp(topic, mqtt_topic_mode) == 0)
  {
    // Gunakan ArduinoJson untuk parse payload JSON
    StaticJsonDocument<250> doc;
    char payloadStr[length + 1];
    strncpy(payloadStr, (char *)payload, length);
    payloadStr[length] = '\0';

    DeserializationError error = deserializeJson(doc, payloadStr);
    if (error)
    {
      Serial.print("Gagal parse JSON dari MQTT: ");
      Serial.println(error.c_str());
      return;
    }

    // Baca parameter dari payload
    if (doc.containsKey("kontrol_aktif"))
    {
      String newMode = doc["kontrol_aktif"].as<String>();
      if (newMode == "Fuzzy" || newMode == "PID")
      {
        kontrolAktif = newMode;
        resetPID(); // Reset PID jika mode berubah
        Serial.print("Mode kontrol diubah ke: ");
        Serial.println(kontrolAktif);
      }
    }

    // Baca dan perbarui setpoint
    if (doc.containsKey("suhu_setpoint"))
    {
      suhuSetpoint = doc["suhu_setpoint"];
      Serial.print("Setpoint suhu diubah ke: ");
      Serial.println(suhuSetpoint);
    }

    // Baca dan perbarui Kp, Ki, Kd
    if (doc.containsKey("kp"))
    {
      Kp = doc["kp"];
      Serial.print("Kp diubah ke: ");
      Serial.println(Kp);
    }
    if (doc.containsKey("ki"))
    {
      Ki = doc["ki"];
      Serial.print("Ki diubah ke: ");
      Serial.println(Ki);
    }
    if (doc.containsKey("kd"))
    {
      Kd = doc["kd"];
      Serial.print("Kd diubah ke: ");
      Serial.println(Kd);
    }
  }
}

void reconnect_mqtt()
{
  while (!client.connected())
  {
    Serial.print("Mencoba koneksi MQTT...");
    if (client.connect("esp32-aquarium-client"))
    {
      Serial.println("terhubung!");
      client.subscribe(mqtt_topic_mode);
      Serial.println("Subscribe ke: " + String(mqtt_topic_mode));
    }
    else
    {
      Serial.print("gagal, rc=");
      Serial.print(client.state());
      Serial.println(" coba lagi dalam 5 detik");
      delay(5000);
    }
  }
}

float bacaSuhuDS18B20()
{
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  if (tempC == -127.00 || isnan(tempC))
  {
    Serial.println("Gagal membaca sensor suhu!");
    return suhuTerakhir;
  }
  else
  {
    suhuTerakhir = tempC;
    return tempC;
  }
}

// 🔹 Fungsi Baca Turbidity via ADS1115
float bacaTurbidityPersen()
{
  int16_t adcValue = ads.readADC_SingleEnded(0);
  if (adcValue < 0)
    adcValue = 0;
  if (adcValue > 32767)
    adcValue = 32767;

  float kekeruhanPersen = (1.0 - (static_cast<float>(adcValue) / 32767.0)) * 100.0;
  if (kekeruhanPersen < 0.0)
    kekeruhanPersen = 0.0;
  if (kekeruhanPersen > 100.0)
    kekeruhanPersen = 100.0;

  turbidityTerakhir = adcValue;
  turbidityPersenTerakhir = kekeruhanPersen;
  return kekeruhanPersen;
}

void kirimDataMQTT(float suhu, float turbidityPersen, double dayaOutput)
{
  StaticJsonDocument<250> doc;
  doc["suhu"] = suhu;
  doc["turbidity"] = turbidityPersen;
  doc["kontrol_aktif"] = kontrolAktif;
  doc["pwm_output"] = dayaOutput; //(int)(dayaOutput * 2.55); untuk konversi ke 8 bit

  char buffer[250];
  serializeJson(doc, buffer);

  client.publish(mqtt_topic_data, buffer);
  if (debugMode)
  {
    Serial.print("Data dikirim ke MQTT: ");
    Serial.println(buffer);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("=== Memulai Sistem Aquarium dengan ADS1115 ===");

  // 🔹 Inisialisasi I2C dan ADS1115
  Wire.begin();
  if (!ads.begin())
  {
    Serial.println("Gagal menginisialisasi ADS1115!");
    while (1)
      ;
  }
  Serial.println("ADS1115 terdeteksi di alamat 0x48");

  // Setup PWM
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(HEATER_PIN, PWM_CHANNEL);

  sensors.begin();
  lastTime = millis();

  setup_wifi();
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
}

void loop()
{
  if (!client.connected())
  {
    reconnect_mqtt();
  }
  client.loop();

  unsigned long now = millis();
  if (now - waktuTerakhirKirim > intervalKirim)
  {
    waktuTerakhirKirim = now;

    float suhuAktual = bacaSuhuDS18B20();
    float turbidityPersenAktual = bacaTurbidityPersen();
    float error = suhuSetpoint - suhuAktual;

    double dayaOutput = 0;

    if (kontrolAktif == "Fuzzy")
    {
      dayaOutput = hitungFuzzy(error);
      if (debugMode)
        Serial.println("Mode: Fuzzy");
    }
    else if (kontrolAktif == "PID")
    {
      dayaOutput = hitungPID(error);
      if (debugMode)
        Serial.println("Mode: PID");
    }

    int pwmValue = (int)(dayaOutput * 2.55); // <-- Konversi hanya untuk PWM
    ledcWrite(PWM_CHANNEL, pwmValue);

    if (debugMode)
    {
      Serial.printf("Suhu: %.2f°C | Turbidity: %.2f%% | PWM: %d\n", suhuAktual, turbidityPersenAktual, pwmValue);
    }

    kirimDataMQTT(suhuAktual, turbidityPersenAktual, dayaOutput);
  }
}