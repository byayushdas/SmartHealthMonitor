#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h> // BMP180 library
#include "MAX30100_PulseOximeter.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// GPS Setup
TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // UART2
#define RXD2 16 // GPS TX → ESP32 GPIO16 (RX)
#define TXD2 17 // GPS RX → ESP32 GPIO17 (TX)

// UV Sensor Pin
#define UV_SENSOR_PIN 35

// WiFi & MQTT Broker Configuration
const char* ssid = "ssid ";
const char* password = "password";
const char* mqtt_server = "test.mosquitto.org";
WiFiClient espClient;
PubSubClient client(espClient);

Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;
PulseOximeter pox;
#define ECG_PIN 34

void onBeatDetected() {
  Serial.println("Beat Detected!");
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  gpsSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // WiFi Connect
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");

  // MQTT Connect
  client.setServer(mqtt_server, 1883);
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP32_Client")) {
      Serial.println(" connected!");
    } else {
      Serial.print(" failed, retry in 5 sec...");
      delay(5000);
    }
  }

  // Sensor Init
  if (!mpu.begin()) {
    Serial.println("MPU6050 init failed!"); while (1);
  }
  if (!pox.begin()) {
    Serial.println("MAX30100 init failed!"); while (1);
  }
  pox.setOnBeatDetectedCallback(onBeatDetected);
  if (!bmp.begin()) {
    Serial.println("BMP180 init failed!"); while (1);
  }
}

void loop() {
  // Update GPS
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // Read Sensors
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float accX = a.acceleration.x;
  float accY = a.acceleration.y;
  float accZ = a.acceleration.z;
  float gyroX = g.gyro.x;
  float gyroY = g.gyro.y;
  float gyroZ = g.gyro.z;

  pox.update();
  float heartRate = pox.getHeartRate();
  float spo2 = pox.getSpO2();
  int ecgRaw = analogRead(ECG_PIN);

  // Convert ECG reading to mV
  float ecgVoltage_mV = (ecgRaw / 4095.0) * 3300.0; // Convert to mV
  float ecgReal_mV = ((ecgVoltage_mV - 1500.0) / 1000.0)*1.9; // Subtract offset and divide by gain
if (ecgReal_mV < 0) {
  ecgReal_mV *= -1;
}


  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0;

  // UV Sensor Readings
  int uvRaw = analogRead(UV_SENSOR_PIN);
  float uvVoltage = (uvRaw / 4095.0) * 3.3; // For 12-bit ADC, 3.3V reference
  float uvIndex = uvVoltage / 0.1; // Approximate conversion

  // Get GPS values
  float latitude = 0.0;
  float longitude = 0.0;
  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
  }

  // Display on Serial Monitor
  Serial.print("Latitude: "); Serial.print(latitude, 6);
  Serial.print(", Longitude: "); Serial.println(longitude, 6);
  Serial.print("HR: "); Serial.print(heartRate);
  Serial.print(" bpm, SpO2: "); Serial.print(spo2);
  Serial.print(" %, ECG: "); Serial.print(ecgReal_mV);
  Serial.print(" mV, Temp: "); Serial.print(temperature); Serial.print("°C");
  Serial.print(", Pressure: "); Serial.print(pressure); Serial.print("hPa");
  Serial.print(", Acc(X,Y,Z): "); Serial.print(accX); Serial.print(","); Serial.print(accY); Serial.print(", "); Serial.print(accZ);
  Serial.print(", Gyro(X,Y,Z): "); Serial.print(gyroX); Serial.print(","); Serial.print(gyroY); Serial.print(", "); Serial.println(gyroZ);
  Serial.print("UV Voltage: "); Serial.print(uvVoltage, 2);
  Serial.print(" V, UV Index: "); Serial.println(uvIndex, 2);
  Serial.println("---------------------------------------------------");

  // Publish All Data to MQTT
  String payload = "{";
  payload += "\"latitude\": " + String(latitude, 6) + ", ";
  payload += "\"longitude\": " + String(longitude, 6) + ", ";
  payload += "\"heartRate\": " + String(heartRate) + ", ";
  payload += "\"spo2\": " + String(spo2) + ", ";
  payload += "\"ecg\": " + String(ecgReal_mV) + ", ";
  payload += "\"temperature\": " + String(temperature) + ", ";
  payload += "\"pressure\": " + String(pressure) + ", ";
  payload += "\"uvIndex\": " + String(uvIndex, 2) + ", ";
  payload += "\"acceleration\": {\"x\": " + String(accX, 2) + ", \"y\":" + String(accY, 2) + ", \"z\": " + String(accZ, 2) + "}, ";
  payload += "\"gyroscope\": {\"x\": " + String(gyroX, 2) + ", \"y\": " + String(gyroY, 2) + ", \"z\": " + String(gyroZ, 2) + "}";
  payload += "}";

client.publish("ahmm", payload.c_str());

  delay(1000);
}