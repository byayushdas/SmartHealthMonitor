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
// WiFi & MQTT Broker Configuration
const char* ssid = "WifiSSID";
const char* password = "WifiPassword";
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
int ecgValue = analogRead(ECG_PIN);
float temperature = bmp.readTemperature();
float pressure = bmp.readPressure() / 100.0;
// Get GPS values (only if available)
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
Serial.print(" %, ECG: "); Serial.print(ecgValue);
Serial.print(", Temp: "); Serial.print(temperature); Serial.print("
°C");
Serial.print(", Pressure: "); Serial.print(pressure); Serial.print("
hPa");
Serial.print(", Acc(X,Y,Z): "); Serial.print(accX); Serial.print(",
"); Serial.print(accY); Serial.print(", "); Serial.print(accZ);
Serial.print(", Gyro(X,Y,Z): "); Serial.print(gyroX); Serial.print(",
"); Serial.print(gyroY); Serial.print(", "); Serial.println(gyroZ);
Serial.println("---------------------------------------------------");
// Publish All Data (including lat/lon) to MQTT
String payload = "{";
payload += "\"latitude\": " + String(latitude, 6) + ", ";
payload += "\"longitude\": " + String(longitude, 6) + ", ";
payload += "\"heartRate\": " + String(heartRate) + ", ";
payload += "\"spo2\": " + String(spo2) + ", ";
payload += "\"ecg\": " + String(ecgValue) + ", ";
payload += "\"temperature\": " + String(temperature) + ", ";
payload += "\"pressure\": " + String(pressure) + ", ";
payload += "\"acceleration\": {\"x\": " + String(accX, 2) + ", \"y\":
" + String(accY, 2) + ", \"z\": " + String(accZ, 2) + "}, ";
payload += "\"gyroscope\": {\"x\": " + String(gyroX, 2) + ", \"y\": "
+ String(gyroY, 2) + ", \"z\": " + String(gyroZ, 2) + "}";
payload += "}";
client.publish("sensordata", payload.c_str());
delay(1000);
}