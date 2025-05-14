Hardware Interface:-<br /><br />
Modules/Sensors:
- NodeMCU ESP32 Devkit V1<br />
- MAX30100 (Heart Rate & Blood Oxygen Level)<br />
- MPU6050 (Gyroscope)<br />
- AD8232 (ECG)<br />
- BMP180 (Pressure & Temperature)<br />
- CJMCU-GUVA-S12SD (UV Voltage & UV Index)<br />
- GY-GPSV3 NEO-M8N (GPS)<br /><br />
Upload the file "esp32.ino" in the ESP32 using Arduino IDE.<br />
Import the file "node-red-flow.json" to the Node-Red and Deploy it.<br /><br />
- The Data from the Sensors is received from MQTT Mosquitto Public Server and saved in a CSV File.<br />
- The AI Prediction Model is based on the Data of the CSV File.<br />
