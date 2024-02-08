// Include Libraries
#include <Wire.h>
#include <WiFi.h>
#include <Arduino.h>
#include <HTTPClient.h>
#include "Credentials.h"
#include "gps_pin.h"
#include <ArduinoJson.h>
#include <TinyGPS++.h>

// Serial Communication for sensor, SIM and GPS
#ifndef SerialAT
#define SerialAT Serial
#endif

#ifndef SerialGPS
#define SerialGPS Serial2
#endif

#ifndef sensor
#define sensor Serial1
#endif


const char *server = "45.61.55.203";                  // Replace with your server IP address
const int serverPort = 9090;                         // Replace with your server port
const String access_token = "XAjieqa2LNFTIvyasBki";  // Replace with access token


// Declare global variables to hold sensor data
unsigned int soilHumidity = 0;
unsigned int soilTemperature = 0;
unsigned int soilConductivity = 0;
unsigned int soilPH = 0;
unsigned int nitrogen = 0;
unsigned int phosphorus = 0;
unsigned int potassium = 0;

TinyGPSPlus gps;

// Global variables for latitude and longitude
double latitude = 0.0;
double longitude = 0.0;
double altitude = 0.0;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  // Initializing sensor
  sensor.begin(4800, SERIAL_8N1, 32, 33);  // Start hardware serial at 4800 bps

  //Initializing GPS
  initializeGPS();

  // Connecting to WiFi
  connectToWiFi();
}

void loop() {

  readSensorData();
  // Print Sensor Value
  Serial.print("Soil Humidity: ");
  Serial.print((float)soilHumidity / 10.0);
  Serial.print(" %    ");
  Serial.print("Soil Temperature: ");
  Serial.print((float)soilTemperature / 10.0);
  Serial.print(" °C    ");
  Serial.print("Soil Conductivity: ");
  Serial.print((float)soilConductivity/100.0);
  Serial.print(" dS/m    ");  // microsiemens per centimeter (µS/cm)
  Serial.print("Soil pH: ");
  Serial.print((float)soilPH / 10.0);
  Serial.print("   ");
  Serial.print("Nitrogen: ");
  Serial.print(nitrogen);
  Serial.print(" mg/kg    ");
  Serial.print("Phosphorus: ");
  Serial.print(phosphorus);
  Serial.print(" mg/kg    ");
  Serial.print("Potassium: ");
  Serial.print(potassium);
  Serial.println(" mg/kg");

  //Read GPS data
  if (readGPSData(latitude, longitude, altitude)) {
    // Do something with latitude, longitude, and altitude here
    Serial.print("Latitude: ");
    Serial.print(latitude, 10);
    Serial.print("  Longitude: ");
    Serial.print(longitude, 10);
    Serial.print("  Altitude: ");
    Serial.println(altitude, 2);
  }

  // POST the sensor data via HTTP
  sendSensorDataAttributes(soilTemperature, soilHumidity, soilConductivity, soilPH, nitrogen, phosphorus, potassium, latitude, longitude, altitude);
  sendSensorDataTelemetry(soilTemperature, soilHumidity, soilConductivity, soilPH, nitrogen, phosphorus, potassium, latitude, longitude, altitude);

  // delay(10000);  // Delay between readings
}

void connectToWiFi() {
  WiFi.begin(mySSID, myPASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
}

void readSensorData() {
  byte queryData[]{ 0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08 };
  byte receivedData[19];

  sensor.write(queryData, sizeof(queryData));  // Send query data to NPK
  delay(5000);

  if (sensor.available() >= sizeof(receivedData)) {        // Check if there is enough bytes available to read
    sensor.readBytes(receivedData, sizeof(receivedData));  // Read the received data into the receivedData array

    // Parsing the received data
    soilHumidity = (receivedData[3] << 8) | receivedData[4];
    soilTemperature = (receivedData[5] << 8) | receivedData[6];
    soilConductivity = (receivedData[7] << 8) | receivedData[8];
    soilPH = (receivedData[9] << 8) | receivedData[10];
    nitrogen = (receivedData[11] << 8) | receivedData[12];
    phosphorus = (receivedData[13] << 8) | receivedData[14];
    potassium = (receivedData[15] << 8) | receivedData[16];
  }
}

void sendSensorDataAttributes(float SoilTemperature, float SoilHumidity, float SoilConductivity, float SoilPH, float Nitrogen, float Phosphorus, float Potassium, double latitude, double longitude, double altitude) {

  // Create JSON payload
  StaticJsonDocument<200> jsonDocument;
  jsonDocument["humidity"] = (float)SoilHumidity / 10.0;
  jsonDocument["temperature"] = (float)SoilTemperature / 10.0;
  jsonDocument["conductivity"] = (float)SoilConductivity/100.0;
  jsonDocument["ph"] = (float)SoilPH / 10.0;
  jsonDocument["nitrogen"] = Nitrogen;
  jsonDocument["phosphorus"] = Phosphorus;
  jsonDocument["potassium"] = Potassium;
  jsonDocument["latitude"] = latitude;
  jsonDocument["longitude"] = longitude;
  jsonDocument["altitude"] = altitude;

  String payload;
  serializeJson(jsonDocument, payload);

  // Send POST request
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    http.begin("http://" + String(server) + ":" + String(serverPort) + "/api/v1/" + access_token + "/attributes");
    // http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.POST(payload);

    if (httpResponseCode > 0) {
      Serial.println("Attribute POST request sent successfully");
      Serial.print("Server response: ");
      Serial.println(httpResponseCode);
    } else {
      Serial.print("Error on sending Attribute POST request: ");
      Serial.println(httpResponseCode);
    }

    http.end();
  } else {
    Serial.println("WiFi Disconnected \n");
  }
}

void sendSensorDataTelemetry(float SoilTemperature, float SoilHumidity, float SoilConductivity, float SoilPH, float Nitrogen, float Phosphorus, float Potassium, double latitude, double longitude, double altitude) {

  // Create JSON payload
  StaticJsonDocument<200> jsonDocument;
  jsonDocument["humidity"] = (float)SoilHumidity / 10.0;
  jsonDocument["temperature"] = (float)SoilTemperature / 10.0;
  jsonDocument["conductivity"] = (float)SoilConductivity/100;
  jsonDocument["ph"] = (float)SoilPH / 10.0;
  jsonDocument["nitrogen"] = Nitrogen;
  jsonDocument["phosphorus"] = Phosphorus;
  jsonDocument["potassium"] = Potassium;
  jsonDocument["latitude"] = latitude;
  jsonDocument["longitude"] = longitude;
  jsonDocument["altitude"] = altitude;

  String payload;
  serializeJson(jsonDocument, payload);

  // Send POST request
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    http.begin("http://" + String(server) + ":" + String(serverPort) + "/api/v1/" + access_token + "/telemetry");
    // http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.POST(payload);

    if (httpResponseCode > 0) {
      Serial.println("Telemetry POST request sent successfully");
      Serial.print("Server response: ");
      Serial.println(httpResponseCode);
      Serial.println("");
    } else {
      Serial.print("Error on sending Telemetry POST request: ");
      Serial.println(httpResponseCode);
      Serial.println("");
    }

    http.end();
  } else {
    Serial.println("WiFi Disconnected \n");
  }
}

void initializeGPS() {
  // Initialization code for pins and serial ports
  pinMode(BOARD_POWER_ON_PIN, OUTPUT);
  digitalWrite(BOARD_POWER_ON_PIN, HIGH);

  pinMode(BOARD_RST_PIN, OUTPUT);
  digitalWrite(BOARD_RST_PIN, LOW);

  pinMode(BOARD_MODEM_PWR_PIN, OUTPUT);
  digitalWrite(BOARD_MODEM_PWR_PIN, LOW);
  delay(100);
  digitalWrite(BOARD_MODEM_PWR_PIN, HIGH);
  delay(1000);
  digitalWrite(BOARD_MODEM_PWR_PIN, LOW);

  // Modem Serial port
  SerialAT.begin(115200, SERIAL_8N1, BOARD_MODEM_RX_PIN, BOARD_MODEM_TX_PIN);
  // GPS Serial port
  SerialGPS.begin(9600, SERIAL_8N1, BOARD_GPS_RX_PIN, BOARD_GPS_TX_PIN);

  Serial.println("Initializing GPS...");
  delay(2000);
}

bool readGPSData(double &latitude, double &longitude, double &altitude) {
  while (SerialGPS.available()) {
    int c = SerialGPS.read();
    if (gps.encode(c)) {
      if (gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
      }
      if (gps.altitude.isValid()) {
        altitude = gps.altitude.meters();
        return true;  // Successfully obtained coordinates and altitude
      }
    }
  }
  return false;  // No valid coordinates or altitude found
}
