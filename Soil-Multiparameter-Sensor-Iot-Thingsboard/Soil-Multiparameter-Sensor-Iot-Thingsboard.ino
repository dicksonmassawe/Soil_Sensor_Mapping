// Include Libraries
#include <Wire.h>
#include <WiFi.h>
#include <Arduino.h>
#include <HTTPClient.h>
#include "Credentials.h"
#include "gps_pin.h"
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include "time.h"
#include "sntp.h"

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

// Time libraries
const char *ntpServer1 = "pool.ntp.org";
const char *ntpServer2 = "time.nist.gov";
const long gmtOffset_sec = 7200;
const int daylightOffset_sec = 3600;
String year = "";
String month = "";
String day = "";
String date = "";
String hour = "";
String minute = "";
String second = "";


const char *server = "45.61.55.203";                 // Replace with your server IP address
const int serverPort = 9090;                         // Replace with your server port
const String access_token = "XAjieqa2LNFTIvyasBki";  // Replace with access token


// Declare global variables to hold sensor data
unsigned int soilHumidity = 0;
unsigned int soilTemperature = 0;
unsigned int soilConductivity = 0;
unsigned int soilPH = 0;
float nitrogen = 0;
unsigned int phosphorus = 0;
float potassium = 0;

TinyGPSPlus gps;

// Global variables for latitude and longitude
double latitude = 0.0;
double longitude = 0.0;
double altitude = 0.0;

// LED and  Buzzer
#define white1 34
#define white2 35
#define red 25
#define blue 13
#define green 2
#define buzzer 14


void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  // Initializing sensor
  sensor.begin(4800, SERIAL_8N1, 33, 32);  // Start hardware serial at 4800 bps

  // LED and Buzzer
  pinMode(white1, OUTPUT);
  pinMode(white2, OUTPUT);
  pinMode(red, OUTPUT);
  pinMode(blue, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(buzzer, OUTPUT);

  digitalWrite(white1, 1);
  digitalWrite(white2, 1);
  digitalWrite(red, 0);
  digitalWrite(blue, 0);
  digitalWrite(green, 0);
  digitalWrite(buzzer, 0);

  // Connecting to WiFi
  connectToWiFi();

  // set notification call-back function
  sntp_set_time_sync_notification_cb(timeavailable);

  /**
   * NTP server address could be aquired via DHCP,
   *
   * NOTE: This call should be made BEFORE esp32 aquires IP address via DHCP,
   * otherwise SNTP option 42 would be rejected by default.
   * NOTE: configTime() function call if made AFTER DHCP-client run
   * will OVERRIDE aquired NTP server address
   */
  sntp_servermode_dhcp(1);  // (optional)

  /**
   * This will set configured ntp servers and constant TimeZone/daylightOffset
   * should be OK if your time zone does not need to adjust daylightOffset twice a year,
   * in such a case time adjustment won't be handled automagicaly.
   */
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);

  //Initializing GPS
  initializeGPS();
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
  Serial.print((float)soilConductivity);
  Serial.print(" uS/m    ");  // microsiemens per centimeter (µS/cm)
  Serial.print("Soil pH: ");
  Serial.print((float)soilPH / 10.0);
  Serial.print("   ");
  Serial.print("Nitrogen: ");
  Serial.print(nitrogen * 0.001);
  Serial.print(" %    ");
  Serial.print("Phosphorus: ");
  Serial.print(phosphorus);
  Serial.print(" mg/kg    ");
  Serial.print("Potassium: ");
  Serial.print(potassium / 390);
  Serial.println(" cmol(+)/kg");

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


  // Get date and time of the collected data
  getFormattedLocalTime(year, month, day, date, hour, minute, second);
  Serial.println(year + "," + month + "," + date + "  " + day + " " + hour + ":" + minute + ":" + second + "\n");

  // POST the sensor data via HTTP
  sendSensorDataAttributes(soilTemperature, soilHumidity, soilConductivity, soilPH, nitrogen, phosphorus, potassium, latitude, longitude, altitude, year, month, day, date, hour, minute, second);
  sendSensorDataTelemetry(soilTemperature, soilHumidity, soilConductivity, soilPH, nitrogen, phosphorus, potassium, latitude, longitude, altitude, year, month, day, date, hour, minute, second);

  // delay(10000);  // Delay between readings
}

void connectToWiFi() {
  WiFi.begin(mySSID, myPASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
    digitalWrite(blue, HIGH);
    delay(1000);
    digitalWrite(blue, LOW);
  }

  Serial.println("Connected to WiFi");
  // buzzerSuccess(blue);
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

void sendSensorDataAttributes(float SoilTemperature, float SoilHumidity, float SoilConductivity, float SoilPH, float Nitrogen, float Phosphorus, float Potassium, double latitude, double longitude, double altitude, String year, String month, String day, String date, String hour, String minute, String second) {

  // Create JSON payload
  StaticJsonDocument<200> jsonDocument;
  jsonDocument["humidity"] = (float)SoilHumidity / 10.0;
  jsonDocument["temperature"] = (float)SoilTemperature / 10.0;
  jsonDocument["conductivity"] = (float)SoilConductivity;
  jsonDocument["ph"] = (float)SoilPH / 10.0;
  jsonDocument["nitrogen"] = Nitrogen * 0.001;
  jsonDocument["phosphorus"] = Phosphorus;
  jsonDocument["potassium"] = Potassium / 390;
  jsonDocument["latitude"] = latitude;
  jsonDocument["longitude"] = longitude;
  jsonDocument["altitude"] = altitude;
  jsonDocument["year"] = year;
  jsonDocument["month"] = month;
  jsonDocument["day"] = day;
  jsonDocument["date"] = date;
  jsonDocument["hour"] = hour;
  jsonDocument["minute"] = minute;
  jsonDocument["second"] = second;

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
      // buzzerSuccess(green);
    } else {
      Serial.print("Error on sending Attribute POST request: ");
      Serial.println(httpResponseCode);
      // buzzerError(red);
    }

    http.end();
  } else {
    Serial.println("WiFi Disconnected \n");
    // Connecting to WiFi
    connectToWiFi();
  }
}

void sendSensorDataTelemetry(float SoilTemperature, float SoilHumidity, float SoilConductivity, float SoilPH, float Nitrogen, float Phosphorus, float Potassium, double latitude, double longitude, double altitude, String year, String month, String day, String date, String hour, String minute, String second) {

  // Create JSON payload
  StaticJsonDocument<200> jsonDocument;
  jsonDocument["humidity"] = (float)SoilHumidity / 10.0;
  jsonDocument["temperature"] = (float)SoilTemperature / 10.0;
  jsonDocument["conductivity"] = (float)SoilConductivity;
  jsonDocument["ph"] = (float)SoilPH / 10.0;
  jsonDocument["nitrogen"] = Nitrogen * 0.001;
  jsonDocument["phosphorus"] = Phosphorus;
  jsonDocument["potassium"] = Potassium / 390;
  jsonDocument["latitude"] = latitude;
  jsonDocument["longitude"] = longitude;
  jsonDocument["altitude"] = altitude;
  jsonDocument["year"] = year;
  jsonDocument["month"] = month;
  jsonDocument["day"] = day;
  jsonDocument["date"] = date;
  jsonDocument["hour"] = hour;
  jsonDocument["minute"] = minute;
  jsonDocument["second"] = second;

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
      // buzzerSuccess(green);
    } else {
      Serial.print("Error on sending Telemetry POST request: ");
      Serial.println(httpResponseCode);
      Serial.println("");
      // buzzerError(red);
    }

    http.end();
  } else {
    Serial.println("WiFi Disconnected \n");
    // Connecting to WiFi
    connectToWiFi();
  }
}

void initializeGPS() {
  bool gpsInitialized = false;

  while (!gpsInitialized) {
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

    // Check if GPS is initialized
    if (SerialGPS.available()) {
      gpsInitialized = true;
      Serial.println("GPS Initialized successfully.");
      // buzzerSuccess(white2);
    } else {
      Serial.println("Failed to initialize GPS. Retrying...");
      // buzzerError(white2);
    }
  }
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


void getFormattedLocalTime(String &year, String &month, String &dayOfWeek, String &day, String &hour, String &minute, String &second) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    year = month = dayOfWeek = day = hour = minute = second = "No time available (yet)";
    // buzzerError(white1);
    return;
  }

  char yearBuffer[5];        // Buffer for year
  char monthBuffer[3];       // Buffer for month
  char dayOfWeekBuffer[10];  // Buffer for day of week
  char dayBuffer[3];         // Buffer for day
  char hourBuffer[3];        // Buffer for hour
  char minuteBuffer[3];      // Buffer for minute
  char secondBuffer[3];      // Buffer for second

  // Convert numerical values to strings
  snprintf(yearBuffer, sizeof(yearBuffer), "%d", timeinfo.tm_year + 1900);
  snprintf(monthBuffer, sizeof(monthBuffer), "%02d", timeinfo.tm_mon + 1);
  snprintf(dayOfWeekBuffer, sizeof(dayOfWeekBuffer), "%s", getDayOfWeekName(timeinfo.tm_wday));
  snprintf(dayBuffer, sizeof(dayBuffer), "%02d", timeinfo.tm_mday);
  snprintf(hourBuffer, sizeof(hourBuffer), "%02d", timeinfo.tm_hour);
  snprintf(minuteBuffer, sizeof(minuteBuffer), "%02d", timeinfo.tm_min);
  snprintf(secondBuffer, sizeof(secondBuffer), "%02d", timeinfo.tm_sec);

  // Assign values to the output variables
  year = String(yearBuffer);
  month = String(monthBuffer);
  dayOfWeek = String(dayOfWeekBuffer);
  day = String(dayBuffer);
  hour = String(hourBuffer);
  minute = String(minuteBuffer);
  second = String(secondBuffer);
}

// Function to get the day of the week name from the day of the week number
const char *getDayOfWeekName(int dayOfWeek) {
  const char *dayNames[] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };
  if (dayOfWeek >= 0 && dayOfWeek < 7) {
    return dayNames[dayOfWeek];
  } else {
    return "Invalid Day";
  }
}

// Callback function (get's called when time adjusts via NTP)
void timeavailable(struct timeval *t) {
  Serial.println("Got time adjustment from NTP!");
  // buzzerSuccess(white1);
}

// void buzzerError(int led) {
//   for (int i = 0; i < 10; ++i) {
//     digitalWrite(buzzer, HIGH);
//     digitalWrite(led, HIGH);
//     delay(50);
//     digitalWrite(buzzer, LOW);
//     delay(50);
//   }
//   digitalWrite(led, LOW);
// }

// void buzzerSuccess(int led) {
//   for (int i = 0; i < 3; ++i) {
//     digitalWrite(buzzer, HIGH);
//     digitalWrite(led, HIGH);
//     delay(500);
//     digitalWrite(buzzer, LOW);
//     delay(500);
//   }
//   digitalWrite(led, LOW);
// }
