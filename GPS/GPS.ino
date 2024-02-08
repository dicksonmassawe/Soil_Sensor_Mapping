#ifndef SerialAT
#define SerialAT Serial1
#endif

#ifndef SerialGPS
#define SerialGPS Serial2
#endif


#define BOARD_MODEM_DTR_PIN                 25
#define BOARD_MODEM_TX_PIN                  26
#define BOARD_MODEM_RX_PIN                  27
#define BOARD_MODEM_PWR_PIN                 4
#define BOARD_ADC_PIN                       35
#define BOARD_POWER_ON_PIN                  12
#define BOARD_MODEM_RI_PIN                  33
#define BOARD_RST_PIN                       5
#define BOARD_SDCARD_MISO                   2
#define BOARD_SDCARD_MOSI                   15
#define BOARD_SDCARD_SCLK                   14
#define BOARD_SDCARD_CS                     13

#define BOARD_GPS_TX_PIN                    21
#define BOARD_GPS_RX_PIN                    22
#define BOARD_GPS_PPS_PIN                   23
#define BOARD_GPS_WAKEUP_PIN                19

#include <Arduino.h>
#include <TinyGPS++.h>

TinyGPSPlus gps;

// Global variables for latitude and longitude
double latitude = 0.0;
double longitude = 0.0;
double altitude = 0.0;

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

    Serial.begin(115200);
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
                return true; // Successfully obtained coordinates and altitude
            }
        }
    }
    return false; // No valid coordinates or altitude found
}


void setup() {
    initializeGPS();
}

void loop() {
    double altitude = 0.0;
    if (readGPSData(latitude, longitude, altitude)) {
        // Do something with latitude, longitude, and altitude here
        Serial.print("Latitude: ");
        Serial.println(latitude, 6);
        Serial.print("Longitude: ");
        Serial.println(longitude, 6);
        Serial.print("Altitude: ");
        Serial.println(altitude, 2);
    }
    delay(1000); // Adjust delay as needed
}