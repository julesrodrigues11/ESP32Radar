#define THINGSBOARD_ENABLE_PROGMEM 0

#include "WiFiCredentials.cpp"
#include "WiFi.h"
#include "Arduino_MQTT_Client.h"
#include "Update.h"
#include "ThingsBoard.h"
#include "DFRobot_C4001.h"

#pragma region THINGSBOARD VARIABLES
// Access Token
#define TOKEN "LjzVhXC9KWtOyNp9XJQQ"

// Thingsboard Server
#define THINGSBOARD_SERVER "18.130.204.215"

WiFiClient espClient;
Arduino_MQTT_Client client(espClient);

ThingsBoard tb(client);
#pragma endregion

#pragma region SENSOR VARIABLES
const uint8_t RadarAddress = 0x2A;

DFRobot_C4001_I2C radar(&Wire, RadarAddress);

float Distance = 0.0f;
#pragma endregion

// the Wifi radio's status
int status = WL_IDLE_STATUS;

// Function to print to Monitor the Configuration Parameters
// Meant to debug and verify that the sensor is working as expected
void printConfigParams()
{
    Serial.print("Min Range = ");
    Serial.println(radar.getTMinRange());

    Serial.print("Max Range = ");
    Serial.println(radar.getTMaxRange());

    Serial.print("Threshold Range = ");
    Serial.println(radar.getThresRange());

    Serial.print("Fretting Detection = ");
    Serial.println(radar.getFrettingDetection());
}

// Function to scan for devices attached to the I2C bus
void Scanner()
{
    Serial.println();
    Serial.println("I2C scanner. Scanning ....");

    byte count = 0;
    //Wire.begin(SDA, SCL); // This line can be omitted on boards other than the esp32c6
    for (byte i = 8; i < 120; i++)
    {
        Wire.beginTransmission(i);
        if (Wire.endTransmission() == 0)
        {
            Serial.print("Found address: ");
            Serial.print(i, DEC);
            Serial.print(" (0x");
            Serial.print(i, HEX);
            Serial.println(")");
            count++;
        }
    }
    Serial.print("Found ");
    Serial.print(count, DEC);
    Serial.print(" device(s).");
}

// Function to Initialise WiFi
// Initialises connection based on the WiFi AP Name and Password provided
void InitWiFi()
{
    Serial.print("Connecting to AP ...");
    // Attempt to connect to WiFi networki

    WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected to AP");
    digitalWrite(LED_BUILTIN, HIGH);

}

// Function to reconnect to WiFi if connection ever drops
void reconnect()
{
  // Loop until we're reconnected
  status = WiFi.status();
  if ( status != WL_CONNECTED) {
    WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Connected to AP");
  }
}

// Setup Function - Initialises Serial, WiFi and C4001 Presence Sensor
void setup()
{
    Serial.begin(115200);
    delay(2500);
    Serial.println("Initialised");

    // Initialise LED for debugging purposes
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
    InitWiFi();

    while(!radar.begin())
    {
        Serial.println("No Radars Found");
        delay(1000);
    }
    Serial.println("Radar Found!");

    // Speed Mode - To get object speed and distance from sensor
    radar.setSensorMode(eSpeedMode);

    sSensorStatus_t data;
    data = radar.getStatus();

    // 0 to indicate Stop, 1 to indicate Start
    Serial.print("Work status = ");
    Serial.println(data.workStatus);

    // 0 for Exist Mode, 1 for Speed Mode
    Serial.print("Work mode = ");
    Serial.println(data.workMode);

    // 0 Uninitialised, 1 Initialised
    Serial.print("Init status = ");
    Serial.println(data.initStatus);
    Serial.println();

    // To Set Detection Threshold
    // min The Minimum Distance for Detection Range, Unit cm, Range 0.3~20m (30~2500), must not exceed max
    // max The Maximum Distance for Detection Range, Unit cm, Range 2.4~20m (240~2500)
    // thres The Target Detection Threshold, Dimensionless unit 0.1, Range 0~6553.5 (0~65535)
    // radar.setDetectThres(min, max, thres)
    if(radar.setDetectThres(30, 500, 10))
    {
        Serial.println("Detection Threshold set successfully");
    }

    // Set Fretting Detection
    radar.setFrettingDetection(eON);

    // Print Configuration Params
    printConfigParams();

}

// Loop Function
// Checks WiFi connection and reconnects if necessary
// Checks ThingsBoard connection and reconnects if necessary
// Gets Distance Value from Presence Detection Sensor, and uploads them to ThingsBoard
void loop()
{
      // Reconnect to WiFi, if needed
    if (WiFi.status() != WL_CONNECTED)
    {
        pinMode(LED_BUILTIN, LOW);
        reconnect();
        return;
    }

    // Reconnect to ThingsBoard, if needed
    if (!tb.connected()) {

        // Connect to the ThingsBoard
        Serial.print("Connecting to: ");
        Serial.print(THINGSBOARD_SERVER);
        Serial.print(" with token ");
        Serial.println(TOKEN);
        if (!tb.connect(THINGSBOARD_SERVER, TOKEN))
        {
            Serial.println("Failed to connect");
            return;
        }
        Serial.println("Connection successful!");
    }

    // Important line, radar will not get target distance from sensor otherwise
    radar.getTargetNumber();

    Distance = radar.getTargetRange();
    Serial.print("Sending data to Thingsboard - ");
    Serial.println(Distance);

    tb.sendTelemetryData("Distance", Distance);

    tb.loop();
    
    delay(30);
}