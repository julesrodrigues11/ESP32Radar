#define THINGSBOARD_ENABLE_PROGMEM 0
//#define THINGSBOARD_ENABLE_DEBUG 1 // Uncomment this line for Thingsboard debugging

#include "WiFiCredentials.cpp"
#include "WiFi.h"
#include "Arduino_MQTT_Client.h"
#include "Update.h"
#include "ThingsBoard.h"
#include "DFRobot_C4001.h"

#pragma region THINGSBOARD VARIABLES

// Access Token - To change depending upon device
#define TOKEN "ari6coh7jcfTJyHq0QoD"

// Thingsboard Server
#define THINGSBOARD_SERVER "52.56.195.105"

WiFiClient espClient;
Arduino_MQTT_Client client(espClient);

ThingsBoard tb(client);
#pragma endregion

#pragma region SENSOR VARIABLES

const int MIN_RANGE = 30;
const int MAX_RANGE = 300;

const uint8_t RadarAddress_1 = 0x2A;
const uint8_t RadarAddress_2 = 0x2B;

DFRobot_C4001_I2C Radar1(&Wire, RadarAddress_1);
DFRobot_C4001_I2C Radar2(&Wire, RadarAddress_2);

float Distance_R1 = 0.0f;
float Distance_R2 = 0.0f;

int R1_Data = 0;
int R2_Data = 0;

#pragma endregion

#pragma region SMOOTHING VARIABLES
float ewmaAlpha = 0.1f;
float ewma_R1 = 0.0f;
float ewma_R2 = 0.0f;
#pragma endregion

#pragma region I2C VARIABLES
#define SLAVE_ADDRESS 8
#pragma endregion

#pragma region OTHER VARIABLES
#define MIN_NORM 1
#define MAX_NORM 100
#pragma endregion

// the Wifi radio's status
int status = WL_IDLE_STATUS;

// Function to obtain JSON string from the data values provided
// Appends the static values to the JSON string value and then returns it
// The data values need to be converted into strings to facilitate appending
std::string GetJsonString(int distance, int smoothedDistance)
{
    std::string jsonString = "";

    jsonString.append("{Radar1_Distance:");
    jsonString.append(std::to_string(distance));
    jsonString.append(",Radar2_Distance:");
    jsonString.append(std::to_string(smoothedDistance));
    jsonString.append("}");

    return jsonString;
}

// Function to map value from provided range to fall within the range of 1 to 100
int MapValue(int value)
{
    int mappedValue = map(value, MIN_RANGE, MAX_RANGE, MIN_NORM, MAX_NORM);
    if (mappedValue < MIN_NORM || mappedValue > MAX_NORM)
        return MAX_NORM;
    else
        return mappedValue;
}

// Function to Transfer Data over I2C on selected address
// Compares the two values and transfers the lower value
void TransferData(int radarRead1, int radarRead2)
{
    Wire1.beginTransmission(SLAVE_ADDRESS);
    if(radarRead1 <= radarRead2)
    {
        Wire1.write(radarRead1);
    }
    else
    {
        Wire1.write(radarRead2);
    }
    Wire1.endTransmission();
}

// Function to print to Monitor the Configuration Parameters
// Meant to debug and verify that the sensor is working as expected
void printConfigParams(DFRobot_C4001_I2C radar)
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
    Wire.begin(SDA, SCL); // This line can be omitted on boards other than the esp32c6
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
    // Attempt to connect to WiFi network

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

// Function to Initialise Radar passed as parameter
// Sets Detection Parameters for given radar
void InitialiseRadar(DFRobot_C4001_I2C radar)
{
    while(!radar.begin())
    {
        Serial.println("Radar Not Found!");
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
    if(radar.setDetectThres(MIN_RANGE, MAX_RANGE, 10))
    {
        Serial.println("Detection Threshold set successfully");
    }

    // Set Fretting Detection
    radar.setFrettingDetection(eON);

    // Print Configuration Params
    // printConfigParams(radar);
}

// Setup Function - Initialises Serial, WiFi and C4001 Presence Sensors 1 and 2
void setup()
{
    Serial.begin(115200);
    delay(2500);
    Serial.println("Initialised");

    // Initialise Wire1 on pins D2 and D3
    Wire1.begin(D2, D3);

    // Initialise LED for debugging purposes
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    WiFi.begin(WIFI_AP_NAME, WIFI_PASSWORD);
    InitWiFi();

    InitialiseRadar(Radar1);
    InitialiseRadar(Radar2);
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

    #pragma region RADAR_1
    //Important line, radar will not get target distance from sensor otherwise
    if(Radar1.getTargetNumber() >= 1)
    {
        // Get Distance Value from Radar 1
        Distance_R1 = Radar1.getTargetRange();
        
        // Apply Smoothing to Distance Value obtained
        ewma_R1 = (ewmaAlpha * Distance_R1) + (1 - ewmaAlpha) * ewma_R1;
        R1_Data = MapValue(ewma_R1 * 100);
    }
    else
    {
        R1_Data = MAX_NORM;
    }

    #pragma endregion

    #pragma region RADAR_2
    //Important line, radar will not get target distance from sensor otherwise
    if(Radar2.getTargetNumber())
    {
        // Get Distance Value from Radar 2
        Distance_R2 = Radar2.getTargetRange();

        //Apply Smoothing to Distance Value obtained
        ewma_R2 = (ewmaAlpha * Distance_R2) + (1 - ewmaAlpha) * ewma_R2;
        R2_Data = MapValue(ewma_R2 * 100);

    }
    else
    {
        R2_Data = MAX_NORM;
    }

    #pragma endregion

    tb.sendTelemetryData("Radar1_Dist", R1_Data);
    tb.sendTelemetryData("Radar2_Dist", R2_Data);

    TransferData(R1_Data, R2_Data);

    tb.loop();
    
    delay(200);
}