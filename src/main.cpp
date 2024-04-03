#include "DFRobot_C4001.h"

const uint8_t RadarAddress = 0x2A;

DFRobot_C4001_I2C radar(&Wire, RadarAddress);

float Distance = 0.0f;

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

// Setup Function - Initialises Serial and C4001 Presence Sensor
void setup()
{
    Serial.begin(115200);
    delay(2500);
    Serial.println("Initialised");

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

// Loop Function - Gets object distance from sensor, parses it accordingly
void loop()
{
    // Important line, radar will not get target distance from sensor otherwise
    radar.getTargetNumber();

    Distance = radar.getTargetRange();
    if (Distance < 5.0f && Distance >= 3.0f)
    {
        Serial.print("Cold - ");
    }
    else if (Distance < 3.0f && Distance >= 1.0f)
    {
        Serial.print("Lukewarm - ");
    }
    else if (Distance < 1.0f)
    {
        Serial.print("Hot - ");
    }
    Serial.println(Distance);

    delay(1000);
}