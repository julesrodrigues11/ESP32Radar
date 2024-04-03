#include "DFRobot_C4001.h"

uint8_t RadarAddress = 0x2A;

DFRobot_C4001_I2C radar(&Wire, RadarAddress);

void printConfigParams();

void Scanner()
{
    Serial.println();
    Serial.println("I2C scanner. Scanning ....");

    byte count = 0;
    Wire.begin(SDA, SCL);
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


void setup()
{
    Serial.begin(115200);
    delay(2500);
    Serial.println("Initialised");

    Wire.begin(SDA, SCL);

    while(!radar.begin())
    {
        Serial.println("No Radars Found");
        delay(1000);
    }
    Serial.println("Radar Found!");

    // Exist Mode
    radar.setSensorMode(eExitMode);

    sSensorStatus_t data;
    data = radar.getStatus();

    // 0 stop 1 start
    Serial.print("Work status = ");
    Serial.println(data.workStatus);

    // 0 is exist 1 speed
    Serial.print("Work mode = ");
    Serial.println(data.workMode);

    // 0 Uninitialised 1 Initialised
    Serial.print("Init status = ");
    Serial.println(data.initStatus);
    Serial.println();

    // Minimum Detection Range - Minimum Distance, Unit cm, Range 0.3~25m (30~2500), not exceeding max, otherwise the function is abnormal
    // Maximum Detection Range - Maximum Distance, Unit cm, Range 2.4~25m (240~2500)
    // Trig Detection Range - Maximum Distance, Unit cm, Default trig = max
    if(radar.setDetectionRange(/*Min*/30, /*Max*/200, /*Trig*/200))
    {
        Serial.println("Detection range set successfully");
    }

    // Set Trigger Sensitivity 0 - 9
    if(radar.setTrigSensitivity(1))
    {
        Serial.println("Trig sensitivity set successfully");
    }

    // Set Keep Sensitivity 0 - 9
    if(radar.setKeepSensitivity(2))
    {
        Serial.println("Keep sensitivity set successfully");
    }

    // trig Trigger delay, Unit 0.01s, Range 0~2s (0~200)
    // keep Maintain the detection timeout, Unit 0.5s, Range 2~1500s (4~3000)
    if(radar.setDelay(/*trig*/100, /*keep*/4))
    {
        Serial.println("Delay set successfully");
    }

    // Get Config Params
    printConfigParams();
}

void printConfigParams()
{
    Serial.println();
    Serial.print("Trig Sensitivity = ");
    Serial.println(radar.getTrigSensitivity());

    Serial.print("Keep Sensitivity = ");
    Serial.println(radar.getKeepSensitivity());

    Serial.print("Min Range = ");
    Serial.println(radar.getMinRange());

    Serial.print("Max Range = ");
    Serial.println(radar.getMaxRange());

    Serial.print("Trig Range = ");
    Serial.println(radar.getTrigRange());

    Serial.print("Keep Time = ");
    Serial.println(radar.getKeepTimerout());

    Serial.print("Trig Delay = ");
    Serial.println(radar.getTrigDelay());

}

void loop()
{
  // Determine whether the object is moving
  if(radar.motionDetection()){
    Serial.println("Motion Detected");
    Serial.println();
  }
  else
  {
    Serial.println("Sleep");
  }
  delay(1000);
}