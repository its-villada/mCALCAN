#include <Arduino.h>

#define PC_BAUDRATE 9600
#define GPS_BAUDRATE 9600

void setup()
{
    Serial.begin(PC_BAUDRATE);
    Serial2.begin(GPS_BAUDRATE);
}

// If there is a data from the receiver, read it and send to the PC or vice versa.
void loop()
{
    if (Serial2.available())
    {
        Serial.write(Serial2.read());
    }

    if (Serial.available())
    {
        Serial2.write(Serial.read());
    }
}
