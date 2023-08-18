#include <Arduino.h>
#include "Robot.h"
#include "SerialHandler.h"

#define SERIAL_PORT Serial

Robot *robot;
SerialHandler SH = SerialHandler();
void setup()
{
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    SH.setSerial(SERIAL_PORT);
    // device->setSerial(SERIAL_PORT);

    // Blink the LED yellow until the serial connection is established.
    while (!SERIAL_PORT)
    {
        digitalWrite(13, millis() / 1000 % 2 == 0);
        delay(1);
    }
}
void loop()
{
}