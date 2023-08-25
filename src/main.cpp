#include <Arduino.h>
#include "Robot.h"
#include "SerialHandler.h"

#define SERIAL_PORT Serial1
#define SERIAL_BAUD_RATE 115200

Robot *robot;
SerialHandler SH = SerialHandler();
void setup()
{
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    SERIAL_PORT.begin(SERIAL_BAUD_RATE);
    SH.setSerial(SERIAL_PORT);
    SH.setDebug(true);
    robot = new Robot();
    SH.setRobot(robot);
    // Blink the LED yellow until the serial connection is established.
    while (!SERIAL_PORT)
    {
        digitalWrite(13, millis() / 1000 % 2 == 0);
        delay(1);
    }
    SH.setPrintFrequency(0);
}
void loop()
{
    SH.update();
    robot->update();
}