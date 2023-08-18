#ifndef Robot_H
#define Robot_H
#include "Arduino.h"
#include "RoboticArm.h"
#include "MobileBase.h"
// #include "SerialHandler.h"

class Robot
{
private:
    Stream *serial;

public:
    Robot();
    void setSerial(Stream &serial);
};

#endif