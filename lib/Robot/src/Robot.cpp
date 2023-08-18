#include "Robot.h"

Robot::Robot()
{
    // MobileBase mobileBase;
    // RoboticArm roboticArm;
}

void Robot::setSerial(Stream& serial) { this->serial = &serial; }