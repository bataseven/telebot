#ifndef MobileBase_H
#define MobileBase_H
#include "Arduino.h"
#include "AccelStepper.h"

#define MICRO_STEPPING_WHEEL 8
#define WHEEL_STEPS_PER_REVOLUTION 200

struct WheelVelocities
{
    double w_fl; // in steps/s
    double w_fr; // in steps/s
    double w_rl; // in steps/s
    double w_rr; // in steps/s
};

class MobileBase
{
public:
    enum ControlMode
    {
        MANUAL
    };
    MobileBase(int step_fl, int dir_fl, int step_fr, int dir_fr, int step_rl, int dir_rl, int step_rr, int dir_rr, double lx, double ly, double r);
    void update();
    AccelStepper wheel_fl;
    AccelStepper wheel_fr;
    AccelStepper wheel_rl;
    AccelStepper wheel_rr;
    void setDesiredVx(double desiredVx);
    void setDesiredVy(double desiredVy);
    void setDesiredWz(double desiredWz);
    void setMaxBaseSpeed(double maxBaseSpeedInMetersPerSecond);
    void setMaxBaseAngularSpeed(double maxBaseAngularSpeedInRadiansPerSecond);

private:
    double _wheelDiameter = 0.080; // in m
    double _lx = 10; // in m
    double _ly = 10; // in m
    double _r = 10;  // in m
    double _desiredVx = 0;
    double _desiredVy = 0;
    double _desiredWz = 0;
    double _maxBaseSpeed = 0.5; // in m/s
    double _maxBaseAngularSpeed = PI; // in rad/s
    double _maxWheelSpeed;
    WheelVelocities calculateIK();
    ControlMode _controlMode;
    void _setMaxWheelSpeed();
};

#endif