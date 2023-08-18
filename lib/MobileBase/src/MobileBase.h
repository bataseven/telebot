#ifndef MobileBase_H
#define MobileBase_H
#include "Arduino.h"
#include "AccelStepper.h"
struct WheelVelocities
{
    double w_fl;
    double w_fr;
    double w_rl;
    double w_rr;
};

class MobileBase
{
public:
    enum ControlMode
    {
        MANUAL
    };
    MobileBase(int step_fl, int dir_fl, int step_fr, int dir_fr, int step_rl, int dir_rl, int step_rr, int dir_rr, int lx, int ly, int r);
    void update();
    AccelStepper wheel_fl;
    AccelStepper wheel_fr;
    AccelStepper wheel_rl;
    AccelStepper wheel_rr;
    void setDesiredVx(uint32_t desiredVx);
    void setDesiredVy(uint32_t desiredVy);
    void setDesiredWz(uint32_t desiredWz);

private:
    uint32_t _lx = 10; // in mm
    uint32_t _ly = 10; // in mm
    uint32_t _r = 10;  // in mm
    uint32_t _desiredVx = 0;
    uint32_t _desiredVy = 0;
    uint32_t _desiredWz = 0;
    WheelVelocities calculateIK();
};

#endif