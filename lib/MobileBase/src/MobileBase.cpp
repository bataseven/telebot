#include "MobileBase.h"

MobileBase::MobileBase(int step_fl, int dir_fl, int step_fr, int dir_fr, int step_rl, int dir_rl, int step_rr, int dir_rr, int lx, int ly, int r)
    : wheel_fl(AccelStepper::DRIVER, step_fl, dir_fl),
      wheel_fr(AccelStepper::DRIVER, step_fr, dir_fr),
      wheel_rl(AccelStepper::DRIVER, step_rl, dir_rl),
      wheel_rr(AccelStepper::DRIVER, step_rr, dir_rr)
{
    _lx = lx;
    _ly = ly;
    _r = r;
}

void MobileBase::update()
{
    WheelVelocities wheelVelocities = calculateIK();
    wheel_fl.setSpeed(wheelVelocities.w_fl);
    wheel_fr.setSpeed(wheelVelocities.w_fr);
    wheel_rl.setSpeed(wheelVelocities.w_rl);
    wheel_rr.setSpeed(wheelVelocities.w_rr);
    wheel_fl.runSpeed();
    wheel_fr.runSpeed();
    wheel_rl.runSpeed();
    wheel_rr.runSpeed();
}

WheelVelocities MobileBase::calculateIK()
{
    double v_x = _desiredVx;
    double v_y = _desiredVy;
    double w_z = _desiredWz;
    double l_x = _lx;
    double l_y = _ly;
    double r = _r;

    WheelVelocities result;

    result.w_fl = (1.0 / r) * (v_x - v_y - (l_x + l_y) * w_z);
    result.w_fr = (1.0 / r) * (v_x + v_y + (l_x + l_y) * w_z);
    result.w_rl = (1.0 / r) * (v_x + v_y - (l_x + l_y) * w_z);
    result.w_rr = (1.0 / r) * (v_x - v_y + (l_x + l_y) * w_z);

    return result;
}