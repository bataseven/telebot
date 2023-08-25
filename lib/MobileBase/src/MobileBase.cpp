#include "MobileBase.h"

MobileBase::MobileBase(int step_fl, int dir_fl, int step_fr, int dir_fr, int step_rl, int dir_rl, int step_rr, int dir_rr, double lx, double ly, double r)
    : wheel_fl(AccelStepper::DRIVER, step_fl, dir_fl),
      wheel_fr(AccelStepper::DRIVER, step_fr, dir_fr),
      wheel_rl(AccelStepper::DRIVER, step_rl, dir_rl),
      wheel_rr(AccelStepper::DRIVER, step_rr, dir_rr)
{
    _lx = lx;
    _ly = ly;
    _r = r;

    _setMaxWheelSpeed();
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

    v_x = map(v_x, -1, 1, -_maxBaseSpeed, _maxBaseSpeed);
    v_y = map(v_y, -1, 1, -_maxBaseSpeed, _maxBaseSpeed);
    w_z = map(w_z, -1, 1, -_maxBaseAngularSpeed, _maxBaseAngularSpeed);

    WheelVelocities result;

    // Make sure no division by zero
    result.w_fl = (r == 0 || (v_x - v_y - (l_x + l_y) * w_z) == 0) ? 0 : (1.0 / r) * (v_x - v_y - (l_x + l_y) * w_z);
    result.w_fr = (r == 0 || (v_x + v_y + (l_x + l_y) * w_z) == 0) ? 0 : (1.0 / r) * (v_x + v_y + (l_x + l_y) * w_z);
    result.w_rl = (r == 0 || (v_x + v_y - (l_x + l_y) * w_z) == 0) ? 0 : (1.0 / r) * (v_x + v_y - (l_x + l_y) * w_z);
    result.w_rr = (r == 0 || (v_x - v_y + (l_x + l_y) * w_z) == 0) ? 0 : (1.0 / r) * (v_x - v_y + (l_x + l_y) * w_z);

    // Convert from rad/s to steps/s
    result.w_fl = result.w_fl * (WHEEL_STEPS_PER_REVOLUTION * MICRO_STEPPING_WHEEL) / (2 * PI);
    result.w_fr = result.w_fr * (WHEEL_STEPS_PER_REVOLUTION * MICRO_STEPPING_WHEEL) / (2 * PI);
    result.w_rl = result.w_rl * (WHEEL_STEPS_PER_REVOLUTION * MICRO_STEPPING_WHEEL) / (2 * PI);
    result.w_rr = result.w_rr * (WHEEL_STEPS_PER_REVOLUTION * MICRO_STEPPING_WHEEL) / (2 * PI);

    return result;
}

// Desired speed of the robot in forward direction.
// Values should be between -1 and 1
// 1 is maxBaseSpeed forward
// -1 is maxBaseSpeed backward
void MobileBase::setDesiredVx(double desiredVx)
{
    _desiredVx = constrain(desiredVx, -1, 1);
}

// Desired speed of the robot in sideways direction.
// Values should be between -1 and 1
// 1 is maxBaseSpeed to the right
// -1 is maxBaseSpeed to the left
void MobileBase::setDesiredVy(double desiredVy)
{
    _desiredVy = constrain(desiredVy, -1, 1);
}

// Desired angular speed of the robot.
// Values should be between -1 and 1
// 1 is maxBaseAngularSpeed clockwise
// -1 is maxBaseAngularSpeed counterclockwise
void MobileBase::setDesiredWz(double desiredWz)
{
    _desiredWz = constrain(desiredWz, -1, 1);
}

void MobileBase::setMaxBaseSpeed(double maxBaseSpeedInMetersPerSecond)
{
    _maxBaseSpeed = maxBaseSpeedInMetersPerSecond;
    _setMaxWheelSpeed();
}

void MobileBase::setMaxBaseAngularSpeed(double maxBaseAngularSpeedInRadiansPerSecond)
{
    _maxBaseAngularSpeed = maxBaseAngularSpeedInRadiansPerSecond;
}

void MobileBase::_setMaxWheelSpeed()
{
    // Convert from m/s to steps/s
    _maxWheelSpeed = _maxBaseSpeed / (_wheelDiameter * PI) * (WHEEL_STEPS_PER_REVOLUTION * MICRO_STEPPING_WHEEL);

    // Allow max speed to be slightly larger to accomodate for the rotation of the robot
    wheel_fl.setMaxSpeed(_maxWheelSpeed * 2);
    wheel_fr.setMaxSpeed(_maxWheelSpeed * 2);
    wheel_rl.setMaxSpeed(_maxWheelSpeed * 2);
    wheel_rr.setMaxSpeed(_maxWheelSpeed * 2);
    wheel_fl.setAcceleration(100000);
    wheel_fr.setAcceleration(100000);
    wheel_rl.setAcceleration(100000);
    wheel_rr.setAcceleration(100000);
}
