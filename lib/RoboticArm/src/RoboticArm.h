#ifndef RoboticArm_H
#define RoboticArm_H
#include "Arduino.h"
#include "AccelStepper.h"

struct Joint
{
    int32_t ID;
    AccelStepper motor;
    uint32_t limit;
    uint32_t microStep = 8;
    uint32_t stepsPerRev = 200;

    float motorAngle = 0;
    float jointAngle = 0;

    float reductionRatio;
    
    uint32_t switchPin;
    bool switchReading = false;
    bool switchReadingPrev = false;
    uint32_t switchDebounceTime = 0;
    bool switchState = false;
    bool switchStatePrev = false;

    bool homed = false;
    bool homing = false;
    enum HomingState{IDLE, MOVING_TO_LIMIT_SWITCH, MOVING_TO_ZERO};
    HomingState homingState = HomingState::IDLE;
    // 0 is highest priority
    uint32_t homingPriority = 0;
    int32_t homingStartPos = 0;
};

struct Link
{
    uint32_t length = 0;
    uint32_t mass = 0;
};

class RoboticArm
{
public:
    enum ControlMode
    {
        NONE, VELOCITY_IK, INITIAL_CONFIG
    };
    RoboticArm(uint32_t numJoints);
    void update();
    void init();
    void addJoint(uint32_t stepPin, uint32_t dirPin, uint32_t jointLimit, float reductionRatio, uint32_t switchPin);
    void setControlMode(ControlMode controlMode) { _controlMode = controlMode; }
    void homeJoint(Joint &joint, bool waitForPreviousJoints = false);
    void homeAllJoints();
    
    Joint *joints;
    Link *links;

private:
    uint32_t _numJoints;
    int32_t _jointIndex = 0;
    ControlMode _controlMode = NONE;
    float desiredCartesianVelocity[3];
    void _homeJoint(Joint &joint);
    void _readJointSwitch(Joint &joint);
    void _calculateAngles();
};

#endif