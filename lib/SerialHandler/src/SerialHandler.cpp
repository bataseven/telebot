#include "SerialHandler.h"

/*"""

 Serial Message Format:
 Every message sent and received over the serial should start with a '_startMarker' and end with an '_endMarker'.
 First character (after the start character) of the message is the message type for both sending and reception.
 The message type is followed by the message payload.
 Different values inside the payload are separated by a '_separator'.
 SERIAL_SEPERATOR_CH = '#'
_separator = '#'

  - While sending to haptic device:
        To get gripper data:    "G" + <Gripper Width> + SERIAL_SEPERATOR_CH + <Gripper Speed> + SERIAL_SEPERATOR_CH +
<Gripper Force>  => "G0.5#0.5#0.5"

To specify which program is connected to the haptic device over the serial port: "C" + <Program> => "C0"
List of programs are defined as enum in the header file.

To stop the motors:     "S" + <Motor Index> + <Stop(0 or 1)> => "S01"

To change PID gains:    "K" + "P" or "I" or "D" + <MotorIndex> + SERIAL_SEPERATOR_CH + <Gain> => "KP1#0.5"

To change waveforms:    "W" + "T" or "F" or "A" or "O" + <Generator Index> + SERIAL_SEPERATOR_CH + <Value> => "WF1#0.5"

To change the controller mode: "O" + <Motor Index> + <ControllerMode> => "O10"

To set desired force:   "D" + <Desired Force> => "D11.5"

To send the stiffness and the nominal width of the object: "P" + <Nominal Width> + # + <Stiffness> + # + <Damping> =>
"P50000#2.5#0.1"

To set the limits (hard or soft) for the motors: "L" + "H" or "S" + <Motor Index> + <Lower Limit> + # + <Upper Limit> =>
"LS01000#2000"

To home the motors: "H" + <Motor Index> => "H1"

To toggle set the serial handler debug mode: "B" + <Debug Mode> => "B1" or "B0"


  - While receiving from haptic device:

        (No longer used) Desired aperture of the gripper:    "T" + <Next Point On The Trajectory>

        PID gains of every controller mode: "K" + <Motor Index> + <Kp1> + # + <Ki1> + # + <Kd1> + # + <Kp2> + # + <Ki2>
+ # + <Kd2> + # + <Kp3> + # + <Ki3> + # + <Kd3>


Haptic device status: "H" + <Motor Index> + <Encoder Pos>
+ SERIAL_SEPERATOR_CH + <Encoder Speed> => "H1123#123" (123 pos 123 speed)

Waveform values: "W" + <Generator Index> + <Value> => "W110.5" (10.5 value of generator 1)

Force Values:                       "F" +
<Force1> + SERIAL_SEPERATOR_CH + <Force2> + SERIAL_SEPERATOR_CH + <Force3> + SERIAL_SEPERATOR_CH + <Force4> =>
"F0.5#0.5#0.5#0.5"

Net Finger Force:                   "N" + <Net Finger Force>

Desired Finger Force:               "D" + <Desired Finger Force>

Limits (hard or soft) for the motors: "L" + "H" or "S" + <Motor Index> + <Lower Limit> + # + <Upper Limit> =>
"LS01000#2000"

MPU6050 Euler Angles:               "E" + <Yaw> + SERIAL_SEPERATOR_CH + <Roll> + SERIAL_SEPERATOR_CH + <Pitch> =>
"E10.2#10.2#10.2" MPU6050 Quaternions: "Q" + <Q0> + SERIAL_SEPERATOR_CH + <Q1> + SERIAL_SEPERATOR_CH + <Q2> +
SERIAL_SEPERATOR_CH + <Q3> => "Q0.5#0.5#0.5#0.5"
"""*/
void SerialHandler::update()
{
    _printPeriodically(_printFrequency, _debug);
    _receiveNonBlocking();
}

void SerialHandler::setSerial(Stream &serial)
{
    _serial = &serial;
    advancedSerial::setPrinter(serial);
}

void SerialHandler::setEndMarker(char endMarker) { _endMarker = endMarker; }

void SerialHandler::setStartMarker(char startMarker) { _startMarker = startMarker; }

void SerialHandler::setSeperator(char seperator) { _separator = seperator; }

void SerialHandler::_receiveNonBlocking()
{
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char rc;

    if (_serial->available() <= 0)
        return;

    rc = _serial->read();
    if (recvInProgress == true)
    {
        if (rc != _endMarker)
        {
            _receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= _numChars)
            {
                this->pln("Buffer Overflow");
                ndx = _numChars - 1;
            }
        }
        else
        {
            // digitalWrite(13, !digitalRead(13));
            _receivedChars[ndx] = '\0'; // terminate the string when the end marker arrives
            recvInProgress = false;
            ndx = 0;
            this->parseString(_receivedChars);
        }
    }

    else if (rc == _startMarker)
    {
        recvInProgress = true;
    }
}

void SerialHandler::parseString(char *string)
{
    const char seperator[2] = {_separator, '\0'};
    char messageType = string[0];
    // Remove the first character from the string using memmove. First character is the message type.
    memmove(string, string + 1, strlen(string));

    switch (messageType)
    {
    }
}
void SerialHandler::printGains()
{
}

void SerialHandler::_printPeriodically(float freq, bool debug = false)
{
    if (freq <= 0)
        return;
    // Guard close to when the next print should happen
    if (millis() - _periodicTimer < 1000.0 / freq)
        return;
    _periodicTimer = millis();

    if (debug)
    {
        // Interrupts are disabled while accessing the device data
        noInterrupts();
        // Store the data here
        interrupts();
        // Print here
    }
    else
    {
        noInterrupts(); // Interrupts are disabled while accessing the device data
        // Store the data here
        interrupts(); // Interrupts are enabled again
        // Print here
    }
    this->pln();
}

void SerialHandler::setPrintFrequency(float freq) { _printFrequency = freq; }
void SerialHandler::setDebug(bool debug) { _debug = debug; }

Stream &SerialHandler::getSerial()
{
    Stream *serial = &*_serial;
    return *serial;
}