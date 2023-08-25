#include "SerialHandler.h"

/*"""

 Serial Message Format:
 Every message sent and received over the serial should start with a '_startMarker' and end with an '_endMarker'.
 First character (after the start character) of the message is the message type for both sending and reception.
 The message type is followed by the message payload.
 Different values inside the payload are separated by a '_separator'.
 SERIAL_SEPERATOR_CH = '#'
_separator = '#'
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
    Serial.println(string);
    const char seperator[2] = {_separator, '\0'};
    char messageType = string[0];
    // Remove the first character from the string using memmove. First character is the message type.
    memmove(string, string + 1, strlen(string));

    switch (messageType)
    {
    case 'V':
        // Parse the message in the following format: <V0.4#0.5#0.5>
        // Start, end and messagetype characters are already removed. So it is 0.4#0.5#0.5
        // First is Vx second is Vy and third is wz
        // Split the string into tokens using the seperator
        // Having a velocity of 1 is full speed        
        char *token = strtok(string, seperator);
        robot->base->setDesiredVx(atof(token));
        
        token = strtok(NULL, seperator);
        robot->base->setDesiredVy(atof(token));
        
        token = strtok(NULL, seperator);
        robot->base->setDesiredWz(atof(token));

        this->pln();
        return;
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
        float w_fl = robot->base->wheel_fl.speed();
        float w_fr = robot->base->wheel_fr.speed();
        float w_rl = robot->base->wheel_rl.speed();
        float w_rr = robot->base->wheel_rr.speed();
        interrupts();
        this->p(w_fl).p(" ").p(w_fr).p(" ").p(w_rl).p(" ").p(w_rr);
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
void SerialHandler::setRobot(Robot *robot) { this->robot = robot; }