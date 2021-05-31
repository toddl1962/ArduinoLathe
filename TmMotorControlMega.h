/********************************************************************************
 *          D C   T r e a d m i l l   M o t o r   C o n t r o l l e r           *
 *                                                                              *
 *          Todd Lumpkin                                                        *
 *          October 2019                                                        *
 ********************************************************************************/

#ifndef _TmMotorControlMega_H
#define _TmMotorControlMega_H
#include <TimerOne.h>       // Used for PWM output
#include <TimerThree.h>     // Used for LED timing
#include <TimerFour.h>      // Used for swich input debounce
#include <TimerFive.h>      // Used for RPM calculation
#include "StateMachine.h"

#ifdef DEBUG
  #define DebugInit(_X) Serial.begin(_X)
  #define DebugPrint(_X) Serial.print(_X)
  #define DebugPrintln(_X) Serial.println(_X)
#else
  #define DebugInit(_X)
  #define DebugPrint(_X)
  #define DebugPrintln(_X)
#endif

///////////////////////////////////////////////////////////////////////////////
//  P i n   D e f i n t i o n s
///////////////////////////////////////////////////////////////////////////////

const int potPin = A0;      // IN: Speed Pot Analog  
const int pwmPin = 11;      // OUT: PWM to treadmill controller.  This MUST be
                            //      on pin 11 to use the Timer1 PWM output.
const int faultLedPin = 7;  // OUT: Fault indicator pin
const int runLedPin = 6;    // OUT: Run indicator pin
const int thermMonPin = 5;  // IN: Thermal shutdowm monitor pin
const int fwdRunPin = 4;    // IN: Forward run switch pin
const int revRunPin = 3;    // IN: Reverse run switch pin
const int rpmPulsePin = 2;  // IN: RPM pulse input.  This MUST be on pin 2
                            //     because it is an external interrupt.

///////////////////////////////////////////////////////////////////////////////
//  C o n s t a n t s
///////////////////////////////////////////////////////////////////////////////

enum RunSwitchStateEnum
{
  SWITCH_OFF,
  SWITCH_FWD,
  SWITCH_REV,
  SWITCH_INVALID
};

enum FaultTypeEnum
{
  SWITCH_FAULT,
  THERMAL_FAULT,
  SOFTWARE_FAULT,
  FAULT_ENUM_SIZE
};

const int loopCycleMs = 10;           // Main loop cycle time in ms.  Rather arbitrary
                                      // but slow enough to prevent reading analogs
                                      // too fast and plenty fast for everything else
const int potDeltaThresh = 4;         // Change detection threshold on pot
const int maxDuty = 800;              // Max Duty Cycle required to clamp RPM to
                                      // reasonable level
const int minDuty = 110;              // Min Duty Cycle required to trigger MC-2100
const int debounceHoldMs = 50;        // Debounce time for switch transitions
const long debounceCycleMs = 2;       // Debounce interrupt cycle
const unsigned long pwmPeriodMs = 50; // Output Signal PWM Period to treadmill 
                                      // controller (50ms)
const unsigned long runLedRevCycleMs = 500;  // Run LED blink cycle for reverse
const unsigned long runLedOffCycleMs = 1000; // Run LED blink cycle for off state
const unsigned long faultLedCycleMs[FAULT_ENUM_SIZE] = {1000, 500, 250};  // switch, thermal, sw

///////////////////////////////////////////////////////////////////////////////
//  S t a t e   M a c h i n e   D e c l a r a t i o n s
///////////////////////////////////////////////////////////////////////////////

class RunData : public EventData
{
  public:
    RunData(RunSwitchStateEnum switchState, int speedLevel):
      m_switchState(switchState),
      m_speedLevel(speedLevel)
    {
    }
    RunSwitchStateEnum m_switchState;
    int m_speedLevel;
};

class FaultData : public EventData
{
  public:
    FaultData(FaultTypeEnum faultType):
      m_faultType(faultType)
    {
    }
    FaultTypeEnum m_faultType;
};


class TmMotorControlSM: public StateMachine
{
  public:
    TmMotorControlSM() :
      StateMachine(ST_MAX_STATES),
      m_lastSpeedLevel(0),
      m_runLedCycleMs(0)
    {
        Serial.println(F("TmMotorControlSM::TmMotorControlSM"));
    }

    // External events taken by this state machine
    void SwitchChangeEvent(RunData* data);
    void SpeedChangeEvent(RunData* data);
    void FaultDetectedEvent(FaultData* data); 

  private:
    int m_lastSpeedLevel;
    unsigned long m_runLedCycleMs;

    // State enumeration order must match the order of state method entries
    // in the state map.
    enum States
    {
      ST_OFF,
      ST_RUN,
      ST_STOP,
      ST_FAULT,
      ST_MAX_STATES
    };

    // Define the state machine state functions with event data type
    STATE_DECLARE(TmMotorControlSM, Off,        RunData)
    ENTRY_DECLARE(TmMotorControlSM, RunEntry,   RunData)
    STATE_DECLARE(TmMotorControlSM, Run,        RunData)
    EXIT_DECLARE(TmMotorControlSM,  RunExit)
    ENTRY_DECLARE(TmMotorControlSM, StopEntry,  RunData)
    STATE_DECLARE(TmMotorControlSM, Stop,       RunData)
    EXIT_DECLARE(TmMotorControlSM,  StopExit)
    STATE_DECLARE(TmMotorControlSM, Fault,      FaultData)

    // State map to define state object order. Each state map entry defines a
    // state object.
    BEGIN_STATE_MAP_EX
      STATE_MAP_ENTRY_ALL_EX(&Off,   0, 0,           0)          // ST_OFF
      STATE_MAP_ENTRY_ALL_EX(&Run,   0, &RunEntry,   &RunExit)   // ST_RUN
      STATE_MAP_ENTRY_ALL_EX(&Stop,  0, &StopEntry,  &StopExit)  // ST_STOP
      STATE_MAP_ENTRY_ALL_EX(&Fault, 0, 0,           0)          // ST_FAULT
    END_STATE_MAP_EX
};
byte debounceInput(byte debouncePin, volatile byte& currentState, byte& lastReading, unsigned long& lastTimeMs);
void turnRunLedOn(unsigned long runLedCycleMs);
void turnRunLedOff(void);
void turnFaultLedOn(const unsigned long faultLedCycleMs);
int getSpeedLevel(void);
void setSpeedLevel(int speedLevel);
RunSwitchStateEnum getRunSwitchState(void);
int getRpmMeasurement();
void printRunData(RunData* data);
void printFaultData(FaultData* data);

void blinkRunLedISR(void);
void blinkFaultLedISR(void);
void debounceInputsISR(void);
void rpmPulseISR(void);
void rpmCalcISR(void);


#endif // ifndef _TmMotorControlMega_H
