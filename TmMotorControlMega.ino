/********************************************************************************
 *          D C   T r e a d m i l l   M o t o r   C o n t r o l l e r           *
 *                                                                              *
 *          Todd Lumpkin                                                        *
 *          November 2019                                                       *
 ********************************************************************************/

#define DEBUG  
#include "TmMotorControlMega.h"

///////////////////////////////////////////////////////////////////////////////
//  V a r i a b l e s
///////////////////////////////////////////////////////////////////////////////

//
// Variables accessed by interrupts.  Any multi-byte variable accesses from the main 
// loop need to be preceded by 'noInterrupts()' (to disable interrupts) and followed by 
// 'interrupts()' (to enable interrupts).
//
// Sort of like this:
//
//  noInterrupts();
//  fooCopy = foo;
//  interrupts();
// 
static volatile int rpmPulseCounter = 0;         // only used by RPM interrupts
static volatile int rpmMeasurement = 0;          // RPM. Main loop read-only with interrupts disabled!
static volatile byte fwdRunSwitchState = LOW;    // Foward run switch debounced state. Main loop read-only
static volatile byte revRunSwitchState = LOW;    // Reverse run switch debounced state. Main loop read-only
static volatile byte thermMonState = LOW;        // Thermal monitor input debounced state. Main loop read-only

///////////////////////////////////////////////////////////////////////////////
//  S t a t e   M a c h i n e   I m p l e m e n t a t i o n
///////////////////////////////////////////////////////////////////////////////

//
//  State machine state definitions
//

// Off State
STATE_DEFINE(TmMotorControlSM, Off, RunData)
{
  DebugPrintln("TmMotorControlSM::ST_Off");
  // Turn motor off
  setSpeedLevel(0);
}

// Run State
ENTRY_DEFINE(TmMotorControlSM, RunEntry, RunData)
{
  DebugPrintln("TmMotorControlSM::EN_RunEntry");
  m_lastSpeedLevel = 0; // this will force the run LED on in the state function

  // Set LED cycle time.  For forward we want the LED
  // to stay on.  For reverse we want it to blink.
  switch (data->m_switchState)
  {
    case SWITCH_FWD:
      // 0 means solid on
      m_runLedCycleMs = 0; 
      break;
    case SWITCH_REV:
      m_runLedCycleMs = runLedRevCycleMs;
      break;
    default:
      // This should never happen!!!
      InternalEvent(ST_FAULT, new FaultData(SOFTWARE_FAULT));
      break;
  }

  // Make sure the run LED starts in the off state
  turnRunLedOff();
}

STATE_DEFINE(TmMotorControlSM, Run, RunData)
{
  DebugPrintln("TmMotorControlSM::ST_Run");
  
  // Sanity check speed and switch
  if (data->m_speedLevel > 1023 || data->m_speedLevel < 0 || data->m_switchState == SWITCH_OFF)
  {
    // This should never happen!!!
    InternalEvent(ST_FAULT, new FaultData(SOFTWARE_FAULT));
  }

  setSpeedLevel(data->m_speedLevel);

  if (m_lastSpeedLevel == 0 && data->m_speedLevel > 0) 
  {
    // off to on
    turnRunLedOn(m_runLedCycleMs);
  }
  else if (m_lastSpeedLevel > 0 && data->m_speedLevel == 0)
  {
    // on to off
    turnRunLedOff();
  }
  m_lastSpeedLevel = data->m_speedLevel;
}

EXIT_DEFINE(TmMotorControlSM, RunExit)
{
  DebugPrintln("TmMotorControlSM::EX_RunExit");
  // Turn motor off
  setSpeedLevel(0);
  turnRunLedOff();
}

// Stop State
ENTRY_DEFINE(TmMotorControlSM, StopEntry, RunData)
{
  DebugPrintln("TmMotorControlSM::EN_StopEntry");

  // Sanity check

  if (data->m_switchState != SWITCH_OFF)
  {
    // This should never happen!!!
    InternalEvent(ST_FAULT, new FaultData(SOFTWARE_FAULT));
  }

  turnRunLedOn(runLedOffCycleMs);
}

STATE_DEFINE(TmMotorControlSM, Stop, RunData)
{
  DebugPrintln("TmMotorControlSM::ST_Stop");
  delay(3000);
  InternalEvent(ST_OFF, new RunData(data->m_switchState, data->m_speedLevel));
}

EXIT_DEFINE(TmMotorControlSM, StopExit)
{
  DebugPrintln("TmMotorControlSM::EX_StopExit");
  turnRunLedOff();
}

// Fault State
STATE_DEFINE(TmMotorControlSM, Fault, FaultData)
{
  DebugPrintln("TmMotorControlSM::ST_Fault");

  //
  // Shut 'er down Clancy she's pumpin' mud!!!
  //
  setSpeedLevel(0);
  turnRunLedOff();
  Timer4.detachInterrupt();
  Timer5.detachInterrupt();
  turnFaultLedOn(faultLedCycleMs[data->m_faultType]);

  // Wait here forever, effectively halting the processing of the Arduino.  
  // Power cycle required to recover from fault.
  while(1);
}


//
//  State machine events
//

void TmMotorControlSM::SwitchChangeEvent(RunData* data)
{
  DebugPrintln("TmMotorControlSM::SwitchChangeEvent");

  // If we are OFF then a switch change transitions us to the 
  // RUN state.  If we are in the RUN state then a switch change
  // transitions us to the STOP state where we will delay until
  // the motor stops, and then automatically transitions to the
  // OFF state.  Otherwise, we risk burning out the MOSFET on 
  // the MC-2100 controller board if we transition to RUN again
  // while the motor is turning!!!

  BEGIN_TRANSITION_MAP                         // Current State
    TRANSITION_MAP_ENTRY(ST_RUN)               // ST_OFF
    TRANSITION_MAP_ENTRY(ST_STOP)              // ST_RUN
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)        // ST_STOP
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)        // ST_FAULT
  END_TRANSITION_MAP(data)
}

void TmMotorControlSM::SpeedChangeEvent(RunData* data)
{
  DebugPrintln("TmMotorControlSM::SpeedChangeEvent");

  // We only care about the speed changing if we are in the
  // RUN state.
  BEGIN_TRANSITION_MAP                         // Current State
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)        // ST_OFF
    TRANSITION_MAP_ENTRY(ST_RUN)               // ST_RUN
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)        // ST_STOP
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)        // ST_FAULT
  END_TRANSITION_MAP(data)
}

void TmMotorControlSM::FaultDetectedEvent(FaultData* data)
{
  DebugPrintln("TmMotorControlSM::FaultDetectedEvent");

  // Faults can be detected in any state
  BEGIN_TRANSITION_MAP                         // Current State
    TRANSITION_MAP_ENTRY(ST_FAULT)             // ST_OFF
    TRANSITION_MAP_ENTRY(ST_FAULT)             // ST_RUN
    TRANSITION_MAP_ENTRY(ST_FAULT)             // ST_STOP
    TRANSITION_MAP_ENTRY(EVENT_IGNORED)        // ST_FAULT
  END_TRANSITION_MAP(data)
}

///////////////////////////////////////////////////////////////////////////////
//  H e l p e r   F u n c t i o n s
///////////////////////////////////////////////////////////////////////////////

byte debounceInput(byte debouncePin, volatile byte& currentState, byte& lastReading, unsigned long& lastTimeMs)
{
  // Read the input pin
  byte currentReading = digitalRead(debouncePin);

  if (currentReading != lastReading) 
  { 
    // Pin state just changed.  Reset the time and record the last reading value.
    lastTimeMs = millis(); 
    lastReading = currentReading;
  }
  else if (currentState != currentReading && ((millis() - lastTimeMs) >= debounceHoldMs))
  {
    // 
    // If we got here then we know:
    //
    // 1) current reading equals the last reading, 
    // 2) the current state does not equal the current reading, and 
    // 3) the curent reading has been stable for at least debounceHoldMs,
    //
    // Therefore we need to report a new current state
    //
    currentState = currentReading;
  }
  else
  {
    // nothing
  }

  // The currentState is returned via the currentState reference and as a return
  // value.  You pick.
  return currentState;
}

void turnRunLedOn(unsigned long runLedCycleMs = 0)
{
  if (runLedCycleMs == 0)
  {
    // LED is solid
    Timer3.detachInterrupt(); // just in case
    digitalWrite(runLedPin, HIGH);
  }
  else
  {
    // LED is blinking
    digitalWrite(runLedPin, LOW);
    Timer3.attachInterrupt(blinkRunLedISR, runLedCycleMs * 1000); 
  }
}

void turnRunLedOff(void)
{
  Timer3.detachInterrupt();
  digitalWrite(runLedPin, LOW);
}

void turnFaultLedOn(unsigned long faultLedCycleMs = 0)
{
  if (faultLedCycleMs == 0)
  {
    // LED is solid
    Timer3.detachInterrupt(); // just in case
    digitalWrite(faultLedPin, HIGH);
  }
  else
  {
    // LED is blinking
    digitalWrite(faultLedPin, LOW);
    Timer3.attachInterrupt(blinkFaultLedISR, faultLedCycleMs * 1000); 
  }
}

int smoothAnalog(int reading) 
{
  const int numSamples = 10;
  static int samples[numSamples];
  static int sampleIndex = 0;
  static int sampleSum = 0;

  //
  // Analogs are a 10-bit value (0-1023), therefore we could
  // average up to 32 values with a signed int.
  // NOTE: It will take 10 measurements to fill the sample
  // array and return a true smoothed value.  This is not a
  // problem for our application and not worth compensating for.
  //
  
  // Update sum
  sampleSum -= samples[sampleIndex];
  samples[sampleIndex] = reading;
  sampleSum += samples[sampleIndex++];
  sampleIndex = sampleIndex % numSamples;

  // Return average of last numSamples measurements
  return sampleSum/numSamples;
}

int getSpeedLevel(void)
{
  static int potValue = 0; // persist pot value for delta comparison
  int potReading;
  int speedLevel;

  // Read and condition pot value
  potReading = smoothAnalog(analogRead(potPin));
  if (potReading <= potDeltaThresh)
  {
    potValue = 0;
  }
  else
  {
    if (abs(potReading - potValue) > potDeltaThresh) 
    { 
      // Only accept new value if it's far enough from the current accepted value
      potValue = potReading;
    }
  }

  // Convert Pot input to pwm level to send to MC-2100
  speedLevel = map(potValue, 0, 1023, minDuty, maxDuty);
  if (speedLevel == minDuty) speedLevel = 0; // Force off
 
  return speedLevel; 
}

void setSpeedLevel(int speedLevel)
{
  // Set PWM duty cycle   
  Timer1.setPwmDuty(pwmPin, speedLevel);
}

RunSwitchStateEnum getRunSwitchState(void)
{
  // There is a double throw switch corresponding to 2 switch
  // inputs into the Arduino that are active low.  Since it 
  // would be physically impossible for both inputs to be 
  // low (unless there is a short somewhere) we will report 
  // that as a fault.

  RunSwitchStateEnum switchState = SWITCH_INVALID;

  if (fwdRunSwitchState == HIGH && revRunSwitchState == HIGH) 
  { 
    switchState = SWITCH_OFF;
  }
  else if (fwdRunSwitchState == LOW && revRunSwitchState == HIGH) 
  {
    switchState = SWITCH_FWD;
  }
  else if (fwdRunSwitchState == HIGH && revRunSwitchState == LOW) 
  {
    switchState = SWITCH_REV;
  }
  else if (fwdRunSwitchState == LOW && revRunSwitchState == LOW) 
  {
    switchState = SWITCH_INVALID;
  }
  else
  {
    // ain't no other possibilities
  }
  return switchState;
}

int getRpmMeasurement()
{
  // Disable interrupts to read the RPM measurement in order to prevent
  // corruption.
  noInterrupts();
  int rpmCopy = rpmMeasurement;
  interrupts();
  return rpmCopy;
}

#ifdef DEBUG
void printRunData(RunData* data)
{
  DebugPrint("Switch State=");
  switch (data->m_switchState)
  {
    case SWITCH_OFF:
      DebugPrintln("OFF");
      break;
    case SWITCH_FWD:
      DebugPrintln("FWD");
      break;
    case SWITCH_REV:
      DebugPrintln("REV");
      break;
    case SWITCH_INVALID:
      DebugPrintln("FAULT");
      break;
    default:
      DebugPrintln("*** UNDEFINED ***");
      break;
  }
  DebugPrint("Speed Level=");
  DebugPrintln(data->m_speedLevel);
} 

void printFaultData(FaultData* data)
{
  DebugPrint("Fault Type=");
  switch (data->m_faultType)
  {
    case SWITCH_FAULT:
      DebugPrintln("SWITCH_FAULT");
      break;
    case THERMAL_FAULT:
      DebugPrintln("THERMAL_FAULT");
      break;
    case SOFTWARE_FAULT:
      DebugPrintln("SOFTWARE_FAULT");
      break;
    default:
      DebugPrintln("*** UNDEFINED ***");
      break;
  }
}
#else
  #define printRunData(_X)
  #define printFaultData(_X)
#endif

///////////////////////////////////////////////////////////////////////////////
//  I n t e r r u p t   S e r v i c e   R o u t i n e s
///////////////////////////////////////////////////////////////////////////////

void blinkRunLedISR(void)
{
  static byte runLedState = LOW;

  // Flip the state and write new one
  runLedState = !runLedState;
  digitalWrite(runLedPin, runLedState);
}

void blinkFaultLedISR(void)
{
  static byte faultLedState = LOW;

  // Flip the state and write new one
  faultLedState = !faultLedState;
  digitalWrite(faultLedPin, faultLedState);
}

void debounceInputsISR(void)
{
  // Static storage for last readings
  static byte lastFwdRunReading = fwdRunSwitchState;
  static byte lastRevRunReading = revRunSwitchState;
  static byte lastThermMonReading = thermMonState;

  // Static storage for last times
  static unsigned long lastFwdRunTimeMs = millis();
  static unsigned long lastRevRunTimeMs = millis();
  static unsigned long lastThermMonTimeMs = millis();

  // Process Fwd Run switch input
  debounceInput(fwdRunPin, fwdRunSwitchState, lastFwdRunReading, lastFwdRunTimeMs);

  // Process Rev Run switch input
  debounceInput(revRunPin, revRunSwitchState, lastRevRunReading, lastRevRunTimeMs);

  // Process Thermal Monitor input.  Note: this input is not from a mechanical switch 
  // but we will debounce anyway to take care of potential noise.
  debounceInput(thermMonPin, thermMonState, lastThermMonReading, lastThermMonTimeMs);
}

void rpmPulseISR(void)
{
  // Triggers on falling edge of the pulse from the hall effect sensor
  rpmPulseCounter++;
}

void rpmCalcISR(void)
{
  const int numRpmSamples = 3;
  static int rpmSamples[numRpmSamples];
  static int sampleIndex = 0;
  static int rpmSum = 0;

  // Accumulate this second's pulses into the array
  rpmSum -= rpmSamples[sampleIndex];
  rpmSamples[sampleIndex] = rpmPulseCounter;
  rpmSum += rpmSamples[sampleIndex++];
  sampleIndex %= numRpmSamples;

  // Average the number of pulses for the last 'numRpmSamples' seconds and convert to RPM
  rpmMeasurement = (rpmSum*60)/numRpmSamples; // convert rotations per second to 
                                              // rotations per minute
  
  // Reset pulse counter to accumulate for next second.  Note that
  // interrupts are masked so we don't have to worry about getting
  // interrupted by the rpmPulseISR.
  
  rpmPulseCounter = 0;
}

///////////////////////////////////////////////////////////////////////////////
//  I n i t i a l i z e
///////////////////////////////////////////////////////////////////////////////

//
// This is run once by the Arduino infrastructure.
//

void setup()
{
  //
  // Setup Inputs
  //

  // Configure Input Pins
  pinMode(potPin, INPUT);  // Analog
  pinMode(rpmPulsePin, INPUT_PULLUP);
  pinMode(fwdRunPin, INPUT_PULLUP); 
  pinMode(revRunPin, INPUT_PULLUP);
  pinMode(thermMonPin, INPUT_PULLUP);
  
  // Initialize switch states
  fwdRunSwitchState = digitalRead(fwdRunPin);
  revRunSwitchState = digitalRead(revRunPin);
  thermMonState = digitalRead(thermMonPin);

  //
  // Outputs
  //

  // Configure Output Pins
  pinMode(pwmPin, OUTPUT);
  pinMode(runLedPin, OUTPUT);
  pinMode(faultLedPin, OUTPUT);

  // Initialize Outputs
  digitalWrite(pwmPin, LOW);
  digitalWrite(runLedPin, LOW);
  digitalWrite(faultLedPin, LOW);

  //
  // Timers and interrupts
  //

  // Timer 1: PWM output
  Timer1.initialize(pwmPeriodMs * 1000);      // Set Timer1 period to 50 ms for PWM output
  Timer1.pwm(pwmPin, 0);                      // Setup Timer 1 PWM at 0% duty cycle

  // Timer 3: LED Control
  Timer3.initialize();                        // Initialize LED timer

  // Timer 4: Debounce Timer
  Timer4.initialize(debounceCycleMs * 1000);  // Set debounce cycle time

  // Timer 5: RPM Timer
  Timer5.initialize(1000L * 1000L);           // Set 1s timer for RPM calculation

  //
  // LED test
  //

  digitalWrite(runLedPin, HIGH);
  digitalWrite(faultLedPin, HIGH);
  delay(1500);
  digitalWrite(runLedPin, LOW);
  digitalWrite(faultLedPin, LOW);

  //
  // Git 'er done (attach interrupts)
  //
  // attachInterrupt(digitalPinToInterrupt(rpmPulsePin), rpmPulseISR, FALLING);
  Timer4.attachInterrupt(debounceInputsISR);  // Debounce ISR
  Timer5.attachInterrupt(rpmCalcISR);         // RPM calculation ISR  
  DebugInit(9600);
}


///////////////////////////////////////////////////////////////////////////////
//  E x e c u t e
///////////////////////////////////////////////////////////////////////////////

//
// This is the main processing loop called repeatedly by the Arduino infrastructure.
// It only generates events to the state machine.  Actions (if any) in response to 
// those events are determined by the current state machine state.
//

void loop()
{
  // Instantiate State Machine
  static TmMotorControlSM tmMotorControlSM;
  
  // Speed processing variables
  int currentSpeedLevel;
  static int lastSpeedLevel = 0;

  // Switch processing variables
  RunSwitchStateEnum currentRunSwitchState;
  static RunSwitchStateEnum lastRunSwitchState = SWITCH_OFF;

  // Get the speed and switch state
  currentSpeedLevel = getSpeedLevel();
  currentRunSwitchState = getRunSwitchState();

  // Determine if we need to generate a switch change event
  if (currentRunSwitchState != lastRunSwitchState)
  {
    if (currentRunSwitchState != SWITCH_INVALID && (lastRunSwitchState == SWITCH_OFF || currentRunSwitchState == SWITCH_OFF))
    {
      RunData* pSwitchData;
      // We either went OFF to ON or ON to OFF (regardless of direction)
      DebugPrintln("Switch Event");
      pSwitchData = new RunData(currentRunSwitchState, currentSpeedLevel);
      tmMotorControlSM.SwitchChangeEvent(pSwitchData);
      lastRunSwitchState = currentRunSwitchState;
    }
    else // We had a switch fault or we had an illegal switch transition from fwd to rev or rev to fwd
    {
      DebugPrintln("Switch Fault");
      tmMotorControlSM.FaultDetectedEvent(new FaultData(SWITCH_FAULT));
    }
  }

  // Determine if we need to generate a speed change event
  if (currentSpeedLevel != lastSpeedLevel)
  {
    DebugPrint("currentSpeedLevel = ");
    DebugPrintln(currentSpeedLevel);
    tmMotorControlSM.SpeedChangeEvent(new RunData(currentRunSwitchState, currentSpeedLevel));
    lastSpeedLevel = currentSpeedLevel;
  }

  // Determine if we had a thermal event (motor overheated)
  if (thermMonState == HIGH)
  {
    tmMotorControlSM.FaultDetectedEvent(new FaultData(THERMAL_FAULT));
  }

  // Delay (mainly to prevent reading analogs too fast)
  delay(loopCycleMs);
}
