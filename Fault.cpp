#include "Fault.h"
#include "TmMotorControlMega.h"

//----------------------------------------------------------------------------
// FaultHandler
//----------------------------------------------------------------------------
void FaultHandler(const char* file, unsigned short line)
{
  // shut 'er down she's pumpin' mud
  Serial.println(F("Software Fault"));
  turnRunLedOff();
  setSpeedLevel(0);
  turnFaultLedOn(faultLedCycleMs[SWITCH_FAULT]);
  while(1);
}
