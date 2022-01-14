/**********************************************************************

CurrentMonitor.cpp
COPYRIGHT (c) 2013-2016 Gregg E. Berman

Part of DCC++ BASE STATION for the Raspberry Pi Pico

**********************************************************************/

#include "DCp_Pico.h"
#include "CurrentMonitor.h"
#include "Config.h"

extern bool track_power;

///////////////////////////////////////////////////////////////////////////////

CurrentMonitor::CurrentMonitor(int pin, char *msg){
    this->pin=pin;
    this->msg=msg;
    current=0;
  } // CurrentMonitor::CurrentMonitor
  
boolean CurrentMonitor::checkTime(){
  if(millis()-sampleTime<CURRENT_SAMPLE_TIME)            // no need to check current yet
    return(false);
  sampleTime=millis();                                   // note millis() uses TIMER-0.  For UNO, we change the scale on Timer-0.  For MEGA we do not.  This means CURENT_SAMPLE_TIME is different for UNO then MEGA
  return(true);  
} // CurrentMonitor::checkTime
  
void CurrentMonitor::check(){
  current=analogRead(pin)*CURRENT_SAMPLE_SMOOTHING+current*(1.0-CURRENT_SAMPLE_SMOOTHING);        // compute new exponentially-smoothed current
if(current>CURRENT_SAMPLE_MAX && track_power){                               // current overload
#ifdef DEBUG
    Serial1.print("Current = ");
    Serial1.print(current);
    Serial1.print(" ");
    Serial1.println(msg);
	return;
#endif
    digitalWrite(IN1,LOW);                                                     // disable both Motor Control Channels
    digitalWrite(IN2,LOW);                                                     // regardless of which caused current overload
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,LOW);
    track_power = false;
    INTERFACE.print(msg);                                                      // print corresponding error message
  }    
} // CurrentMonitor::check  

long int CurrentMonitor::sampleTime=0;
