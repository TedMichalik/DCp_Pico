/**********************************************************************

CurrentMonitor.h
COPYRIGHT (c) 2013-2016 Gregg E. Berman

Part of DCC++ BASE STATION for the Raspberry Pi Pico

**********************************************************************/

#ifndef CurrentMonitor_h
#define CurrentMonitor_h

#include "Arduino.h"

#define  CURRENT_SAMPLE_SMOOTHING   0.01
#define  CURRENT_SAMPLE_MAX         900
#define  CURRENT_SAMPLE_TIME        10

struct CurrentMonitor{
  static long int sampleTime;
  int pin;
  float current;
  char *msg;
  CurrentMonitor(int, char *);
  static boolean checkTime();
  void check();
};

#endif
