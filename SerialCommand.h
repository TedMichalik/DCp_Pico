/**********************************************************************
SerialCommand.h
  Part of DC+ BASE STATION for the Raspberry Pi Pico
  Ted Michalik 2022
  
  A derivative work of
  Geoff Bunza 2019  Rev 2.1
COPYRIGHT (c) 2013-2016 Gregg E. Berman
**********************************************************************/

#ifndef SerialCommand_h
#define SerialCommand_h

#include "CurrentMonitor.h"

#define  MAX_COMMAND_LENGTH         30
#define  MAX_CV                    128

struct SerialCommand{
  static char commandString[MAX_COMMAND_LENGTH+1];
  static CurrentMonitor *mMonitor;
  static void init(CurrentMonitor *);
  static void parse(char *);
  static void process();
  static void check_function();
}; // SerialCommand

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE FUNCTION STRUCTURE
//

struct QUEUE
{
  int inuse;
  int current_position;
  int increment;
  int stop_value;
  int start_value;
};

#endif
