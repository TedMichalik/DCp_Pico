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

#include "PacketRegister.h"
#include "CurrentMonitor.h"

#define  MAX_COMMAND_LENGTH         30

struct SerialCommand{
  static char commandString[MAX_COMMAND_LENGTH+1];
  static volatile RegisterList *mRegs, *pRegs;
  static CurrentMonitor *mMonitor;
  static void init(volatile RegisterList *, volatile RegisterList *, CurrentMonitor *);
  static void parse(char *);
  static void process();
}; // SerialCommand

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE FUNCTION STRUCTURE
//

#define SET_CV_Address       24           // THIS ADDRESS IS FOR SETTING CV'S Like a Loco
#define Accessory_Address    40           // THIS ADDRESS IS THE START OF THE SWITCHES RANGE
                                          // WHICH WILL EXTEND FOR 16 MORE SWITCH ADDRESSES
#define CV_To_Store_SET_CV_Address	121
#define CV_Accessory_Address CV_ACCESSORY_DECODER_ADDRESS_LSB
						  
struct QUEUE
{
  int inuse;
  int current_position;
  int increment;
  int stop_value;
  int start_value;
};

struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};
  
#endif




