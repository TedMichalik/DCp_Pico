/**********************************************************************

DCCpp_Uno.h
COPYRIGHT (c) 2013-2016 Gregg E. Berman

Part of DCC++ BASE STATION for the Arduino

**********************************************************************/

#include "Config.h"

#ifndef DCp_Pico_h
#define DCp_Pico_h

/////////////////////////////////////////////////////////////////////////////////////
// RELEASE VERSION
/////////////////////////////////////////////////////////////////////////////////////

#define VERSION "1.2.1+"

/////////////////////////////////////////////////////////////////////////////////////
// BOARD SETUP
/////////////////////////////////////////////////////////////////////////////////////

#if defined  ARDUINO_RASPBERRY_PI_PICO

  #define ARDUINO_TYPE    "RASPBERRY_PI_PICO"
  #define MOTOR_SHIELD_NAME "L298N"
  #define MAX_THROTTLES  2     // MAX Number Throttles Supported


  //Track A
  #define ENA 17          // PWM Speed (0 - 255)
  #define IN1 18          // Direction (0 - Off/Reverse; 1 - Forward)
  #define IN2 19          // Direction (0 - Off/Forward; 1 - Reverse)
  #define CURRENT_MONITOR_PIN_MAIN A0  // Need external circuit to measure current.
  
  //Track B
  #define ENB 20          // PWM Speed (0 - 255)
  #define IN3 21          // Direction (0 - Off/Reverse; 1 - Forward)
  #define IN4 22          // Direction (0 - Off/Forward; 1 - Reverse)
  #define CURRENT_MONITOR_PIN_PROG A1  // Need external circuit to measure current.

#else

  #error CANNOT COMPILE - DC+ ONLY WORKS WITH A RASPBERRY PI PICO

#endif

/////////////////////////////////////////////////////////////////////////////////////
// SELECT COMMUNICATION INTERACE
/////////////////////////////////////////////////////////////////////////////////////

#if COMM_INTERFACE == 0

  #define COMM_TYPE 0
  #define INTERFACE Serial

#else
  // Serial was not selected

  #error CANNOT COMPILE - DC+ FOR THE PICO CAN ONLY USE SERIAL COMMUNICATION - PLEASE SELECT THIS IN THE CONFIG FILE

#endif

/////////////////////////////////////////////////////////////////////////////////////
// SET WHETHER TO SHOW PACKETS - DIAGNOSTIC MODE ONLY
/////////////////////////////////////////////////////////////////////////////////////

// If SHOW_PACKETS is set to 1, then for select main operations track commands that modify an internal DCC packet register,
// if printFlag for that command is also set to 1, DCC++ BASE STATION will additionally return the 
// DCC packet contents of the modified register in the following format:

//    <* REG: B1 B2 ... Bn CSUM / REPEAT>
//
//    REG: the number of the main operations track packet register that was modified
//    B1: the first hexidecimal byte of the DCC packet
//    B2: the second hexidecimal byte of the DCC packet
//    Bn: the nth hexidecimal byte of the DCC packet
//    CSUM: a checksum byte that is required to be the final byte in any DCC packet
//    REPEAT: the number of times the DCC packet was re-transmitted to the tracks after its iniital transmission
 
#define SHOW_PACKETS  1       // set to zero to disable printing of every packet for select main operations track commands

/////////////////////////////////////////////////////////////////////////////////////

#endif
