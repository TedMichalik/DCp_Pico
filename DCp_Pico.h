/**********************************************************************
DCp_Pico.h
  Part of DC+ BASE STATION for the Raspberry Pi Pico
  Ted Michalik 2022
  
  A derivative work of
  Geoff Bunza 2019  Rev 2.1
  COPYRIGHT (c) 2013-2015 Gregg E. Berman
**********************************************************************/

#include "Config.h"

#ifndef DCp_Pico_h
#define DCp_Pico_h

/////////////////////////////////////////////////////////////////////////////////////
// RELEASE VERSION
/////////////////////////////////////////////////////////////////////////////////////

#define VERSION "3.1"

/////////////////////////////////////////////////////////////////////////////////////
// BOARD SETUP - Pin numbers are Pico GP numbers, not physical pin numbers.
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
  
  //Function Pins
  #define tim_delay 500
  #define numfpins 12     // Pins 2 thru 13
							//Note: Maximum value of 13 (pins 2 thru 14) without rewriting code.

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

#endif
