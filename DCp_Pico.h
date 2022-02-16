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
// RELEASE VERSION - Based on latest DCC-EX CommandStation-EX
/////////////////////////////////////////////////////////////////////////////////////

#define VERSION "3.2.0 rc11"

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
  #define numfpins 12     // Pins 2 thru 13 (F0 - F11)
							//Note: Maximum value of 13 (pins 2 thru 14) without rewriting code.
  #define F0pin 2
  #define F1pin 3
  #define F2pin 4
  #define F3pin 5
  #define F4pin 6
  #define F5pin 7
  #define F6pin 8
  #define F7pin 9
  #define F8pin 10
  #define F9pin 11
  #define F10pin 12
  #define F11pin 13
  

#else

  #error CANNOT COMPILE - DC+ ONLY WORKS WITH A RASPBERRY PI PICO

#endif

/////////////////////////////////////////////////////////////////////////////////////
// SELECT COMMUNICATION INTERACE
/////////////////////////////////////////////////////////////////////////////////////

#if COMM_INTERFACE == 0
  // USB was selected
  #define INTERFACE Serial
  #define DEBUG_OUT Serial1

#else
  // UART was selected
  #define INTERFACE Serial1
  #define DEBUG_OUT Serial

#endif

/////////////////////////////////////////////////////////////////////////////////////

#endif
