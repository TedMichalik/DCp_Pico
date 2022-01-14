/**********************************************************************
Config.h
  Part of DC+ BASE STATION for the Raspberry Pi Pico
  Ted Michalik 2022
  
  A derivative work of
  Geoff Bunza 2019  Rev 2.1
  COPYRIGHT (c) 2013-2015 Gregg E. Berman
**********************************************************************/

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE MOTOR_SHIELD_TYPE ACCORDING TO THE FOLLOWING TABLE:
//
//  0 = L298N CONTROLLER          (MAX 18V/2A PER CHANNEL)

#define MOTOR_SHIELD_TYPE   0

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE NUMBER OF MAIN TRACK REGISTER

#define MAX_MAIN_REGISTERS 12

/////////////////////////////////////////////////////////////////////////////////////
//
// DEFINE COMMUNICATIONS INTERFACE
//
//  0 = Built-in Serial Port

#define COMM_INTERFACE   0

// ******** REMOVE THE "//" IN THE FOOLOWING LINE TO SEND DEBUGGING
// ******** INFO TO THE SERIAL MONITOR

#define DEBUG
