/**********************************************************************

Config.h
COPYRIGHT (c) 2013-2016 Gregg E. Berman

Part of DCC++ BASE STATION for the Raspberry Pi Pico

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
