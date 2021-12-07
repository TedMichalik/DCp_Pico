//  DCp_Pico
//  Part of DC+ BASE STATION for the Raspberry Pi Pico
//
//  A dreivative work of
//  Geoff Bunza 2019  Rev 2.1
//  COPYRIGHT (c) 2013-2015 Gregg E. Berman
//  Part of DC+ BASE STATION for the Arduino//
// DC+ BASE STATION

#include "DCp_Pico.h"
#include "PacketRegister.h"
#include "CurrentMonitor.h"
#include "Sensor.h"
#include "SerialCommand.h"
#include "Accessories.h"
#include "EEStore.h"
#include "Config.h"

// DECLARE GLOBAL OBJECTS TO PROCESS AND STORE DCC PACKETS AND MONITOR TRACK CURRENTS.
// NOTE REGISTER LISTS MUST BE DECLARED WITH "VOLATILE" QUALIFIER TO ENSURE THEY ARE PROPERLY UPDATED BY INTERRUPT ROUTINES

volatile RegisterList mainRegs(MAX_MAIN_REGISTERS);    // create list of registers for MAX_MAIN_REGISTER Main Track Packets
volatile RegisterList progRegs(2);                     // create a shorter list of only two registers for Program Track Packets

char msgA[5] = "<p2>";
char msgB[5] = "<p3>";
CurrentMonitor mainMonitor(CURRENT_MONITOR_PIN_MAIN,msgA);  // create monitor for current on Track A
CurrentMonitor progMonitor(CURRENT_MONITOR_PIN_PROG,msgB);  // create monitor for current on Track B

bool track_power = true;  // global power indicator variable
int toggle = HIGH;        // used to toggle the onboard LED
int x = 0;                // used for frequency of toggle

///////////////////////////////////////////////////////////////////////////////
// PRINT CONFIGURATION INFO TO UART SERIAL PORT

void showConfiguration(){

  Serial1.print("<iDC+ BASE STATION FOR  ");      // Print Status to UART Serial Line to check configuration 
  Serial1.print(ARDUINO_TYPE);
  Serial1.print(" / ");
  Serial1.print(MOTOR_SHIELD_NAME);
  Serial1.print(": BUILD ");
  Serial1.print(__DATE__);
  Serial1.print(" ");
  Serial1.print(__TIME__);
  Serial1.print(">");
  Serial1.print("<N");
  Serial1.print(COMM_TYPE);
  Serial1.print(": ");
  Serial1.println("SERIAL>");
} // showConfiguration

///////////////////////////////////////////////////////////////////////////////
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);            // configure USB serial interface
  Serial.flush();

#ifdef DEBUG
  Serial1.begin(115200);           // Initialise UART 0
  Serial1.flush();
#endif

  // TRACK A
  analogWrite(ENA, 0);    // Set speed to 0
  pinMode(IN1, OUTPUT);   // Both outputs LOW to stop
  digitalWrite(IN1, LOW);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN2, LOW);
  // TRACK B
  analogWrite(ENB, 0);    // Set speed to 0
  pinMode(IN3, OUTPUT);   // Both outputs LOW to stop
  digitalWrite(IN3, LOW);
  pinMode(IN4, OUTPUT);
  digitalWrite(IN4, LOW);

  EEStore::init();                                           // initialize and load Turnout and Sensor definitions stored in EEPROM

  SerialCommand::init(&mainRegs, &progRegs, &mainMonitor);   // create structure to read and parse commands from serial line

#ifdef DEBUG
  showConfiguration();
#endif

} // setup

///////////////////////////////////////////////////////////////////////////////
// the loop function runs over and over again forever
void loop() {

  SerialCommand::process();             // check for, and process, and new serial commands
  
  if(CurrentMonitor::checkTime()){      // if sufficient time has elapsed since last update, check current draw on Main and Program Tracks 
    mainMonitor.check();
    progMonitor.check();

    if(++x > 100){
      digitalWrite(LED_BUILTIN, toggle);  // toggle the LED
      toggle = 1 - toggle;                // toggle the variable ( 1 - 1(HIGH) = 0(LOW) )
                                          //                     ( 1 - 0(LOW) = 1(HIGH) )
      x = 0;
    }
  }
  Sensor::check();    // check sensors for activate/de-activate
  
} // loop
