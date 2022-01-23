//  DCp_Pico
//  Part of DC+ BASE STATION for the Raspberry Pi Pico
//  Ted Michalik 2022
//
//  A dreivative work of
//  Geoff Bunza 2019  Rev 2.1
//  COPYRIGHT (c) 2013-2015 Gregg E. Berman
// DC+ BASE STATION

#include "DCp_Pico.h"
#include "CurrentMonitor.h"
#include "Sensor.h"
#include "SerialCommand.h"
#include "Accessories.h"
#include "EEStore.h"
#include "Config.h"

// DECLARE GLOBAL OBJECTS TO MONITOR TRACK CURRENTS.
// NOTE REGISTER LISTS MUST BE DECLARED WITH "VOLATILE" QUALIFIER TO ENSURE THEY ARE PROPERLY UPDATED BY INTERRUPT ROUTINES

char msgA[5] = "<p2>";
char msgB[5] = "<p3>";
CurrentMonitor mainMonitor(CURRENT_MONITOR_PIN_MAIN,msgA);  // create monitor for current on Track A
CurrentMonitor progMonitor(CURRENT_MONITOR_PIN_PROG,msgB);  // create monitor for current on Track B

bool track_power = true;  // global power indicator variable
int toggle = HIGH;        // used to toggle the onboard LED
int x = 0;                // used for frequency of toggle
int fpins [] = {F0pin,F1pin,F2pin,F3pin,F4pin,F5pin,F6pin,F7pin,F8pin,F9pin,F10pin,F11pin};

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

  SerialCommand::init(&mainMonitor);   // create structure to read and parse commands from serial line

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

  int i;
  uint8_t cv_value;
// Initialize the Function pins as outputs
  for (int i=0; i < numfpins; i++) {
     pinMode(fpins[i], OUTPUT);
     digitalWrite(fpins[i], 0);
  }
  for (int i=0; i < numfpins; i++) {
     digitalWrite(fpins[i], 1);
     delay (tim_delay/10);
  }
  delay( tim_delay);
  for (int i=0; i < numfpins; i++) {
     digitalWrite(fpins[i], 0);
     delay (tim_delay/10);
  }
  delay( tim_delay);
  
  EEStore::init();                                           // initialize and load Turnout and Sensor definitions stored in EEPROM

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
	
	SerialCommand::check_function();		// 

    if(++x > 100){
      digitalWrite(LED_BUILTIN, toggle);  // toggle the LED
      toggle = 1 - toggle;                // toggle the variable ( 1 - 1(HIGH) = 0(LOW) )
                                          //                     ( 1 - 0(LOW) = 1(HIGH) )
      x = 0;
    }
  }
  Sensor::check();    // check sensors for activate/de-activate
  
} // loop
