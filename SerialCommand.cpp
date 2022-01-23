/**********************************************************************
SerialCommand.cpp
  Part of DC+ BASE STATION for the Raspberry Pi Pico
  Ted Michalik 2022
  
  A derivative work of
  Geoff Bunza 2019  Rev 2.1

COPYRIGHT (c) 2013-2016 Gregg E. Berman
**********************************************************************/

// DCC++ BASE STATION COMMUNICATES VIA THE SERIAL PORT USING SINGLE-CHARACTER TEXT COMMANDS
// WITH OPTIONAL PARAMTERS, AND BRACKETED BY < AND > SYMBOLS.  SPACES BETWEEN PARAMETERS
// ARE REQUIRED.  SPACES ANYWHERE ELSE ARE IGNORED.  A SPACE BETWEEN THE SINGLE-CHARACTER
// COMMAND AND THE FIRST PARAMETER IS ALSO NOT REQUIRED.

// See SerialCommand::parse() below for defined text commands.

#include "SerialCommand.h"
#include "DCp_Pico.h"
#include "Accessories.h"
#include "Sensor.h"
#include "Outputs.h"
#include "EEStore.h"
#include "Config.h"
#include <Servo.h>

Servo servo[numfpins];
#define servo_slowdown  3   //servo loop counter limit
int servo_slow_counter = 0; //servo loop counter to slowdown servo transit

int nReg;
int cab;
int tSpeed;
int tDirection;
int fByte, eByte;
int8_t cv;
int16_t cv_value;
int16_t fungrp;
bool FunctionState[29] = { }; // Functions F0 thru F28, initialized to all off.
uint32_t NewFunctionState;
bool Bit_State;
int startfn;
int stopfn;
int speedTable[MAX_THROTTLES+1] = { };
int MyCVs[MAX_CV] = { };
int t;                                    // temp - Rewrite code for FADE function to eliminate this.

// Create Function queue
QUEUE ftn_queue[numfpins];
  
extern bool track_power;
extern int fpins[numfpins];

///////////////////////////////////////////////////////////////////////////////

char SerialCommand::commandString[MAX_COMMAND_LENGTH+1];
CurrentMonitor *SerialCommand::mMonitor;

///////////////////////////////////////////////////////////////////////////////

void SerialCommand::init(CurrentMonitor *_mMonitor){
  mMonitor=_mMonitor;
  sprintf(commandString,"");
  
  // Initialize CVs
  MyCVs[30] = 2;  //F0 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  MyCVs[31] = 1;    // Rate  Blink=Eate,Servo=Rate
  MyCVs[32] = 28;   //  Start Position F0=0
  MyCVs[33] = 140;  //  End Position   F0=1
  MyCVs[34] = 28;   //  Current Position
  MyCVs[35] = 2;  //F1 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  MyCVs[36] = 1;    // Rate  Blink=Eate,Servo=Rate
  MyCVs[37] = 28;   //  Start Position Fx=0
  MyCVs[38] = 140;  //  End Position   Fx=1
  MyCVs[39] = 28;   //  Current Position
  MyCVs[40] = 2;  //F2 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  MyCVs[41] = 1;    // Rate  Blink=Eate,Servo=Rate
  MyCVs[42] = 28;   //  Start Position Fx=0
  MyCVs[43] = 140;  //  End Position   Fx=1
  MyCVs[44] = 28;   //  Current Position
  MyCVs[45] = 2;  //F3 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  MyCVs[46] = 1;    // Rate  Blink=Eate,Servo=Rate
  MyCVs[47] = 28;   //  Start Position Fx=0
  MyCVs[48] = 140;  //  End Position   Fx=1
  MyCVs[49] = 28;   //  Current Position
  MyCVs[50] = 0;  //F4 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  MyCVs[51] = 1;    // Rate  Blink=Eate,Servo=Rate
  MyCVs[52] = 28;   //  Start Position Fx=0
  MyCVs[53] = 140;  //  End Position   Fx=1
  MyCVs[54] = 28;   //  Current Position
  MyCVs[55] = 0;  //F5 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  MyCVs[56] = 1;    // Rate  Blink=Eate,Servo=Rate
  MyCVs[57] = 28;   //  Start Position Fx=0
  MyCVs[58] = 140;  //  End Position   Fx=1
  MyCVs[59] = 28;   //  Current Position
  MyCVs[60] = 0;  //F6 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  MyCVs[61] = 1;    // Rate  Blink=Eate,Servo=Rate
  MyCVs[62] - 28;   //  Start Position Fx=0
  MyCVs[63] = 140;  //  End Position   Fx=1
  MyCVs[64] = 28;   //  Current Position
  MyCVs[65] = 0;  //F7 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  MyCVs[66] = 1;    // Rate  Blink=Eate,Servo=Rate
  MyCVs[67] = 28;   //  Start Position Fx=0
  MyCVs[68] = 140;  //  End Position   Fx=1
  MyCVs[69] = 28;   //  Current Position
  MyCVs[70] = 1;  //F8 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  MyCVs[71] = 1;    // Rate  Blink=Eate,Servo=Rate
  MyCVs[72] = 1;    //  Start Position Fx=0
  MyCVs[73] = 50;   //  End Position   Fx=1
  MyCVs[74] = 1;    //  Current Position
  MyCVs[75] = 1;  //F9 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  MyCVs[76] = 1;    // Rate  Blink=Eate,Servo=Rate
  MyCVs[77] = 1;    //  Start Position Fx=0
  MyCVs[78] = 50;   //  End Position   Fx=1
  MyCVs[79] = 1;    //  Current Position
  MyCVs[80] = 3;  //F10 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  MyCVs[81] = 1;    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  MyCVs[82] = 1;    //  Start Position Fx=0
  MyCVs[83] = 50;   //  End Position   Fx=1
  MyCVs[84] = 20;   //  Current Position
  MyCVs[85] = 0;  //F11 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  MyCVs[86] = 1;    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  MyCVs[87] = 1;    //  Start Position Fx=0
  MyCVs[88] = 50;    //  End Position   Fx=1
  MyCVs[89] = 1;    //  Current Position
//FUTURE USE
  MyCVs[90] = 0;  //F12 Config  0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  MyCVs[91] = 1;    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  MyCVs[92] = 1;    //  Start Position Fx=0
  MyCVs[93] = 4;    //  End Position   Fx=1
  MyCVs[94] = 1;    //  Current Position

// Initialize Function Queue
for (int i=0; i < numfpins; i++) {
	ftn_queue[i].inuse = 0;
	ftn_queue[i].current_position = 0;
	ftn_queue[i].increment = 0;
	ftn_queue[i].stop_value = 0;
	ftn_queue[i].start_value = 0;
}
} // SerialCommand:SerialCommand

///////////////////////////////////////////////////////////////////////////////

void SerialCommand::process(){
  char c;
    
  #if COMM_TYPE == 0

    while(INTERFACE.available()>0){    // while there is data on the serial line
     c=INTERFACE.read();
     if(c=='<')                    // start of new command
       sprintf(commandString,"");
     else if(c=='>')               // end of new command
       parse(commandString);                    
     else if(strlen(commandString)<MAX_COMMAND_LENGTH)    // if comandString still has space, append character just read from serial line
       sprintf(commandString,"%s%c",commandString,c);     // otherwise, character is ignored (but continue to look for '<' or '>')
    } // while
  
  #elif COMM_TYPE == 1

    EthernetClient client=INTERFACE.available();

    if(client){
      while(client.connected() && client.available()){        // while there is data on the network
      c=client.read();
      if(c=='<')                    // start of new command
        sprintf(commandString,"");
      else if(c=='>')               // end of new command
        parse(commandString);                    
      else if(strlen(commandString)<MAX_COMMAND_LENGTH)    // if comandString still has space, append character just read from network
        sprintf(commandString,"%s%c",commandString,c);     // otherwise, character is ignored (but continue to look for '<' or '>')
      } // while
    }

  #endif

} // SerialCommand:process
   
///////////////////////////////////////////////////////////////////////////////

void SerialCommand::parse(char *com){

#ifdef DEBUG
  Serial1.println(com);
#endif

  switch(com[0]){

/***** SET ENGINE THROTTLES USING 128-STEP SPEED CONTROL ****/    

    case 't':       // <t REGISTER CAB SPEED DIRECTION>
/*
 *    sets the throttle for a given register/cab combination 
 *    
 *    REGISTER: an internal register number, from 1 through MAX_MAIN_REGISTERS (inclusive), to store the DCC packet used to control this throttle setting
 *    CAB:  the short (1-127) or long (128-10293) address of the engine decoder
 *    SPEED: throttle speed from 0-126, or -1 for emergency stop (resets SPEED to 0)
 *    DIRECTION: 1=forward, 0=reverse.  Setting direction when speed=0 or speed=-1 only effects directionality of cab lighting for a stopped train
 *    
 *    returns: <T REGISTER SPEED DIRECTION>
 *    
 */
 //     mRegs->setThrottle(com+1);
      
      if(sscanf(com+1,"%d %d %d %d",&nReg,&cab,&tSpeed,&tDirection)!=4)
        return;

      if((cab<1) || (cab > MAX_THROTTLES))
        return;  

      if(tSpeed<0) 
        tSpeed=0;

      if (track_power)   {
        switch ( cab ) {
          case 1:
             digitalWrite(IN1, tDirection);   // set direction
             digitalWrite(IN2, 1-tDirection);
             analogWrite(ENA, tSpeed<<1);     // set speed
          break;
          case 2:
             digitalWrite(IN3, tDirection);   // set direction
             digitalWrite(IN4, 1-tDirection);
             analogWrite(ENB, tSpeed<<1);     // set speed
          break;
      
          default:
          break;
        }
      }
      else {
        analogWrite(ENA, 0);       // set speed to zero
        analogWrite(ENB, 0);       // set speed to zero
        tSpeed=0;
      }
      INTERFACE.print("<T");
      INTERFACE.print(cab); INTERFACE.print(" ");
      INTERFACE.print(tSpeed); INTERFACE.print(" ");
      INTERFACE.print(tDirection);
      INTERFACE.print(">");
      speedTable[cab]=tDirection==1?tSpeed:-tSpeed;
      break;

/***** OPERATE ENGINE DECODER FUNCTIONS F0-F28 ****/    

    case 'f':       // <f CAB BYTE1 [BYTE2]>
/*
 *    turns on and off engine decoder functions F0-F28 (F0 is sometimes called FL)  
 *    NOTE: setting requests transmitted directly to mobile engine decoder --- current state of engine functions is not stored by this program
 *    
 *    CAB:  the short (1-127) or long (128-10293) address of the engine decoder
 *    
 *    To set functions F0-F4 on (=1) or off (=0):
 *      
 *    BYTE1:  128 + F1*1 + F2*2 + F3*4 + F4*8 + F0*16
 *    BYTE2:  omitted
 *   
 *    To set functions F5-F8 on (=1) or off (=0):
 *   
 *    BYTE1:  176 + F5*1 + F6*2 + F7*4 + F8*8
 *    BYTE2:  omitted
 *   
 *    To set functions F9-F12 on (=1) or off (=0):
 *   
 *    BYTE1:  160 + F9*1 +F10*2 + F11*4 + F12*8
 *    BYTE2:  omitted
 *   
 *    To set functions F13-F20 on (=1) or off (=0):
 *   
 *    BYTE1: 222 
 *    BYTE2: F13*1 + F14*2 + F15*4 + F16*8 + F17*16 + F18*32 + F19*64 + F20*128
 *   
 *    To set functions F21-F28 on (=1) of off (=0):
 *   
 *    BYTE1: 223
 *    BYTE2: F21*1 + F22*2 + F23*4 + F24*8 + F25*16 + F26*32 + F27*64 + F28*128
 *   
 *    returns: NONE
 * 
 */
//      mRegs->setFunction(com+1);
  if(sscanf(com+1,"%d %d %d",&cab,&fByte,&eByte)<2)
    return;

#ifdef DEBUG
	 Serial1.print("fByte = ");
	 Serial1.println(fByte, BIN);
#endif
  NewFunctionState = 0;
  fungrp = (fByte & 0xE0);
  if(fungrp==0xC0) {
	fungrp = fByte;
#ifdef DEBUG
	 Serial1.print("eByte = ");
	 Serial1.println(eByte, BIN);
#endif
  }
  
  switch(fungrp){
	case 0x80:                      // this is a request for functions FL,F1-F4
	  NewFunctionState = NewFunctionState + (fByte & 0x10)>>4;		// FL (F0)
	  NewFunctionState = NewFunctionState + (fByte & 0x0F)*2;	//F1-F4
	  startfn = 0;
	  stopfn = 4;
	  break;
	case 0xB0:
	  NewFunctionState = (fByte & 0x0F)<<5;		// this is a request for functions F5-F8
	  startfn = 5;
	  stopfn = 8;
	  break;
	case 0xA0:
	  NewFunctionState = (fByte & 0x0F)<<9;		// this is a request for functions F9-F12
	  startfn = 9;
	  stopfn = 12;
	  break;
	case 0xDE:
	  NewFunctionState = (eByte & 0xFF)<<13;		// this is a request for functions F13-F20
	  startfn = 13;
	  stopfn = 20;
	  break;
	case 0xDF:
	  NewFunctionState = (eByte & 0xFF)<<21;		// this is a request for functions F21-F28
	  startfn = 21;
	  stopfn = 28;
	  break;
	}
#ifdef DEBUG
	 Serial1.print("NewFunctionState = ");
	 Serial1.println(NewFunctionState, BIN);
#endif
    
  for (int i=startfn; i <= stopfn; i++) {
	Bit_State = (NewFunctionState>>i & 0x01);
	if(Bit_State != FunctionState[i]){
		FunctionState[i] = Bit_State;
#ifdef DEBUG
	 Serial1.print("F");
	 Serial1.print(i);
	 Serial1.print(": Bit_State = ");
	 Serial1.println(Bit_State);
#endif
		if(i > numfpins) break; // Unprogrammed Function - Quit
  switch ( MyCVs[ 30+(i*5)] )  {  // Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
    case 0:    // On - Off LED
      digitalWrite (fpins[i], Bit_State);
      ftn_queue[i].inuse = 0;
      break;
    case 1:    // Blinking LED
      if (Bit_State)  { // ON
        ftn_queue[i].inuse = 1;
        ftn_queue[i].increment = int(MyCVs[ 31+(i*5) ]);
        ftn_queue[i].start_value = int(MyCVs[ 32+(i*5) ]);
        ftn_queue[i].stop_value = int(MyCVs[ 33+(i*5) ]);
        digitalWrite(fpins[i], ftn_queue[i].start_value);
        ftn_queue[i].current_position = 0;
      } else { //OFF
        ftn_queue[i].inuse = 0;
        digitalWrite(fpins[i], 0);
        }
      break;
    case 2:    // Servo
      if (ftn_queue[i].inuse == 0)  {
	    ftn_queue[i].inuse = 1;
		servo[i].attach(fpins[i]);
	  }
      if (Bit_State==1) ftn_queue[i].increment = char (MyCVs[ 31+(i*5) ]);
        else ftn_queue[i].increment = - char(MyCVs[ 31+(i*5) ]);
      if (Bit_State==1) ftn_queue[i].stop_value = MyCVs[ 33+(i*5) ];
        else ftn_queue[i].stop_value = MyCVs[ 32+(i*5) ];
      break;
    case 3:    // Blinking LED PAIR
      if (Bit_State)  { // ON
        ftn_queue[i].inuse = 1;
        ftn_queue[i].increment = int(MyCVs[ 31+(i*5) ]);
        ftn_queue[i].start_value = int(MyCVs[ 32+(i*5) ]);
        ftn_queue[i].stop_value = int(MyCVs[ 33+(i*5) ]);
        digitalWrite(fpins[i], ftn_queue[i].start_value);
        digitalWrite(fpins[i+1], (1-ftn_queue[i].start_value));
        ftn_queue[i].current_position = 0;
      } else { // OFF
        ftn_queue[i].inuse = 0;
        digitalWrite(fpins[i], 0);
        digitalWrite(fpins[i+1], 0);
        }
      break;
    case 4:    // Pulse Output based on Rate*10 Milliseconds
      if ((ftn_queue[i].inuse==0) && (Bit_State==1)) {  //First Turn On Detected
        digitalWrite(fpins[i], 1);
		delay (10*ftn_queue[i].increment);
        digitalWrite(fpins[i], 0);
		ftn_queue[i].inuse = 1;                    //inuse set to 1 says we already pulsed
      } else 
          if (Bit_State==0)  ftn_queue[i].inuse = 0;
      break;	  
    case 5:    // Fade On
#define fadedelay 24
      if ((ftn_queue[i].inuse==0) && (Bit_State==1))  {
        ftn_queue[i].inuse = 1;
        for (t=0; t<ftn_queue[i].stop_value; t+=ftn_queue[i].increment) {
          digitalWrite( fpins[i], 1);
          delay(fadedelay*(t/(1.*ftn_queue[i].stop_value)));
          digitalWrite( fpins[i], 0);
          delay(fadedelay-(fadedelay*(t/(1.*ftn_queue[i].stop_value))));
        }
        digitalWrite( fpins[i],  1 );
      } else {
          if ((ftn_queue[i].inuse==1) && (Bit_State==0)) {
            ftn_queue[i].inuse = 0;
            digitalWrite(fpins[i], 0);
          }
        }
      break;
    case 6:    // Future Function
      ftn_queue[i].inuse = 0;
      break;
    default:
      ftn_queue[i].inuse = 0;
      break;
    }
	}
  }
      break;
      
/***** OPERATE STATIONARY ACCESSORY DECODERS  ****/    

    case 'a':       // <a ADDRESS SUBADDRESS ACTIVATE>
/*
 *    turns an accessory (stationary) decoder on or off
 *    
 *    ADDRESS:  the primary address of the decoder (0-511)
 *    SUBADDRESS: the subaddress of the decoder (0-3)
 *    ACTIVATE: 1=on (set), 0=off (clear)
 *    
 *    Note that many decoders and controllers combine the ADDRESS and SUBADDRESS into a single number, N,
 *    from  1 through a max of 2044, where
 *    
 *    N = (ADDRESS - 1) * 4 + SUBADDRESS + 1, for all ADDRESS>0
 *    
 *    OR
 *    
 *    ADDRESS = INT((N - 1) / 4) + 1
 *    SUBADDRESS = (N - 1) % 4
 *    
 *    returns: NONE
 */
//      mRegs->setAccessory(com+1);
      break;

/***** CREATE/EDIT/REMOVE/SHOW & OPERATE A TURN-OUT  ****/    

    case 'T':       // <T ID THROW>
/*
 *   <T ID THROW>:                sets turnout ID to either the "thrown" or "unthrown" position
 *   
 *   ID: the numeric ID (0-32767) of the turnout to control
 *   THROW: 0 (unthrown) or 1 (thrown)
 *   
 *   returns: <H ID THROW> or <X> if turnout ID does not exist
 *   
 *   *** SEE ACCESSORIES.CPP FOR COMPLETE INFO ON THE DIFFERENT VARIATIONS OF THE "T" COMMAND
 *   USED TO CREATE/EDIT/REMOVE/SHOW TURNOUT DEFINITIONS
 */
      Turnout::parse(com+1);
      break;

/***** CREATE/EDIT/REMOVE/SHOW & OPERATE AN OUTPUT PIN  ****/    

    case 'Z':       // <Z ID ACTIVATE>
/*
 *   <Z ID ACTIVATE>:          sets output ID to either the "active" or "inactive" state
 *   
 *   ID: the numeric ID (0-32767) of the output to control
 *   ACTIVATE: 0 (active) or 1 (inactive)
 *   
 *   returns: <Y ID ACTIVATE> or <X> if output ID does not exist
 *   
 *   *** SEE OUTPUTS.CPP FOR COMPLETE INFO ON THE DIFFERENT VARIATIONS OF THE "O" COMMAND
 *   USED TO CREATE/EDIT/REMOVE/SHOW TURNOUT DEFINITIONS
 */
      Output::parse(com+1);
      break;
      
/***** CREATE/EDIT/REMOVE/SHOW A SENSOR  ****/    

    case 'S': 
/*   
 *   *** SEE SENSOR.CPP FOR COMPLETE INFO ON THE DIFFERENT VARIATIONS OF THE "S" COMMAND
 *   USED TO CREATE/EDIT/REMOVE/SHOW SENSOR DEFINITIONS
 */
      Sensor::parse(com+1);
      break;

/***** SHOW STATUS OF ALL SENSORS ****/

    case 'Q':         // <Q>
/*
 *    returns: the status of each sensor ID in the form <Q ID> (active) or <q ID> (not active)
 */
      Sensor::status();
      break;

/***** WRITE CONFIGURATION VARIABLE BYTE TO ENGINE DECODER ON MAIN OPERATIONS TRACK  ****/    

    case 'w':      // <w CAB CV VALUE>
/*
 *    writes, without any verification, a Configuration Variable to the decoder of an engine on the main operations track
 *    
 *    CAB:  the short (1-127) or long (128-10293) address of the engine decoder 
 *    CV: the number of the Configuration Variable memory location in the decoder to write to (1-1024)
 *    VALUE: the value to be written to the Configuration Variable memory location (0-255)
 *    
 *    returns: NONE
*/    
//      mRegs->writeCVByteMain(com+1);
  if(sscanf(com+1,"%d %d %d",&cab,&cv,&cv_value)!=3)
    return;

  if(cv <= MAX_CV){
	  MyCVs[cv] = cv_value;
  }
      break;      

/***** WRITE CONFIGURATION VARIABLE BIT TO ENGINE DECODER ON MAIN OPERATIONS TRACK  ****/    

    case 'b':      // <b CAB CV BIT VALUE>
/*
 *    writes, without any verification, a single bit within a Configuration Variable to the decoder of an engine on the main operations track
 *    
 *    CAB:  the short (1-127) or long (128-10293) address of the engine decoder 
 *    CV: the number of the Configuration Variable memory location in the decoder to write to (1-1024)
 *    BIT: the bit number of the Configurarion Variable regsiter to write (0-7)
 *    VALUE: the value of the bit to be written (0-1)
 *    
 *    returns: NONE
*/        
//      mRegs->writeCVBitMain(com+1);
      break;      

/***** WRITE CONFIGURATION VARIABLE BYTE TO ENGINE DECODER ON PROGRAMMING TRACK  ****/    

    case 'W':      // <W CV VALUE CALLBACKNUM CALLBACKSUB>
/*
 *    writes, and then verifies, a Configuration Variable to the decoder of an engine on the programming track
 *    
 *    CV: the number of the Configuration Variable memory location in the decoder to write to (1-1024)
 *    VALUE: the value to be written to the Configuration Variable memory location (0-255) 
 *    CALLBACKNUM: an arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs that call this function
 *    CALLBACKSUB: a second arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs (e.g. DCC++ Interface) that call this function
 *    
 *    returns: <r CALLBACKNUM|CALLBACKSUB|CV Value)
 *    where VALUE is a number from 0-255 as read from the requested CV, or -1 if verificaiton read fails
*/    
//      pRegs->writeCVByte(com+1);
      break;      

/***** WRITE CONFIGURATION VARIABLE BIT TO ENGINE DECODER ON PROGRAMMING TRACK  ****/    

    case 'B':      // <B CV BIT VALUE CALLBACKNUM CALLBACKSUB>
/*
 *    writes, and then verifies, a single bit within a Configuration Variable to the decoder of an engine on the programming track
 *    
 *    CV: the number of the Configuration Variable memory location in the decoder to write to (1-1024)
 *    BIT: the bit number of the Configurarion Variable memory location to write (0-7)
 *    VALUE: the value of the bit to be written (0-1)
 *    CALLBACKNUM: an arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs that call this function
 *    CALLBACKSUB: a second arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs (e.g. DCC++ Interface) that call this function
 *    
 *    returns: <r CALLBACKNUM|CALLBACKSUB|CV BIT VALUE)
 *    where VALUE is a number from 0-1 as read from the requested CV bit, or -1 if verificaiton read fails
*/    
//      pRegs->writeCVBit(com+1);
      break;      

/***** READ CONFIGURATION VARIABLE BYTE FROM ENGINE DECODER ON PROGRAMMING TRACK  ****/    

    case 'R':     // <R CV CALLBACKNUM CALLBACKSUB>
/*    
 *    reads a Configuration Variable from the decoder of an engine on the programming track
 *    
 *    CV: the number of the Configuration Variable memory location in the decoder to read from (1-1024)
 *    CALLBACKNUM: an arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs that call this function
 *    CALLBACKSUB: a second arbitrary integer (0-32767) that is ignored by the Base Station and is simply echoed back in the output - useful for external programs (e.g. DCC++ Interface) that call this function
 *    
 *    returns: <r CALLBACKNUM|CALLBACKSUB|CV VALUE)
 *    where VALUE is a number from 0-255 as read from the requested CV, or -1 if read could not be verified
*/    
//      pRegs->readCV(com+1);
      break;

/***** TURN ON POWER FROM MOTOR SHIELD TO TRACKS  ****/    

    case '1':      // <1>
/*   
 *    enables power from the motor shield to the main operations and programming tracks
 *    
 *    returns: <p1>
 */    
     track_power = true;
     INTERFACE.print("<p1>");
     break;
          
/***** TURN OFF POWER FROM MOTOR SHIELD TO TRACKS  ****/    

    case '0':     // <0>
/*   
 *    disables power from the motor shield to the main operations and programming tracks
 *    
 *    returns: <p0>
 */
     track_power = false;
     analogWrite(ENA, 0);       // set speed to zero
     analogWrite(ENB, 0);       // set speed to zero
     INTERFACE.print("<p0>");
     break;

/***** READ MAIN OPERATIONS TRACK CURRENT  ****/    

    case 'c':     // <c>
/*
 *    reads current being drawn on main operations track
 *    
 *    returns: <a CURRENT> 
 *    where CURRENT = 0-1024, based on exponentially-smoothed weighting scheme
 */
      INTERFACE.print("<a");
      INTERFACE.print(int(mMonitor->current));
      INTERFACE.print(">");
      break;

/***** READ STATUS OF DCC++ BASE STATION  ****/    

    case 's':      // <s>
/*
 *    returns status messages containing track power status, throttle status, turn-out status, and a version number
 *    NOTE: this is very useful as a first command for an interface to send to this sketch in order to verify connectivity and update any GUI to reflect actual throttle and turn-out settings
 *    
 *    returns: series of status messages that can be read by an interface to determine status of DCC++ Base Station and important settings
 */
      if(track_power)      // check boolean variable
        INTERFACE.print("<p1>");
      else
        INTERFACE.print("<p0>");

      for(int i=0;i<MAX_THROTTLES;i++){
        if(speedTable[i+1]==0)
          continue;
        INTERFACE.print("<T");
        INTERFACE.print(i+1); INTERFACE.print(" ");
        if(speedTable[i+1]>0){
          INTERFACE.print(speedTable[i+1]);
          INTERFACE.print(" 1>");
        } else{
          INTERFACE.print(-speedTable[i+1]);
          INTERFACE.print(" 0>");
        }          
      }
      INTERFACE.print("<iDCC-EX V-");
      INTERFACE.print(VERSION);
      INTERFACE.print(" G-");
      INTERFACE.print(__DATE__);
      INTERFACE.print(">");

      Turnout::show();
      Output::show();
      Sensor::show();
                        
      break;

/***** RETURN NUMBER OF LOCOSLOTS  ****/        
      
    case '#':     // NUMBER OF LOCOSLOTS <#>
/*
 *     Return the number of LOCOSLOTS.
 *     
 *     returns: <# 12>
 *     where MAX_THROTTLES is the number of locos that can be controlled
 */
      INTERFACE.print("<# ");
      INTERFACE.print(12);
      INTERFACE.print(">");
      break;

/***** STORE SETTINGS IN EEPROM  ****/    

    case 'E':     // <E>
/*
 *    stores settings for turnouts and sensors EEPROM
 *    
 *    returns: <e nTurnouts nSensors>
*/
     
    EEStore::store();
    INTERFACE.print("<e ");
    INTERFACE.print(EEStore::eeStore->data.nTurnouts);
    INTERFACE.print(" ");
    INTERFACE.print(EEStore::eeStore->data.nSensors);
    INTERFACE.print(" ");
    INTERFACE.print(EEStore::eeStore->data.nOutputs);
    INTERFACE.print(">");
    break;
    
/***** CLEAR SETTINGS IN EEPROM  ****/    

    case 'e':     // <e>
/*
 *    clears settings for Turnouts in EEPROM
 *    
 *    returns: <O>
*/
     
    EEStore::clear();
    INTERFACE.print("<O>");
    break;

/***** PRINT CARRIAGE RETURN IN SERIAL MONITOR WINDOW  ****/    
                
    case ' ':     // < >                
/*
 *    simply prints a carriage return - useful when interacting with Ardiuno through serial monitor window
 *    
 *    returns: a carriage return
*/
      INTERFACE.println("");
      break;  

///          
/// THE FOLLOWING COMMANDS ARE NOT NEEDED FOR NORMAL OPERATIONS AND ARE ONLY USED FOR TESTING AND DEBUGGING PURPOSES
/// PLEASE SEE SPECIFIC WARNINGS IN EACH COMMAND BELOW
///

/***** ENTER DIAGNOSTIC MODE  ****/    

    case 'D':       // <D>  
/*
 *    SERIAL COMMUNICAITON WILL BE INTERUPTED ONCE THIS COMMAND IS ISSUED - MUST RESET BOARD OR RE-OPEN SERIAL WINDOW TO RE-ESTABLISH COMMS
 */

    Serial.println("\nEntering Diagnostic Mode...");
    delay(1000);
    
    break;

/***** WRITE A DCC PACKET TO ONE OF THE REGSITERS DRIVING THE MAIN OPERATIONS TRACK  ****/    
      
    case 'M':       // <M REGISTER BYTE1 BYTE2 [BYTE3] [BYTE4] [BYTE5]>
/*
 *   writes a DCC packet of two, three, four, or five hexidecimal bytes to a register driving the main operations track
 *   FOR DEBUGGING AND TESTING PURPOSES ONLY.  DO NOT USE UNLESS YOU KNOW HOW TO CONSTRUCT NMRA DCC PACKETS - YOU CAN INADVERTENTLY RE-PROGRAM YOUR ENGINE DECODER
 *   
 *    REGISTER: an internal register number, from 0 through MAX_MAIN_REGISTERS (inclusive), to write (if REGISTER=0) or write and store (if REGISTER>0) the packet 
 *    BYTE1:  first hexidecimal byte in the packet
 *    BYTE2:  second hexidecimal byte in the packet
 *    BYTE3:  optional third hexidecimal byte in the packet
 *    BYTE4:  optional fourth hexidecimal byte in the packet
 *    BYTE5:  optional fifth hexidecimal byte in the packet
 *   
 *    returns: NONE   
 */
//      mRegs->writeTextPacket(com+1);
      break;

/***** WRITE A DCC PACKET TO ONE OF THE REGSITERS DRIVING THE MAIN OPERATIONS TRACK  ****/    

    case 'P':       // <P REGISTER BYTE1 BYTE2 [BYTE3] [BYTE4] [BYTE5]>
/*
 *   writes a DCC packet of two, three, four, or five hexidecimal bytes to a register driving the programming track
 *   FOR DEBUGGING AND TESTING PURPOSES ONLY.  DO NOT USE UNLESS YOU KNOW HOW TO CONSTRUCT NMRA DCC PACKETS - YOU CAN INADVERTENTLY RE-PROGRAM YOUR ENGINE DECODER
 *   
 *    REGISTER: an internal register number, from 0 through MAX_MAIN_REGISTERS (inclusive), to write (if REGISTER=0) or write and store (if REGISTER>0) the packet 
 *    BYTE1:  first hexidecimal byte in the packet
 *    BYTE2:  second hexidecimal byte in the packet
 *    BYTE3:  optional third hexidecimal byte in the packet
 *    BYTE4:  optional fourth hexidecimal byte in the packet
 *    BYTE5:  optional fifth hexidecimal byte in the packet
 *   
 *    returns: NONE   
 */
//      pRegs->writeTextPacket(com+1);
      break;
            
/***** ATTEMPTS TO DETERMINE HOW MUCH FREE SRAM IS AVAILABLE IN ARDUINO  ****/        
      
    case 'F':     // <F>
/*
 *     measure amount of free SRAM memory left on the Arduino based on trick found on the internet.
 *     Useful when setting dynamic array sizes, considering the Uno only has 2048 bytes of dynamic SRAM.
 *     Unfortunately not very reliable --- would be great to find a better method
 *     
 *     returns: <f MEM>
 *     where MEM is the number of free bytes remaining in the Arduino's SRAM
 */
      int v; 
      INTERFACE.print("<f");
      INTERFACE.print(" TODO ");
      INTERFACE.print(">");
      break;

/***** LISTS BIT CONTENTS OF ALL INTERNAL DCC PACKET REGISTERS  ****/        

    case 'L':     // <L>
/*
 *    lists the packet contents of the main operations track registers and the programming track registers
 *    FOR DIAGNOSTIC AND TESTING USE ONLY
 */
      break;

  } // switch
} // SerialCommand::parse

///////////////////////////////////////////////////////////////////////////////

void SerialCommand::check_function() {
	
  for (int i=0; i < numfpins; i++) {
    if (ftn_queue[i].inuse==1)  {

    switch (MyCVs[ 30+(i*5) ]) {
      case 0:
        break;
      case 1:
	    ftn_queue[i].current_position = ftn_queue[i].current_position + ftn_queue[i].increment;
        if (ftn_queue[i].current_position > ftn_queue[i].stop_value) {
          ftn_queue[i].start_value = 1 - ftn_queue[i].start_value; // Toggle the state
          digitalWrite(fpins[i], ftn_queue[i].start_value);
          ftn_queue[i].current_position = 0;
        }
        break;
      case 2:
        {
	  if (servo_slow_counter++ > servo_slowdown)
	    {
        ftn_queue[i].current_position = ftn_queue[i].current_position + ftn_queue[i].increment;
	    if (ftn_queue[i].increment > 0) {
	      if (ftn_queue[i].current_position > ftn_queue[i].stop_value) {
		ftn_queue[i].current_position = ftn_queue[i].stop_value;
                ftn_queue[i].inuse = 0;
		servo[i].detach();
              }
            }
	    if (ftn_queue[i].increment < 0) { 
	      if (ftn_queue[i].current_position < ftn_queue[i].start_value) { 
	        ftn_queue[i].current_position = ftn_queue[i].start_value;
                ftn_queue[i].inuse = 0;
		servo[i].detach();
              }
	    }
            servo[i].write(ftn_queue[i].current_position);
            servo_slow_counter = 0;
	    }
	   }
        break;
      case 3:
	    ftn_queue[i].current_position = ftn_queue[i].current_position + ftn_queue[i].increment;
        if (ftn_queue[i].current_position > ftn_queue[i].stop_value) {
          digitalWrite(fpins[i+1], ftn_queue[i].start_value);
          ftn_queue[i].start_value = 1 - ftn_queue[i].start_value;
          digitalWrite(fpins[i], ftn_queue[i].start_value);
          ftn_queue[i].current_position = 0;
        }
        i++;
        break;
       case 4:   // Simple Pulsed Output based on saved Rate =10*Rate in Milliseconds
		 {
		   ftn_queue[i].inuse = 0;
		   ftn_queue[i].current_position = 0;
           ftn_queue[i].increment = 10 * int (char (MyCVs[ 31+(i*5) ]));
           digitalWrite(fpins[i], 0);
		 }
         break;
	   case 5:   // Fade On

         break;         
       case 6:   // NEXT FEATURE to pin
         break;         
       default:
         break;   
      } // Switch
    } // If Inuse
  } // For loop
  return;
} // SerialCommand::check_function
