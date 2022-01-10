# DCp_Pico
Raspberry Pi Pico base station for DCC++ for old DC trains

##Configuration Details

The multifunction decoder examples all for 6 functions to be assigned to any of the 12 available pins: on/off control, singl e line blinking with variable rate, servo control with start position/stop position/transit rate CV setting and end to end c ontrol via the function (on/off), and paired line blinking with variable rate.

When first loaded the decoder is set to short DCC Mobile address 24 and/or Accessory decoder address 40. The decoder can be  reset to the original parameters by loading CV 120 with 120 (decimal). This will reset everything including the decoder addr ess, when the pushbutton on the Pro Mini is pushed (reset) or by powering the decoder off then on. You will know when the de fault CV setting are being reset as the decoder will flash Digital Pin 14 (A0) for one second. The multifunction decoder add ress can be changed to another short DCC address by changing CV 1. The accessory decoders have 2 addresses: the Accessory ra nge start Address is in CV1 and the multifunction address (with which you can program CV's for the Accessory decoder functio ns) in CV 121 and 122

###The 4 Servo 8 LED decoder configuration
Raspberry Pi Pico Pins are set as follows: 2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19
Pico GP Pin    Function
2              F0 Servo
3              F1 Servo
4              F2 Servo
5              F3 Servo
6              F4 Single LED Blink
7              F5 Single LED Blink
8              F6 Single LED On/Off
9              F7 Single LED On/Off
10             F8 Single LED Blink
11             F9 Single LED Blink
12             F10 Double LED Blink F10 and F11 LEDs (Pins 12 & 13)
13             F11 Single LED Blink (Ignored because of F10)
14             Sensor
15             Sensor
16             Sensor
28             Sensor

Correspondingly, for the 4 Servo 8 LED decoder configuration, CV’s are initially set to the following:

{CV number, Value}    Description
  {1, 24}  Decoder Initial Address
  {30, 2}, //F0 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {31, 1},    //F0 Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {32, 28},   //F0  Start Position F0=0
  {33, 140},  //F0  End Position   F0=1
  {34, 28},   //F0  Current Position
  {35, 2},  //F1 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {36, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {37, 28},   //  Start Position Fx=0
  {38, 140},  //  End Position   Fx=1
  {39, 28},  //  Current Position
  {40, 2},  //F2 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {41, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {42, 28},   //  Start Position Fx=0
  {43, 140},  //  End Position   Fx=1
  {44, 28},    //  Current Position
  {45, 2}, //F3 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {46, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {47, 28},   //  Start Position Fx=0
  {48, 140},  //  End Position   Fx=1
  {49, 28},    //  Current Position
  {50, 1}, //F4 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {51, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {52, 1},   //  Start Position Fx=0
  {53,35},  //  End Position   Fx=1
  {54, 1},    //  Current Position
  {55, 1}, //F5 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {56, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {57, 1},   //  Start Position Fx=0
  {58, 100},  //  End Position   Fx=1
  {59, 1},    //  Current Position
  {60, 0}, //F6 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {61, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {62, 1},   //  Start Position Fx=0
  {63, 10},  //  End Position   Fx=1
  {64, 1},    //  Current Position
  {65, 0}, //F7 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {66, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {67, 1},   //  Start Position Fx=0
  {68, 5},  //  End Position   Fx=1
  {69, 1},    //  Current Position
  {70, 1}, //F8 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {71, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {72, 1},   //  Start Position Fx=0
  {73, 5},  //  End Position   Fx=1
  {74, 1},    //  Current Position
  {75, 1}, //F9 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {76, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {77, 1},   //  Start Position Fx=0
  {78, 20},  //  End Position   Fx=1
  {79, 1},    //  Current Position
  {80, 3}, //F10 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {81, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {82, 1},   //  Start Position Fx=0
  {83, 35},  //  End Position   Fx=1
  {84, 2},    //  Current Position
  {85, 0}, //F11 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade
  {86, 1},    // Rate  Blink=Eate,PWM=Rate,Servo=Rate
  {87, 1},   //  Start Position Fx=0
  {88, 4},  //  End Position   Fx=1
  {89, 1},    //  Current Position
  {120, 0}   Master Reset CV When set to 120 and Power cycled resets all CV’s

Each Function is controlled by a maximum of 5 CV’s.
For example F0 is initially set for servo control:
  {30, 2},     // F0  Pin Function Configuration  2=Servo
  {31, 1},     // F0  Rate  Blink=Rate, Servo=Rate
  {32, 28},   // F0  Start Position  F0=0  Initially 26
  {33, 140},  // F0  End Position   F0=1  Initially 140
  {34, 28},    // F0  Current Position or State

F7 is initially set for single LED blinking control:
  {65, 1},    // F7 Pin Function Configuration  1=Blink
  {66, 1},    // Rate  Blink  1= Slowest
  {67, 1},    //  Start Count Set to 1 or 0
  {68,35},   //  End Count 2-255 -- 255 = Slow Blink
  {69, 1},    //  Current State of LED

F13 is initially set for double LED blinking control of F13 and F14 LED Pins:
  {95, 3},    // F13 Pin Function Configuration  3=Double LED Blink
  {96, 1},    // Rate  Blink  1= Slowest
  {97, 1},    //  Start Count Set to 1 or 0
  {98, 35},  //  End Count 2-255 -- 255 = Slow Blink
  {99, 2},    //  Current State of LED

F9 is initially set for single LED On/Off control:
  {75, 0},    //  F9 Pin Function Configuration  0=On/Off
  {76, 1},    //  Ignored
  {77, 1},    //  Ignored
  {78, 10},  //  Ignored
  {79, 1},    //  Ignored
