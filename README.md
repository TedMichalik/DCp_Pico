# DCp_Pico
Raspberry Pi Pico base station for DCC++ for old DC trains

## Configuration Details

The multifunction decoder examples all for 6 functions to be assigned to any of the 12 available pins: on/off control, singl e line blinking with variable rate, servo control with start position/stop position/transit rate CV setting and end to end c ontrol via the function (on/off), and paired line blinking with variable rate.

When first loaded the decoder is set to short DCC Mobile address 24 and/or Accessory decoder address 40. The decoder can be  reset to the original parameters by loading CV 120 with 120 (decimal). This will reset everything including the decoder addr ess, when the pushbutton on the Pro Mini is pushed (reset) or by powering the decoder off then on. You will know when the de fault CV setting are being reset as the decoder will flash Digital Pin 14 (A0) for one second. The multifunction decoder add ress can be changed to another short DCC address by changing CV 1. The accessory decoders have 2 addresses: the Accessory ra nge start Address is in CV1 and the multifunction address (with which you can program CV's for the Accessory decoder functio ns) in CV 121 and 122

### The 4 Servo 8 LED decoder configuration
Raspberry Pi Pico Pins are set as follows: 2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19

|Pico GP Pin    |Function|
|---|---|
|2              |F0 Servo|
|3              |F1 Servo|
|4              |F2 Servo|
|5              |F3 Servo|
|6              |F4 Single LED On/Off|
|7              |F5 Single LED On/Off|
|8              |F6 Single LED On/Off|
|9              |F7 Single LED On/Off|
|10             |F8 Single LED Blink|
|11             |F9 Single LED Blink|
|12             |F10 Double LED Blink F10 and F11 LEDs (Pins 12 & 13)|
|13             |F11 Single LED Blink (Ignored because of F10)|
|14             |Sensor|
|15             |Sensor|
|16             |Sensor|
|28             |Sensor|

Correspondingly, for the 4 Servo 8 LED decoder configuration, CV’s are initially set to the following:

|{CV number, Value}    |Description|
|---|---|
|  {1, 24}   |Decoder Initial Address (FUTURE)|
|  {30, 2},  |//F0 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade|
|  {31, 2},  |  //F0 Rate  Blink=Eate,PWM=Rate,Servo=Rate|
|  {32, 0},  |  //F0 Start Position F0=0|
|  {33, 12}, |  //F0 End Position   F0=1|
|  {34, 2},  |//F1 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade|
|  {35, 2},  |  //F1 Rate  Blink=Eate,PWM=Rate,Servo=Rate|
|  {36, 0},  |  //F1 Start Position Fx=0|
|  {37, 12}, |  //F1 End Position   Fx=1|
|  {38, 2},  |//F2 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade|
|  {39, 5},  |  //F2 Rate  Blink=Eate,PWM=Rate,Servo=Rate|
|  {40, 0},  |  //F2 Start Position Fx=0|
|  {41, 180},|  //F2 End Position   Fx=1|
|  {42, 2},  |//F3 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade|
|  {43, 5},  |  //F3 Rate  Blink=Eate,PWM=Rate,Servo=Rate|
|  {44, 0},  |  //F3 Start Position Fx=0|
|  {45, 180},|  //F3 End Position   Fx=1|
|  {46, 0},  |//F4 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade|
|  {47, 1},  |  //F4 Rate  Blink=Eate,PWM=Rate,Servo=Rate|
|  {48, 0},  |  //F4 Start Position Fx=0|
|  {49, 1},  |  //F4 End Position   Fx=1|
|  {50, 0},  |//F5 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade|
|  {51, 1},  |  //F5 Rate  Blink=Eate,PWM=Rate,Servo=Rate|
|  {52, 0},  |  //F5 Start Position Fx=0|
|  {53, 1},  |  //F5 End Position   Fx=1|
|  {54, 0},  |//F6 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade|
|  {55, 1},  |  //F6 Rate  Blink=Eate,PWM=Rate,Servo=Rate|
|  {56, 0},  |  //F6 Start Position Fx=0|
|  {57, 1},  |  //F6 End Position   Fx=1|
|  {58, 0},  |//F7 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade|
|  {59, 1},  |  //F7 Rate  Blink=Eate,PWM=Rate,Servo=Rate|
|  {60, 0},  |  //F7 Start Position Fx=0|
|  {61, 1},  |  //F7 End Position   Fx=1|
|  {62, 1},  |//F8 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade|
|  {63, 1},  |  //F8 Rate  Blink=Eate,PWM=Rate,Servo=Rate|
|  {64, 1},  |  //F8 Start Position Fx=0|
|  {65, 50}, |  //F8 End Position   Fx=1|
|  {66, 1},  |//F9 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade|
|  {67, 1},  |  //F9 Rate  Blink=Eate,PWM=Rate,Servo=Rate|
|  {68, 1},  |  //F9 Start Position Fx=0|
|  {69, 50}, |  //F9 End Position   Fx=1|
|  {70, 3},  |//F10 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade|
|  {71, 1},  |  //F10 Rate  Blink=Eate,PWM=Rate,Servo=Rate|
|  {72, 1},  |  //F10 Start Position Fx=0|
|  {73, 50}, |  //F10 End Position   Fx=1|
|  {74, 0},  |//F11 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade|
|  {75, 1},  |  //F11 Rate  Blink=Eate,PWM=Rate,Servo=Rate|
|  {76, 1},  |  //F11 Start Position Fx=0|
|  {77, 50}, |  //F11 End Position   Fx=1|

Each Function is controlled by a maximum of 4 CV’s.
For example F0 is initially set for servo control:

|{CV number, Value}    |Description|
|---|---|
|  {30, 2},  |//F0 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade|
|  {31, 2},  |  //F0 Rate  Blink=Eate,PWM=Rate,Servo=Rate (1=Slowest)|
|  {32, 0},  |  //F0 Start Position F0=0  (0 Degrees)|
|  {33, 12}, |  //F0 End Position   F0=1  (12 Degrees)|

F8 is initially set for single LED blinking control:

|{CV number, Value}    |Description|
|---|---|
|  {62, 1},  |//F8 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade|
|  {63, 1},  |  //F8 Rate  Blink=Eate,PWM=Rate,Servo=Rate (1=Slowest)|
|  {64, 1},  |  //F8 Start Position Fx=0 (Start Count Set to 1 or 0)|
|  {65, 50}, |  //F8 End Position   Fx=1 (End Count 2-255 -- 255 = Slow Blink)|

F10 is initially set for double LED blinking control of F10 and F11 LED Pins:

|{CV number, Value}    |Description|
|---|---|
|  {70, 3},  |//F10 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade|
|  {71, 1},  |  //F10 Rate  Blink=Eate,PWM=Rate,Servo=Rate (1=Slowest)|
|  {72, 1},  |  //F10 Start Position Fx=0 (Start Count Set to 1 or 0)|
|  {73, 50}, |  //F10 End Position   Fx=1 (End Count 2-255 -- 255 = Slow Blink)|

F4 is initially set for single LED On/Off control:

|{CV number, Value}    |Description|
|---|---|
|  {46, 0},  |//F4 Config 0=On/Off,1=Blink,2=Servo,3=DBL LED Blink,4=Pulsed,5=fade|
|  {47, 1},  |  //F4 Rate  Blink=Eate,PWM=Rate,Servo=Rate (Ignored)|
|  {48, 0},  |  //F4 Start Position Fx=0 (Ignored)|
|  {49, 1},  |  //F4 End Position   Fx=1 (Ignored)|
