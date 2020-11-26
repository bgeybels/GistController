#ifndef CONSTANTS_H
#define CONSTANTS_H

// paswoord in base64: omzetten via https://www.base64encode.org
const char* ssid                  = "Fortress_Guest";            
char passwordB64[]                = "d2VsY29tZWd1ZXN0MjczNTUw";  

// temp & delays
const int   DELAY                 = 100;   // 0.1 seconde (loop x10 = 1sec delay)
const float TEMP_INCR             = 0.5;   // Tel bij targetTemp UP/DOWN
const float MINIMUM_TARGET        = 20.0;  // Minimum toegelaten targetTemp
const float MAXIMUM_TARGET        = 23.0;  // Maximum toegelaten targetTemp
const float TEMP_THRESHOLD_INCR   = 0.25;  // Tel bij threashold UP/DOWN
const float MINIMUM_THREASHOLD    = 0.1;   // Minimum toegelaten MIN-THREASH
const float MAXIMUM_THREASHOLD    = 5.0;   // Maximum toegelaten MAX-THREASH
const int   DISPLAY_TIMER_INCR    = 10;    // tel bij Displaytimers
const int   REDIRECT_TIMEOUT      = 1000;  // Back to hoofdscherm *5=Timeout
const int   BETWEEN_MSG_INCR      = 5;     // tel bij targettimebetweenmsg u/d

// Pins in gebruik
const int   BUTTON_ADC_PIN        = A0;    // ButtonPIN (analoog 5 buttons)
const int   COOLING_PIN           = 12;    // GPIO5 Koeler (led-blauw of 220V)
const int   HEATING_PIN           = 14;    // GPIO6 Verwarming (led-rood of 220V)
const int   TEMP_PIN              = 2;     // GPIO4 Temperatuursensor DSB1820

// ADC waardes voor de 5 knoppen op A0 (regeling met weerstanden)
// Zet debug_buttons op true en bekijk de seriële monitor
const int   LEFT_10BIT_ADC        = 815;   // Links
const int   RIGHT_10BIT_ADC       = 545;   // Rechts
const int   UP_10BIT_ADC          = 870;   // Boven
const int   DOWN_10BIT_ADC        = 10;     // Onder
const int   SELECT_10BIT_ADC      = 725;   // Selecteer
const int   NO_BUTTON_ADC         = 1023;  // Geen knop
// deze constante wordt bij de gelezen weerstand geteld = schommeling opvangen
const int   BUTTONHYSTERESIS      =  10;         

// BUTTON-waardes
const int   BUTTON_NONE           = 0; 
const int   BUTTON_RIGHT          = 1;
const int   BUTTON_UP             = 2;
const int   BUTTON_DOWN           = 3;
const int   BUTTON_LEFT           = 4;
const int   BUTTON_SELECT         = 5;

// STATUS-waardes
const int   STATE_COOLING         = 0;
const int   STATE_INACTIVE        = 1;
const int   STATE_HEATING         = 2;

// LCDdisplay waardes
const int   DISPLAY_SUMMARY           = 0;
const int   DISPLAY_STATUS            = 1;
const int   DISPLAY_STATUS_MAX_HEAT   = 2;
const int   DISPLAY_STATUS_MAX_COOL   = 3;
const int   DISPLAY_TEMP_HISTORY_MIN  = 4;
const int   DISPLAY_TEMP_HISTORY_MAX  = 5;
const int   DISPLAY_SET_MAX_TEMP      = 6;
const int   DISPLAY_SET_MIN_TEMP      = 7;
const int   DISPLAY_SET_TARGET        = 8;
const int   DISPLAY_SET_MSG_TIME      = 9;
const int   NO_OF_LCD_STATES          = 10;

// Omzetten millis/seconden naar leesbare vorm
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)  
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN)
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)
#define elapsedDays(_time_) ( _time_ / SECS_PER_DAY) 

#endif // CONFIG
