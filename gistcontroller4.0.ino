/*
 * GIP Jobbe Geybels 2020-2021
 * Gistcontroller v4.0 op een NODEMCU-board (ESP8266)
 * 
 * I2C-ADXL345 Gyroscoop
 * I2C-LCD
 * DS18B20 tempsensor
 * Relays: cooling - heating
 * ESP8266: Stuur berichten via GMail
 * 
 * TODO: stuur warnings als +50min koelen/warmen
 * TODO: stuur bericht als teveel boven/onder temp
 * TODO: millis vervangen door echte datum (wifi=DateTime)
 */
 
#include <LiquidCrystal_I2C.h>             // Librarys voor LCD I2C
#include <Adafruit_ADXL345_U.h>            // Librarys ADXL345
#include <Adafruit_Sensor.h>               // Librarys ADXL345
#include <DallasTemperature.h>             // Librarys voor DS18B20
#include <OneWire.h>                       // Librarys voor DS18B20
#include <ESP8266WiFi.h>                   // Librarys voor wifi
#include <ESPDateTime.h>                   // Datum en tijd
#include "Gsender.h"                       // GMail settings
#include "Base64.h"                        // base-64 voor wifi-paswoord
#include "constants.h"                     // Constante parameters

bool    debug                   = true;
bool    debug_tilt              = false;   // true=serieel tilt-test
bool    debug_buttons           = false;   // true=serieel button-test
bool    debug_msg               = false;    // true=serieel msg-test
bool    debug_wifi              = true;   // true=serieel wifi-test
bool    send_msg                = true;   // true=send mails
String  versie                  = "4.0";   // versienummer
int     lcdBaud                 = 115200;  // LCD baudrate

int     currentControllerState    = 1;     // 0=Koelen, 1=Niet-actief, 2=Verwarmen
int     timeInControllerState     = 0;     // Tijd doorgebracht in een status
float   currentTemp               = 22.00; // Huidige temperatuur
float   targetTemp                = 23.0;  // Te handhaven temperatuur
float   coolingThreshold          = 1.0;   // Uitstelwaaarde voor koeling start
float   heatingThreshold          = 1.0;   // Uitstelwaarde voor verwarmen start
float   maxTemp                   = 0;     // hoogste temperatuur die bereikt werd
float   minTemp                   = 0;     // laagste temperatuur die bereikt werd
int     maxTimeHeating            = 0;     // langste tijd in status VERWARMEN
int     maxTimeCooling            = 0;     // langste tijd in status KOELEN

int     targettimebetweenmsg      = 1800;   // 6 = +/-10sec
int     timeBetweenMessages       = 0;     // Tijd verstreken sinds laatste message
boolean mailSend                  = false; // true = mail send

boolean wifiConnected             = false; // true = wifi connected
boolean datetimeSuccess           = false; // ophalen datetime gelukt/niet gelukt
boolean initSuccess               = false;
String  initMsg                   = "";

boolean buttonPressed             = false; // werd er op een knop gedrukt?
int     currentLCDState           = 0;     // Welk LCD momenteel actief: 0 tem ... 
int     timeInLCDState            = 0;     // > REDIRECT_TIMEOUT = LCD naar 0
int     backlightTimeout          = 0;     // > REDIRECT_TIMEOUTx5 = LCD uit 
boolean isBacklightActive         = true;  // LCD momenteel actief?

float   X_out                     = 0;     // tilt-x-as
float   Y_out                     = 0;     // tilt-y-as
float   max_tilt                  = 0.20;  // verschil op x/y-as
int     currentTilt               = 0;     // de huidige tiltwaarde
int     lastTilt                  = 0;     // de vorige tiltwaarde
long    lastmillis_tilt           = 0;     // millis van de laaste tilt
int     between                   = 0;     // Tijd tussen twee tilts

int     countTilts                = 0;     // Aantal tilts in targettimebetweenmsg
int     countTiltsTotal           = 0;     // Aantal tilts sinds start
int     countStatHeat             = 0;     // Aantal HEAT binnen targettimebetweenmsg
int     countStatHeatTotal        = 0;     // Aantal Heat sinds start
int     countStatCool             = 0;     // Aantal COOL binnen targettimebetweenmsg
int     countStatCoolTotal        = 0;     // Aantal COOL sinds start

// !!! na 50dagen gaat millis terug op 0 (voldoende voor de gisting)
long    StartMillis               = 0;     // Bewaar startpunt programma
long    PastMillis                = 0;     // Verstreken tijd sinds StartMillis
time_t  startDateInt              = 0;     // Startdatum/uur (DateTime)

// Initialiseer librarys 
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);
Adafruit_ADXL345_Unified tilter = Adafruit_ADXL345_Unified(12345);
OneWire oneWire(TEMP_PIN);
DallasTemperature sensors(&oneWire);

void setup(void) {
  // Initialseer LCD en Serieel
  lcd_serial_Init();

  // Initialiseer componenten
  initSuccess = initComponents();
  if (!initSuccess) {
    while(1) {
      // endless loop = initialisatie was niet goed
      lcdShowInit("!! Probleem Init",0);
    }
  }
  
  // Initialiseer currentTemp/max-minTemp
  maxTemp = minTemp = getCurrentTemperature();
}

/**
 * Temperatuur + statusupdate = elke seconde
 * LCD en knoppen             = elke 0.1 seconde
 */
void loop(void) {
  updateTemperature();                      // Haal de huidige temperatuur op
  controlState();                           // Update status: koel,inactief,verwarm
  updateTilted();                           // Controleer tiltsensor

  // onderstaande loop doe je 10 (i=0...9) keer telkens met een delay van DELAY
  for (int i = 0; i< 10; i ++) {
    controlDisplayState();                  // Controleer de knoppen + acties + LCD
    delay(DELAY);                           // !! de enige delay in het programma
  }

  sendMessage();                            // Zend bericht als tijd verstreken 
  displayState();                           // update de LCDdisplay
}

boolean initComponents() {
  // initialiseer pins
  pinMode( BUTTON_ADC_PIN, INPUT );        // multi buttons op A0
  digitalWrite( BUTTON_ADC_PIN, LOW );     // pullup uitzetten op A0
  pinMode( COOLING_PIN, OUTPUT);           // output naar led/220V
  pinMode( HEATING_PIN, OUTPUT );          // output naar led/220V
  digitalWrite( COOLING_PIN, LOW );        // initieel inactief zetten
  digitalWrite( HEATING_PIN, LOW );        // initieel inactief zetten
  sensors.begin();                         // Start the DS18B20 sensor

  // test DS18B20
  lcdShowInit("DS18B20.........",0);
  if (getCurrentTemperature() == -127) {
     lcdShowInit("Error",11);
     return false;
  } else {lcdShowInit("gelukt",10);}
  
  // initialiseer gyro...
  lcdShowInit("ADXL345.........",0); 
  if(!tilter.begin()) {
    lcdShowInit("Error",11);
    return false;
  } else {lcdShowInit("gelukt",10);}
  tilter.setRange(ADXL345_RANGE_4_G);        // initialiseer ADXL345
  
  // Verbind wifi als send_msg = true
  if (send_msg) {
    lcdShowInit("Wifi............",0);
    if (!wifiConnected) {wifiConnected = setupWifi(50);}
    if (!wifiConnected) {
      lcdShowInit("Error",11);
      return false;
    } else {lcdShowInit("gelukt",10);
      
    }
  }
  // Ophalen DateTime
  lcdShowInit("DateTime........",0);
  if (!datetimeSuccess) {datetimeSuccess = setupDateTime(10);}
  if (!datetimeSuccess) {
    lcdShowInit("Error",11);
    return false;
  } else {
    lcdShowInit("gelukt",10);
    // Bewaar het startpunt
    startDateInt = DateTime.now();
    StartMillis = millis();
    }  

  return true;
}

/**
 * Haal de huidige temperatuur op en update MAX- en MIN-Temp
 */
void updateTemperature() {
  currentTemp = getCurrentTemperature();
  if (currentTemp > maxTemp) {
    maxTemp = currentTemp;
  }
  else if (currentTemp < minTemp) {
    minTemp = currentTemp;
  }
}

/**
 * Returns de huidige temperatuur
 */
float getCurrentTemperature() {
  sensors.requestTemperatures();            // Opvragen temperatuur (Dallas)
  return sensors.getTempCByIndex(0);        // 0=eersteIC - kan er meerdere hebben
}

/**
 * Wijzig status adhv temperatuur + bewaar de timers 
 * (HEATING) <=> (INACTIVE) <=> (COOLING)
 */
void controlState() {
  timeInControllerState++; 
  switch ( currentControllerState ) {
    // Momenteel aan het KOELEN
    case STATE_COOLING:
      maxTimeCooling++;
      if (currentTemp < targetTemp + coolingThreshold) {
        //stop KOELEN + zet INACTIEF
        currentControllerState = STATE_INACTIVE;
        if (timeInControllerState > maxTimeCooling) {
          maxTimeCooling = timeInControllerState;
        }
        timeInControllerState = 0;
        digitalWrite(COOLING_PIN, LOW);
      }
      break;
    // Momenteel INACTIEF = niks aan het doen
    case STATE_INACTIVE:
      if (currentTemp < targetTemp - heatingThreshold) {
        //start VERWARMEN
        currentControllerState = STATE_HEATING;
        timeInControllerState = 0;
        countStatHeat++;
        countStatHeatTotal++;
        digitalWrite(HEATING_PIN, HIGH);
      }
      else if (currentTemp > targetTemp + coolingThreshold) {
        //start KOELEN
        currentControllerState = STATE_COOLING;
        timeInControllerState = 0;
        countStatCool++;
        countStatCoolTotal++;
        digitalWrite(COOLING_PIN, HIGH);
      }
      break;
    // Momenteel aan het VERWARMEN
    case STATE_HEATING:
      maxTimeHeating++;
      if (currentTemp > targetTemp - heatingThreshold) {
        //stop VERWARMEN
        currentControllerState = STATE_INACTIVE;
        if (timeInControllerState > maxTimeHeating) {
          maxTimeHeating = timeInControllerState;
        }
        timeInControllerState = 0;
        digitalWrite(HEATING_PIN, LOW);
      } // if
      break;
  } // switch
}

/*
 * Tiltsensor
 */
void updateTilted() {
  getTilt();
  if (currentTilt != lastTilt) {
    // enkel de neutrale stand (=beneden) telt
    if (currentTilt == 0) {
      between = millis() - lastmillis_tilt;
      lastmillis_tilt = millis();
      countTilts++;         // tel 1 bij het aantal tiltwijzigingen binnen 1 bericht
      countTiltsTotal++;    // tel 1 bij totaal aantal tilts
    }

    lastTilt = currentTilt;
    between = millis() - lastmillis_tilt;
    lastmillis_tilt = millis();
    if (debug_tilt) {
      serialTilted();
      }
    }
}

void getTilt() {
  sensors_event_t event; 
  tilter.getEvent(&event);
  X_out = (event.acceleration.x)/9.8;  
  Y_out = (event.acceleration.y)/9.8;
  // Negatieve meting omzetten naar positieve waarde
  if (X_out < 0) { X_out = X_out * -1; }
  if (Y_out < 0) { Y_out = Y_out * -1; }

  currentTilt = 0;
  // Vergelijk de x-y-meting met max_tilt
  // om te vermijden dat bij naar beneden vallen er door 
  // de terugslag een nieuwe tilt geregistreerd wordt
  if (X_out > max_tilt || Y_out > max_tilt) {
    currentTilt = 1;
  } 
}

/*
 * Stuur het bericht met tilt- en statusinfo als tijd om is 
 */
void sendMessage() {
  timeBetweenMessages++;                     // tijd verstreken sinds laatste msg
  if (timeBetweenMessages > targettimebetweenmsg) {
    timeBetweenMessages = 1;
    if (debug_msg) {serialMsg();}

    if (send_msg) {
      if (!wifiConnected) {
        wifiConnected = setupWifi(10);
      }
      if (wifiConnected) {
        String subject      = "Gist Controller " + versie;
        String message      = "";
        message = fillMessage();
        mailSend = sendMail(subject,message);
        }
      }
    countTilts = 0;
    countStatHeat = 0;
    countStatCool = 0;
  }
}

/*
 * Send Mail
 */
boolean sendMail(String subject, String message) {
  boolean state       = true;
  String error_msg    = "";
  
  Gsender *gsender = Gsender::Instance();    // Getting pointer to class instance
  if(gsender->Subject(subject)->Send("gist.controller@gmail.com", message)) {
    state = true;
  }
  else 
  {
    error_msg = gsender->getError(); 
    state = false;
  }
  if (debug_wifi) {serialMsgSend(state,error_msg);}
  return state;
}

/*
 * Connecteer met wifi – returns true als gelukt
 */
boolean setupWifi(int loops) {
  int cntLoops          = 0;
  boolean wifiState     = true; 
  int inputStringLength = sizeof(passwordB64);
  int decodedLength     = Base64.decodedLength(passwordB64, inputStringLength);
  char decodedString[decodedLength];
  Base64.decode(decodedString, passwordB64, inputStringLength);

  WiFi.begin(ssid, decodedString);
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (debug_wifi) {Serial.print(".");}
    if (cntLoops > loops) {
      wifiState = false;
      break;
    }
    cntLoops++;
  }
  if (debug_wifi) {serialWifi(wifiState);}
  return wifiState;
}

/*
 * Ophalen actuele tijd
 */
boolean setupDateTime(int loops) {
  int cntLoops          = 0;
  boolean datetimeState = true; 
  DateTime.setTimeZone(1);
  DateTime.begin();
  while (!DateTime.isTimeValid()) {
    delay(100);
    if (debug_wifi) {Serial.print(".");}
    if (cntLoops > loops) {
      datetimeState = false;
      break;
    }
    cntLoops++;
    DateTime.begin();
  }
  return datetimeState;
}

/**
 * Gebruik knoppen, LCD-displaymodus aanpassen en LCD verversen
 */
void controlDisplayState() {
  
  // Zet LCD uit na verstrijken timeout
  checkBacklightTimeout();
  
  // bewaar tijd in deze LCD-status
  timeInLCDState += DISPLAY_TIMER_INCR;
  // welke knop is er gedrukt?
  byte whichButtonPressed = ReadButtons();

  // Geen knop gedrukt EN momenteel niet in SUMMARY-status 
  if ( !whichButtonPressed && currentLCDState != DISPLAY_SUMMARY) {
    // Indien je te lang in huidige LCD-status zit = Terug naar SUMMARY-status
    if ( timeInLCDState >= REDIRECT_TIMEOUT ) {
      timeInLCDState = 0;
      currentLCDState = DISPLAY_SUMMARY;
      displayState();
    }
    return;
  }  
  // Wissel van LCDstatus met knoppen LINKS/RECHTS
  else if ( whichButtonPressed == BUTTON_RIGHT || 
            whichButtonPressed == BUTTON_LEFT ) {
    handleLeftRight( whichButtonPressed );
    enableBacklight();
    displayState();
  }
  // Knoppen UP/DOWN = wijzig parameters (welke = afhankelijk van de LCD-status)
  else if ( whichButtonPressed == BUTTON_UP || 
            whichButtonPressed == BUTTON_DOWN ) {
    handleUpDown( whichButtonPressed );
    enableBacklight();
    displayState(); 
  }
  else if ( whichButtonPressed == BUTTON_SELECT ) {
    handleSelect();
  }
}

/**
 * Knop = LINKS-RECHTS = aanpassen van de LCD-status
 */
void handleLeftRight( int whichButtonPressed ) {
    // reset de tijd in deze LCD-status
    timeInLCDState = 0;

    if ( whichButtonPressed == BUTTON_RIGHT ) {
      currentLCDState++;
    } else {
      currentLCDState--;
    }
    // test of je niet te ver gaat tov NO_OF_LCD_STATES (momenteel 0 tem 8)
    currentLCDState += NO_OF_LCD_STATES;
    currentLCDState %= NO_OF_LCD_STATES;
}

/**
 * Knop = UP/DOWN = Wijzig parameters, afhankelijk van de huidige LCD-status
 */
void handleUpDown( int whichButtonPressed ) {

   switch ( currentLCDState ) {
    case DISPLAY_SET_MAX_TEMP:
      // Verhoog/verlaag de threashold 
      // hoe lager hoe sneller er gekoeld zal worden... 
      // hoe hoger hoe langer er gewacht wordt met koelen
      if ( whichButtonPressed == BUTTON_UP ) {
        // reset LCD-status-timer
        timeInLCDState = 0;
        // de nieuwe waarde mag niet hoger zijn dan MAXIMUM_THREASHOLD
        if ( coolingThreshold < MAXIMUM_THREASHOLD ) {
          coolingThreshold += TEMP_THRESHOLD_INCR;
        } 
      }
      else if ( whichButtonPressed == BUTTON_DOWN ) {
        // reset LCD-status-timer
        timeInLCDState = 0;
        // de nieuwe waarde mag niet lager zijn dan MINIMUM_THREASHOLD
        if ( coolingThreshold > MINIMUM_THREASHOLD ) {
          coolingThreshold -= TEMP_THRESHOLD_INCR; 
        }
      }
      break;
    case DISPLAY_SET_MIN_TEMP:
      // Verhoog/verlaag de threashold
      // hoe lager hoe sneller er verwarmd zal worden... 
      // hoe hoger hoe langer er gewacht wordt met verwarmen
      if ( whichButtonPressed == BUTTON_UP ) {
        // reset LCD-status-timer
        timeInLCDState = 0;

        // de nieuwe waarde mag niet hoger zijn dan MAXIMUM_THREASHOLD
        if ( heatingThreshold < MAXIMUM_THREASHOLD ) {
          heatingThreshold += TEMP_THRESHOLD_INCR; 
        }
      }
      else if ( whichButtonPressed == BUTTON_DOWN ) {
        // reset LCD-status-timer
        timeInLCDState = 0;
        // de nieuwe waarde mag niet lager zijn dan MINIMUM_THREASHOLD
        if ( heatingThreshold > MINIMUM_THREASHOLD ) {
          heatingThreshold -= TEMP_THRESHOLD_INCR; 
        }
      }
      break;
    case DISPLAY_SET_MSG_TIME:
      // Verhoog/verlaag de tijd tussen boodschappen
      if ( whichButtonPressed == BUTTON_UP ) {
        // reset LCD-status-timer
        timeInLCDState = 0;
        targettimebetweenmsg += BETWEEN_MSG_INCR; 
      }
      else if ( whichButtonPressed == BUTTON_DOWN ) {
        // reset LCD-status-timer
        timeInLCDState = 0;
        targettimebetweenmsg -= BETWEEN_MSG_INCR; 
      }
      break;  
    case DISPLAY_SET_TARGET:
      // Verhoog/verlaag de gewenste temperatuur
      if ( whichButtonPressed == BUTTON_UP ) {
        // reset LCD-status-timer
        timeInLCDState = 0;
        // mag niet hoger gezet worden dan MAXIMUM_TARGET
        if ( targetTemp < MAXIMUM_TARGET ) {
          targetTemp += TEMP_INCR; 
        }
      }
      else if ( whichButtonPressed == BUTTON_DOWN ) {
        // reset LCD-status-timer
        timeInLCDState = 0;
        // mag niet lager gezet worden dan MINIMUM_TARGET
        if ( targetTemp > MINIMUM_TARGET ) {
        targetTemp -= TEMP_INCR; 
        }
      }
      break;
  }
}

/*
 * Knop = SELECT = LCDscherm aan/uit
 */
void handleSelect() {
  if (isBacklightActive) {
    disableBacklight();
  }
  else {
    enableBacklight();
  }
}

/*
 * Welke knop werd er gedrukt?
 */
byte ReadButtons()
{
   unsigned int buttonVoltage;
   // Als er geen knop gedrukt wordt return = BUTTON_NONE
   byte button = BUTTON_NONE;   
   
   // Lees ADC van de button-pin = bepaling welke knop gedrukt
   buttonVoltage = analogRead( BUTTON_ADC_PIN );

   if (debug_buttons) {
      Serial.print(" KnopVoltage = ");
      Serial.println(buttonVoltage);
   }

   // Werd er op een nieuwe knop gedrukt?
   if (   buttonPressed 
       && buttonVoltage > ( NO_BUTTON_ADC - BUTTONHYSTERESIS )) {
     // Knop werd losgelaten
     buttonPressed = false;
     return button;
   } else if ( buttonPressed ) {
     // Knop is nu ingedrukt maar transactie is al verwerkt
     return button;
   }
   
   //Valt het voltage in de geldige waardes
   if (buttonVoltage >= ( RIGHT_10BIT_ADC - BUTTONHYSTERESIS )
    && buttonVoltage <= ( RIGHT_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_RIGHT;
      buttonPressed = true;
   }
   else if(   buttonVoltage >= ( UP_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( UP_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_UP;
      buttonPressed = true;
   }
   else if(   buttonVoltage >= ( DOWN_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( DOWN_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_DOWN;
      buttonPressed = true;
   }
   else if(   buttonVoltage >= ( LEFT_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( LEFT_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_LEFT;
      buttonPressed = true;
   }
   else if(   buttonVoltage >= ( SELECT_10BIT_ADC - BUTTONHYSTERESIS )
           && buttonVoltage <= ( SELECT_10BIT_ADC + BUTTONHYSTERESIS ) )
   {
      button = BUTTON_SELECT;
      buttonPressed = true;
   }   
   return( button );
}

/*
 * Initialiseer LCD 
 */
void lcd_serial_Init() {
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Gist Controller");
  lcd.setCursor(0,1);
  lcd.print("Versie: ");
  lcd.setCursor(8,1);
  lcd.print(versie);
  
  if ( debug || debug_tilt || debug_wifi || debug_buttons || debug_msg) {
      Serial.begin(lcdBaud);
      Serial.println("Initialisatie Gist-controller");
  }
  delay(1000);
}

/*
 * LCD: setup oké 
 */
void lcdShowInit(String initmsg, int pos) {
  lcd.setCursor(pos,2);
  lcd.print(initmsg);
  delay(1000);
}

/**
 * Zet LCD-backlight uit als timeout verstreken (REDIRECT_TIMEOUT * 5)
 */
void checkBacklightTimeout() {
  if (   isBacklightActive 
      && (backlightTimeout += DISPLAY_TIMER_INCR) > (REDIRECT_TIMEOUT * 5) ) {
    disableBacklight();
  }
}

/**
 * disable LCDscherm
 */
void disableBacklight() {
  lcd.noBacklight();
  lcd.noDisplay();
  isBacklightActive = false;
  backlightTimeout = 0;
}

/**
 * enable LCDscherm
 */
void enableBacklight() {
  lcd.display();
  lcd.backlight();
  
  isBacklightActive = true;
  backlightTimeout = 0;
}

/**
 * In welke LCD-status zitten we nu en toon die LCD-status
 */
void displayState()  {
  switch ( currentLCDState ) {
    case DISPLAY_SUMMARY:
      displaySummary();
      break;
    case DISPLAY_TEMP_HISTORY_MIN:
      displayMinHistory();
      break;
    case DISPLAY_TEMP_HISTORY_MAX:
      displayMaxHistory();
      break;
    case DISPLAY_SET_MAX_TEMP:
      displayMaxTempThreshold();
      break;
    case DISPLAY_SET_MIN_TEMP:
      displayMinTempThreshold();
      break;
    case DISPLAY_SET_MSG_TIME:
      displaysetmsgtime();
      break;
    case DISPLAY_STATUS:
      displayStatus();
      break;
    case DISPLAY_STATUS_MAX_COOL:
      displayStatusmaxcool();
      break;
    case DISPLAY_STATUS_MAX_HEAT:
      displayStatusmaxheat();
      break;
    case DISPLAY_SET_TARGET:
      displayTargetTemp();
      break;
    default:
      // Opvangen fout in schermdefinities
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Current State:");
      lcd.setCursor(15,0);
      lcd.print( currentLCDState );
      lcd.setCursor(0,1);
      lcd.print("Invalid State...");
  }
}

/**
 * summary display
 */ 
void displaySummary() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Temp");
  lcd.setCursor(9,0);
  lcd.print(currentTemp);
  lcd.setCursor(14,0);
  lcd.print((char)223);
  lcd.setCursor(15,0);
  lcd.print("C");
  lcd.setCursor(0,1);
  lcd.print("Doel");
  lcd.setCursor(14,1);
  lcd.print((char)223);
  lcd.setCursor(15,1);
  lcd.print("C");
  lcd.setCursor(9,1);
  lcd.print(targetTemp);
  lcd.setCursor(7,1);
  switch ( currentControllerState ) {
    case STATE_COOLING:
      lcd.print("*");
      break;
    case STATE_INACTIVE:
      lcd.print("-");
      break;
    case STATE_HEATING:
      lcd.print("^");
      break;
  }
}

/**
 * display temp history MIN
 */ 
void displayMinHistory() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Temp Historiek:");
  lcd.setCursor(0,1);
  lcd.print("Min: ");
  lcd.setCursor(5,1);
  lcd.print(minTemp);
}
/**
 * display temp history MAX
 */ 
void displayMaxHistory() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Temp Historiek:");
  lcd.setCursor(0,1);
  lcd.print("Max: ");
  lcd.setCursor(5,1);
  lcd.print(maxTemp);
}

/**
 * display max temp threshold
 */ 
void displayMaxTempThreshold() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Koel Threshold");
  lcd.setCursor(0,1);
  lcd.print(coolingThreshold);
  lcd.setCursor(7,1);
  lcd.print("[up/down]");
}

/**
 * display min temp threshold
 */ 
void displayMinTempThreshold() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Warm Threshold");
  lcd.setCursor(0,1);
  lcd.print(heatingThreshold);
  lcd.setCursor(7,1);
  lcd.print("[up/down]");
}

/**
 * display msg-time 
 */ 
void displaysetmsgtime() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Boodschap tijd: ");
  lcd.setCursor(0,1);
  lcd.print(targettimebetweenmsg);
  lcd.setCursor(7,1);
  lcd.print("[up/down]");
}

/**
 * display status
 */ 
void displayStatus() {
  lcd.clear();
  lcd.setCursor(0,0);
  switch (currentControllerState ) {
    case STATE_COOLING:
      lcd.print("Koelen    ");
      break;
    case STATE_INACTIVE:
      lcd.print("Inactief  ");
      break;
    case STATE_HEATING:
      lcd.print("Verwarmen ");
      break;
  }
  lcd.setCursor(0,1);
  lcd.print(showTime(timeInControllerState,false));
}

/**
 * display status
 */ 
void displayStatusmaxcool() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("MaxKoelen:");
  lcd.setCursor(0,1);
  lcd.print(showTime(maxTimeCooling,false));
}
void displayStatusmaxheat() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("MaxWarmen:");
  lcd.setCursor(0,1);
  lcd.print(showTime(maxTimeHeating,false));
}
/**
 * display target temp
 */ 
void displayTargetTemp() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Doel Temp: ");
  lcd.setCursor(0,1);
  lcd.print(targetTemp);
  lcd.setCursor(7,1);
  lcd.print("[up/down]");
}

/**
 * Zet seconden om in een string "dagen hh:mm:ss"
 * val = seconds
 */
String showTime(int val,bool showDays){
  int days                = 0;
  int hours               = 0;
  int minutes             = 0;
  int seconds             = 0;
  String result           = "";
  
  days    = elapsedDays(val);
  hours   = numberOfHours(val);
  minutes = numberOfMinutes(val);
  seconds = numberOfSeconds(val);
  if (showDays) {
    result  = String(zeroPad(days)) + " ";
  }
  result  = result + String(zeroPad(hours));
  result  = result + ":" + String(zeroPad(minutes));
  result  = result + ":" + String(zeroPad(seconds));
  return result;
}

// Verfraai getallen = voeg 0 toe als cijfer 1 positie is
String zeroPad( int value ) {
  String valueString = String(value);
  if (value < 10) {
    valueString = String("0") + valueString;
  }
  return valueString;
}

/*
 * vul string met informatie om te verzenden als message
 */
String fillMessage() {
  String bmsg       = "";

  bmsg = bmsg + DateFormatter::format("Startdatum: %d/%m/%Y %H:%M:%S", startDateInt);
  bmsg = bmsg + DateFormatter::format("Meetpunt  : %d/%m/%Y %H:%M:%S", DateTime.now());
  bmsg = bmsg + "<p>";
  
  bmsg = bmsg + "Verstreken tijd  : " + showTime((millis() - StartMillis)/1000,true) + "</BR>";
  bmsg = bmsg + "Aantal tilts     : " + countTilts + "<p>";
  bmsg = bmsg + "Totaal tilts     : " + countTiltsTotal + "<p>";
  bmsg = bmsg + "Laatste tilt-tijd: " + showTime(between/1000,true) + "<p>";
  bmsg = bmsg + "<p>";
            
  switch ( currentControllerState ) {
    case STATE_COOLING:
      bmsg = bmsg + "Koelen   : ";
      break;
    case STATE_INACTIVE:
      bmsg = bmsg + "Inactief : ";
      break;
    case STATE_HEATING:
      bmsg = bmsg + "Verwarmen: ";
      break;
  }
  bmsg = bmsg + showTime(timeInControllerState,false) + "<p>";
  bmsg = bmsg + "MaxTemp: " + maxTemp + "<p>";
  bmsg = bmsg + "MinTemp: " + minTemp + "<p>";
  bmsg = bmsg + "MaxTimeHeating: ";
  bmsg = bmsg + showTime(maxTimeHeating,false) + "<p>";
  bmsg = bmsg + "MaxTimeCooling: ";
  bmsg = bmsg + showTime(maxTimeCooling,false) + "<p>";
  bmsg = bmsg + "<p>";

  bmsg = bmsg + "Aantal Coolings: " + countStatCool + "<p>";
  bmsg = bmsg + "         Totaal: " + countStatCoolTotal + "<p>";
  bmsg = bmsg + "Aantal Heatings: " + countStatHeat + "<p>";
  bmsg = bmsg + "         Totaal: " + countStatHeatTotal + "<p>";  

  return bmsg;
}

void serialMsg() {
  Serial.println(DateFormatter::format("Startdatum: %d/%m/%Y %H:%M:%S", startDateInt));
  Serial.println(DateFormatter::format("Meetpunt  : %d/%m/%Y %H:%M:%S", DateTime.now()));
  //Serial.println(showTime(DateTime.now()-startDateInt,true));
  
  Serial.print("Verstreken tijd:");
  Serial.println(showTime((millis() - StartMillis)/1000,true));
  
  Serial.printf("Tilts: %i Totaal: %i\n", (countTilts), (countTiltsTotal));
  
  Serial.print("Laatste tilt-tijd: ");
  Serial.println(showTime(between/1000,true));
  
  switch ( currentControllerState ) {
    case STATE_COOLING:
      Serial.print("Koelen: ");
      break;
    case STATE_INACTIVE:
      Serial.print("Inactief: ");
      break;
    case STATE_HEATING:
      Serial.print("Verwarmen: ");
      break;
      }
  Serial.println(showTime(timeInControllerState,false));

  Serial.print("MaxTemp: ");
  Serial.print(maxTemp);
  Serial.print(" MinTemp: ");
  Serial.println(minTemp);
  Serial.print("MaxTimeHeating: ");
  Serial.println(showTime(maxTimeHeating,false));
  Serial.print("MaxTimeCooling: ");
  Serial.println(showTime(maxTimeCooling,false));
      
  Serial.print("Aantal Coolings: ");
  Serial.print(countStatCool);
  Serial.print("  Totaal:");
  Serial.println(countStatCoolTotal);
  Serial.print("Aantal Heatings: ");
  Serial.print(countStatHeat);
  Serial.print("  Totaal:");
  Serial.println(countStatHeatTotal);
  Serial.println("");
}

void serialWifi(boolean wstate) {
  if (wstate) {
    Serial.println("");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
  else {
    Serial.println("");
    Serial.println("Connection failed.");
  }
}

void serialMsgSend(boolean mstate, String msg_error) { 
  if (mstate) {
    Serial.println("Message send");
  } else {
    Serial.print("Error sending message: ");
    Serial.println(msg_error);
  }
}

void serialTilted() {
  Serial.print("Tilt = ");
  Serial.print(String(lastTilt));
  Serial.print(" op: ");
  Serial.print(showTime(lastmillis_tilt/1000,true));
  Serial.print(" Verschil: ");
  Serial.print(showTime(between/1000,true));
  Serial.print(" Count/Tot: ");
  Serial.print(countTilts);
  Serial.print(" / ");
  Serial.println(countTiltsTotal);
}
