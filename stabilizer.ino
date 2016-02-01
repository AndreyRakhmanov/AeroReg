#include <EEPROM.h>

const int      NUM_PARAMS = 6;
volatile float param[NUM_PARAMS + 1];
boolean        isLess = false;

// Programming params assignment.
const int GAIN_SUP_PARAM =   1;
const int GAIN_MAX_PARAM =   2;
const int LOW_BATT_PARAM =   3;
const int PROP_PARAM =       4;
const int THR_SUP_PARAM =    5;
const int THR_MIN_PARAM =    6;

// Params with GAIN and THR calibration data.
const int GMIN =     7;
const int GMAX =     8;
const int TMIN =     9;
const int TMAX =     10;

// MOD signal states.
const int mode2D = 0;
const int mode3D = 1;
const int modeNone = 2;

// Working modes.
boolean directOn = false;
boolean paramSetOn   = false;
boolean calibOn  = false;
boolean enterNormal = false;
boolean startOn = true;
unsigned long tIdleStart = 0;

// Current param related variables.
int curParam = 0;
float curParamVal = 0.0;
int oldCurParam = 0;
unsigned long curParamChangeTime = 0;

// TROTTLE variables
int THR_OUT_PIN = 4;
int THR_IN_PIN = 3;
int tMax = 0;
int tMin = 0;
int tIn = tMax;
int tOut = tMax;
volatile unsigned long tStart = 0;
volatile float ttIn  = 0.0;
float tOutA = 0.0;

// GAIN variables
int GAIN_OUT_PIN = 5;
int GAIN_IN_PIN = 2;
int gMin = 0;
int gMax = 0;
int gIn = gMin;
int gOld = 0;
volatile int gOut = gMin;
volatile unsigned long gStart = 0;
float gAOut  = 1.0;

// SWITCH variables
int MOD_IN_PIN = 0;
volatile int sw = 0;
int oldSw = 0;
volatile int modIn = 0;
volatile unsigned long mStart = 0;
int sw1Level = 1300;
int sw2Level = 1600;

// Zummer variables
int ZUMM_OUT_PIN = 14;
float zIn = 1.0;
boolean lowBatt = false;

// Battery voltage
int BATT_IN = 1;  // Analog pin

// PROP sensor
int PROP_IN_PIN = 7;  // Never used in programm.
volatile int counter = 0;
volatile int rps = 0;
volatile float rpsK = 0.0;

// LED
int LED_OUT_PIN = 15;

// ARDUINO main functions

void setup() {
  oldSw = sw;

  // Initialization iterrupts and timers. 
  systemInit();
  
  formatEEPROM();
  readEEPROM();

  // On start action.
  delay(1000);
  tIdleStart = millis();
}

void loop() {  
  if (startOn) {    
    setLED(false);
    
    if (thrLow()) {
      enterNormal = true;  
    } else {
      switch(sw) {
         case modeNone : directOn = true; break;
         case mode3D : calibOn = true; break;                     
         case mode2D : paramSetOn = true; break;
      }
      
      buzz(3 - sw, 400, 400);
    }        
  } 
  
  if (directOn) {    
     // Exit.
     if (isEndOfProgramming()) {
         directOn = false;  
         enterNormal = true;
     }
  } else if (calibOn) { 
    if (startOn) {
      gMin = 1500;
      gMax = 1500;
      tMax = 1500;
      tMin = 1500;
      gOld = 1500;
    }
      setLED(millis() % 200 / 100);    

      gMin = min(gMin, gIn);
      gMax = max(gMax, gIn);
      tMax = min(tMax, tIn);
      tMin = max(tMin, tIn); 
            
      if (isEndOfProgramming()) {
           // Check if calibration is valid, and 
           // write data of calibration in EEPROM memory. 
           if ((gMax - gMin) > 500 && (tMin - tMax) > 500) {
              EEPROM.write(GMIN, gMin / 10);
              EEPROM.write(GMAX, gMax / 10);              
              EEPROM.write(TMIN, tMax / 10);
              EEPROM.write(TMAX, tMin / 10);                       
              buzz(1, 1000, 500);
           } else {
              // Read old values.
              readEEPROM();
           }
         
           calibOn = false;  
           enterNormal = true;
       } 
  } else if (paramSetOn) {
      if (startOn) {  
        EEPROM.write(0, 0);
        curParamVal = curParam = 0;
        dispValChange(false);             
      }
        
      double newParamVal = ttIn * (NUM_PARAMS + 1);
      double paramDelta = newParamVal - curParamVal;
  
      // Add some hysteresis.
      if (paramDelta > 0.2 || paramDelta < - 0.2) {
        curParamVal = newParamVal;
        curParam = curParamVal;
        if (curParam > NUM_PARAMS)
          curParam = NUM_PARAMS;
          
        // To suppress GAIN change sound.
        dispValChange(false);                
      }
  
      // Change of param.
      if (curParam != oldCurParam) {
        oldCurParam = curParam;
        curParamChangeTime = millis();      
      }
  
      // On current param change.
      if (curParamChangeTime && (millis() - curParamChangeTime > 500) ) {
        soundCurParamChange(); 
        curParamChangeTime = 0;
      }
  
      // Display parameter change.
      dispValChange(true);
  
      // Save new curParam.
      if (switchChanged() && sw == mode2D) {
        buzz(2, 10, 30);           
        writeParam();   
        dispValChange(false);     
        
        boolean released = false;
        boolean setAgain = false;
        
        // Detect second switch move in 1 sec.
        for (int idx = 0; idx< 40; idx++) {
          if (switchChanged()) {
            if (sw != mode2D) {
              released = true;
            } else {
              setAgain = true;         
            }
          }
      
          delay(25);             
        }
                
        if (setAgain) {
            EEPROM.write(0, curParam);
            buzz(1, 500, 100);                         
        } 
      }
  
      // On programming end;
      if (isEndOfProgramming()) {
        paramSetOn = false ;
        enterNormal = true;    
      }
    } else {   // Normal mode.
      if (enterNormal) {
        enterNormal = false;
        oldSw = sw;
        curParam = EEPROM.read(0);
        readParams();
        
        setLED(true);        
        delay(200);        
        buzz(6, 10, 30);          
        delay(200);
        soundCurParamChange();             
        setLED(false);        
      }

      // Making 'low battery bips' by buzzler.
      digitalWrite(ZUMM_OUT_PIN, !lowBatt || (millis() % 5000 / 4500));
        
      soundSwitchChange();      
      // debug();
    }
    
    // Loop end.
    oldSw = sw;
    startOn = false;
    noPPMBlinking();
    delay(25);    
}

// VECTORS

// Optic Sensor interrupt processing.
ISR(INT6_vect)
{
  counter++;
}

// Timer interrupt processing.
ISR(TIMER1_OVF_vect)
{
  cli();
  bool isGain = digitalRead(THR_OUT_PIN) == HIGH;  
  digitalRead(THR_IN_PIN) == HIGH;
 
  TCCR1A = 0;
  digitalWrite(GAIN_OUT_PIN, LOW);
  digitalWrite(THR_OUT_PIN, LOW);
  sei();

  if (isGain) {
    digitalWrite(GAIN_OUT_PIN, HIGH);    
    setTimer(gOut);
  }
}

// THR interrupt processing.
ISR(INT0_vect)
{
  if (digitalRead(THR_IN_PIN) == LOW) {
    tIn = micros() - tStart + 20;    
  } else {    
    digitalWrite(THR_OUT_PIN, HIGH);
    tStart = micros();
    setTimer(tOut);
    sei();

    ttIn = getValue(tMax, tMin, tIn);

    if (paramSetOn || calibOn) {
      tOut = tMax;
    } else if (directOn) {
      tOut = tIn;
    } else if (sw == mode2D) {
      tOutA = ttIn + (ttIn - rpsK) * 5.0 * param[THR_SUP_PARAM];

      float ttMin = param[THR_MIN_PARAM] / 5.0;

      // In stabilizin set minimum gain as tMin.
      if ((ttIn > ttMin) && (tOutA < ttMin)) {
        tOutA = ttMin;
      }      
  
      tOut  = setValue(tMax, tMin, tOutA);      
    } else {
      tOut = tIn;
    }
  } 
}

// GAIN interrupt processing.
ISR(INT1_vect)
{
  if (digitalRead(GAIN_IN_PIN) == LOW) {
    gIn = micros() - mStart + 20;
  } else {
    mStart = micros();

    rps = rps + 5 * counter ;
    rps = rps * 5 / 6;
    counter = 0;
    sei();

    rpsK = param[PROP_PARAM] * rps / 300.0;

    if (paramSetOn || calibOn) {
       gAOut = 0;
    } else if (directOn) {
       gAOut = gIn;
    } else {
       gAOut = param[GAIN_MAX_PARAM] / (1.0 + rpsK * param[GAIN_SUP_PARAM] * 5.0);
    }

    gOut = setValue(gMin, gMax, gAOut);

    // Parameter adjustment.
    if (curParam > 0) {
      param[curParam] = getValue(gMin, gMax, gIn);
    }

    // Battery check.
    zIn = analogRead(BATT_IN) / 1024.0;
    
    lowBatt =  (zIn > 0.1) && (zIn *  4.0 + tOutA * 0.35 < 2.0 + param[LOW_BATT_PARAM] ) && (param[LOW_BATT_PARAM] > 0.1) ;
  } 
}

// MOD interrupt processing.
ISR(INT2_vect)
{
  if (digitalRead(MOD_IN_PIN) == HIGH) {
    gStart = micros();
  } else {
    modIn = micros() - gStart + 20;
    sei();
    
    sw = (modIn < sw1Level) ? 0 :
         (modIn < sw2Level) ? 1 : 2 ;    
  }
}

// UTILITY FUNCTIONS

// Read EEPROM stored parameters into params double array.
void readParams()
{
  for (int i = 1; i <= NUM_PARAMS; i++) {
    param[i] = EEPROM.read(i) / 256.0;
  }
}

// Stores current param param in EEPROM.
void writeParam()
{
  if (curParam > 0) {
    int paramInt = param[curParam] * 256.0;

    if (paramInt < 0)
      paramInt = 0;
    else if (paramInt > 255)
      paramInt = 255;

      EEPROM.write(curParam, paramInt);
  }
}

// Set timer to fire interrupt in some period of time. 
void setTimer(int lapse)
{
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = - lapse * 16;

  TCCR1B |= (1 << CS10);  // enable Timer1 overflow interrupt
}

// Converts PPM pulse length in mks
// to double value (0.0 - 1.0);
float getValue(int v1, int v2, int v)
{
  if (v < v1)
    return 0.0;
  else if (v > v2)
    return 1.0;

  return (v - v1) * 1.0 / (v2 - v1);
}

// Converts double value (0.0 - 1.0) to 
// the PPM pulse length in mks.
int setValue(int v1, int v2, float value)
{
  int v = v1 + value * (v2 - v1);

  if (v < v1)
    return v1;

  if (v > v2)
    return v2;

  return v;
}

// Does pulses of zummer.
void buzz(int num, int millsOn, int millsOff)
{
  delay(millsOff);

  for (int i = 0; i < num; i++) {
    digitalWrite(ZUMM_OUT_PIN, LOW);
    delay(millsOn);
    digitalWrite(ZUMM_OUT_PIN, HIGH);
    delay(millsOff);
  }
}

// Switches on LED and do buzz when 
// current 'variable' parameter crosses 
// stored parameter value. 
void dispValChange(boolean doSound)
{
    if (curParam == 0)
      return;

    int vOld = EEPROM.read(curParam);
    int v = param[curParam] * 256.0;

    setLED(v > vOld);
    
    if (isLess) {
        if (v < vOld - 1) {
            if (doSound)
                buzz(10, 1, 1);
    
            isLess = false;
        }
     } else {
        if (v > vOld + 1) {
            if (doSound)
                buzz(10, 1, 1);
                
            isLess = true;
        }
     }
}

// Detects if the switch is changed
boolean switchChanged() {
  boolean state = oldSw != sw;
  
  oldSw = sw;
  
  return state;  
}

// Detects if throttle is low.
boolean thrLow() {
  return tIn - tMax < 150;
}

// Prints int value into serial.
void printInt(const char* name, int value) {
  Serial.print(name);
  Serial.print(": ");
  Serial.println(value);
}

// Initilizes pins, timer and interrupts.
void systemInit()
{
  pinMode(GAIN_OUT_PIN, OUTPUT);
  pinMode(THR_OUT_PIN, OUTPUT);
  pinMode(LED_OUT_PIN, OUTPUT);
  digitalWrite(LED_OUT_PIN, LOW);
  pinMode(ZUMM_OUT_PIN, OUTPUT);
  digitalWrite(ZUMM_OUT_PIN, HIGH);

  // Timer setup.  
  TCCR1A = 0;
  TIMSK1 = (1 << TOIE1);  // Set CS10 bit so timer runs at clock speed:
  TCCR1B |= (1 << CS10);  // enable Timer1 overflow interrupt
  
  // Enabling interrupts
  cli();
  EICRA |= (1 << ISC00) | (1 << ISC10) | (1 << ISC20);
  EICRB |= (1 << ISC60) | (1 << ISC61);
  EIMSK |= (1 << INT6) | (1 << INT0) | (1 << INT1) | (1 << INT2);
  sei();
}

// First time formatting.
void formatEEPROM()
{
   if (EEPROM.read(GMIN) != 0) 
      return;
  
   EEPROM.write(GMIN, 1096 / 10);
   EEPROM.write(GMAX, 1964 / 10);
   EEPROM.write(TMIN, 1030 / 10);
   EEPROM.write(TMAX, 1892 / 10);
  
   EEPROM.write(GAIN_SUP_PARAM, 0.32 * 255);
   EEPROM.write(GAIN_MAX_PARAM, 1.0 * 255);
   EEPROM.write(LOW_BATT_PARAM, 0.85 * 255);     
   EEPROM.write(PROP_PARAM, 0.73 * 255);
   EEPROM.write(THR_SUP_PARAM, 0.39 * 255);
   EEPROM.write(THR_MIN_PARAM, 0.34 * 255);         
   
   buzz(1, 3000, 0);
}

// Reads EEPROM into memory.
void readEEPROM()
{
  gMin = EEPROM.read(GMIN) * 10;
  gMax = EEPROM.read(GMAX) * 10;  
  tMax = EEPROM.read(TMIN) * 10;
  tMin = EEPROM.read(TMAX) * 10;    

  curParamVal = curParam = EEPROM.read(0);
  
  readParams();
}

// Gives sound signals on change of the 3-pos switch
// position.
void  soundSwitchChange()
{
  if (sw != oldSw) {
         buzz(3 - sw, 5, 20);  
  }
}

// This function blinks LED is one of PPM is missed.
void noPPMBlinking()
{
    long curTime = micros();

    if (curTime - gStart > 1000000 ||
        curTime - tStart > 1000000 || 
        curTime - mStart > 1000000) {
        setLED(millis() % 1000 / 500);    
    }
}

// Here functions for printing parameters into serial port are placed.
void debug()
{
    printInt("MOD IN", modIn);
    printInt("switch mode", sw);
    printInt("THR IN", tIn);
    printInt("GAIN IN", gIn);
}

void soundCurParamChange()
{
    if (curParam != 0) {
      buzz(curParam, 200, 100);
    }
}

void setLED(boolean isOn)
{
  digitalWrite(LED_OUT_PIN, isOn ? 0 : 1);    
}

// Test if throttle is of and GAIN knob doesn't moved 
// during 5 seconds. 
boolean isEndOfProgramming() 
{
     // Find time when stick aren't moving and 
     // throttle is down. 
     if ((tIn - tMax > 50) || (gIn - gOld) > 50 ) {
          gOld = gIn;
          tIdleStart = millis();            
     }
     
     return millis() - tIdleStart > 5000;
}  
