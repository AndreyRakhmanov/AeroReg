#include <EEPROM.h>

const int      NUM_PARAMS = 6;
volatile float param[NUM_PARAMS + 1];
boolean        isLess = false;

// Programming channels assignment.
const int GAIN_PARAM =       1;
const int GAIN_MAX_PARAM =   2;
const int ZUMM_PARAM =       3;
const int PROP_PARAM =       4;
const int THR_PARAM =        5;
const int THR_MIN_PARAM =    6;

const int G1 =     7;
const int G2 =     8;
const int T1 =     9;
const int T2 =     10;

// MOD signal states.
const int mode2D = 0;
const int mode3D = 1;
const int modeNone = 2;

// Working modes.
boolean directOn = false;
boolean progOn   = false;
boolean calibOn  = false;
boolean enterNormal = false;
boolean startOn = true;

int curChannel = 0;
float curChannelVal = 0.0;
int oldCurChannel = 0;
unsigned long endProgTime = 0;
unsigned long channelChangeTime = 0;

// TROTTLE
int THR_OUT_PIN = 4;
int THR_IN_PIN = 3;

int t1 = 0;
int t2 = 0;
int t1A = 1500;
int t2A = 1500;
int tIn = t1;
int tOut = t1;
volatile unsigned long tStart = 0;
float tMin = 0.0;
volatile float ttIn  = 0.0;
float tOutA = 0.0;


// GAIN
int GAIN_OUT_PIN = 5;
int GAIN_IN_PIN = 2;

int g1 = 0;
int g2 = 0;
int g1A = 1500;
int g2A = 1500;
int gIn = g1;
volatile int gOut = g1;
volatile unsigned long sStart = 0;
float gAOut  = 1.0;

// SWITCH
int MOD_IN_PIN = 0;

volatile int sw = 0;
int oldSw = 0;
volatile int modIn = 0;
volatile unsigned long modStart = 0;
int sw1Level = 1300;
int sw2Level = 1600;

// Zummer
int BATT_IN = 1;  // Analog pin
int ZUMM_OUT_PIN = 14;

float zIn = 1.0;
boolean lowBatt = false;

// PROP
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

  startOn = true;

  // On start action.
  delay(1000);
}


void loop() {  
  if (startOn) {    
    if (thrLow()) {
      enterNormal = true;  
    } else {
      switch(sw) {
         case modeNone : directOn = true; break;
         case mode3D : calibOn = true; break;                     
         case mode2D : progOn = true; break;
      }
      
      buzz(3 - sw, 400, 400);
    }        
  } else if (directOn) {
    
     // Exit.
     if (thrLow() && switchChanged()) {
         directOn = false;  
         enterNormal = true;
     }
  } else if (calibOn) { 
      digitalWrite(ZUMM_OUT_PIN, (millis() % 3000 / 50));  

      g1A = min(g1A, gIn);
      g2A = max(g2A, gIn);
      t1A = min(t1A, tIn);
      t2A = max(t2A, tIn); 
            
       // Exit.
       if ((tIn - t1A < 150) && switchChanged()) {
           // Check if calibration is valid, and 
           // write data of calibration in EEPROM memory. 
           if ((g2A - g1A) > 500 && (t2A - t1A) > 500) {
              EEPROM.write(G1, (g1 = g1A) / 10);
              EEPROM.write(G2, (g2 = g2A) / 10);              
              EEPROM.write(T1, (t1 = t1A) / 10);
              EEPROM.write(T2, (t2 = t2A) / 10);                       
              buzz(1, 1000, 100);
           } else {
              // Calibration failed. 
              buzz (3, 300, 300);
           }
         
           calibOn = false;  
           enterNormal = true;
       } 
  } else if (progOn) {
      if (startOn)   
        oldCurChannel = curChannel;
        
      double newChannelVal = ttIn * (NUM_PARAMS + 1);
      double channelDelta = newChannelVal - curChannelVal;
  
      // Add some hysteresis.
      if (channelDelta > 0.2 || channelDelta < - 0.2) {
        curChannelVal = newChannelVal;
        curChannel = curChannelVal;
        if (curChannel > NUM_PARAMS)
          curChannel = NUM_PARAMS;
      }
  
      // Save new curChannel.
      if (switchChanged()) {
        if (sw == mode2D) {
          EEPROM.write(0, curChannel);
          buzz(5, 10, 30);        
          writeParam();
          dispValChange(false);
        }
      }
  
      // Change of channel.
      if (curChannel != oldCurChannel) {
        oldCurChannel = curChannel;
        channelChangeTime = millis();
        endProgTime = 0;
      }
  
      // On channel change.
      if (channelChangeTime && (millis() - channelChangeTime > 500) ) {
        if (curChannel == 0) {
          buzz(30, 1, 1);
          endProgTime = millis();
        } else {
          buzz(curChannel, 200, 100);
        }
  
        channelChangeTime = 0;
      }
  
      // Display parameter change.
      dispValChange(true);
  
      // On programming end;
      if (endProgTime && ( millis() - endProgTime > 2000)) {
        buzz(10, 10, 30);
        curChannel = EEPROM.read(0);
        readParams();
        progOn = false ;
      }
    } else {   // Normal mode.
      if (enterNormal) {
        buzz(5, 10, 30);          
        buzz(curChannel, 200, 100);        
        enterNormal = false;
      }

      // Making 'low battery bips' by buzzler.
      digitalWrite(ZUMM_OUT_PIN, !lowBatt || (millis() % 5000 / 4500));
        
      if (sw != oldSw) {
         buzz(1, 50, 100);  
      }
    }
    
    oldSw = sw;
    //  Serial.print(curChannel);
    //  Serial.print(" ");
    //  if (curChannel)
    //    Serial.print(EEPROM.read(curChannel));
    //  Serial.println("");
    // Serial.println(param[THR_MIN_PARAM] / 5.0);
    // Serial.println(ttIn);
    // Serial.println(gIn);
    // Serial.println(gOut);
    // Serial.println(rpsK);
    // Serial.println(gAOut);
    // Serial.println("");
    // printInt("MOD IN", modIn);
    // printInt("switch mode", sw);
    // printInt("THR IN", tIn);
          
    startOn = false;
    delay(100);    
}


// INTERRUPT VECTORS

// Optic Sensor interrupt
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

    ttIn = getValue(t1, t2, tIn);

    if (progOn || calibOn) {
      tOut = t1;
    } else if (directOn) {
      tOutA = tIn;
    } else if (sw == mode2D) {
      tOutA = ttIn + (ttIn - rpsK) * 5.0 * param[THR_PARAM];

      tMin = param[THR_MIN_PARAM] / 5.0;

      // In stabilizin set minimum gain as tMin.
      if ((ttIn > tMin) && (tOutA < tMin)) {
        tOutA = tMin;
      }      
    } else {
      tOutA = ttIn;
    }
    
    tOut  = setValue(t1, t2, tOutA);
  } 
}

// GAIN interrupt processing.
ISR(INT1_vect)
{
  if (digitalRead(GAIN_IN_PIN) == LOW) {
    gIn = micros() - modStart + 20;
  } else {
    modStart = micros();

    rps = rps + 5 * counter ;
    rps = rps * 5 / 6;
    counter = 0;
    sei();

    rpsK = (0.2 + 0.8 * param[PROP_PARAM]) * rps / 300;

    if (progOn || calibOn) {
       gAOut = 0;
    } else if (directOn) {
       gAOut = gIn;
    } else {
       gAOut = param[GAIN_MAX_PARAM] / (1.0 + rpsK * param[GAIN_PARAM] * 5.0);
    }

    gOut = setValue(g1, g2, gAOut);

    // Parameter adjustment.
    if (curChannel > 0) {
      param[curChannel] = getValue(g1, g2, gIn);
    }

    // Battery check.
    zIn = analogRead(BATT_IN) / 1024.0;
    
    lowBatt =  (zIn > 0.1) && (zIn *  4.0 + tOutA * 0.35 < 2.0 + param[ZUMM_PARAM] ) && (param[ZUMM_PARAM] > 0.1) ;
  } 
}

// MOD interrupt processing.
ISR(INT2_vect)
{
  if (digitalRead(MOD_IN_PIN) == HIGH) {
    sStart = micros();
  } else {
    modIn = micros() - sStart + 20;
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

// Stores current channel param in EEPROM.
void writeParam()
{
  if (curChannel > 0) {
    int paramInt = param[curChannel] * 256.0;

    if (paramInt < 0)
      paramInt = 0;
    else if (paramInt > 255)
      paramInt = 255;

      EEPROM.write(curChannel, paramInt);
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
// current GAIN defined parameter crosses 
// stored parameter value. 
void dispValChange(boolean doSound)
{
    if (curChannel == 0)
      return;

    int vOld = EEPROM.read(curChannel);
    int v = param[curChannel] * 256.0;

    digitalWrite(LED_OUT_PIN, v < vOld);
    
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
  return oldSw != sw;
}

// Detects if throttle is low.
boolean thrLow() {
  return tIn - t1 < 150;
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
   if (EEPROM.read(G1) != 0) 
      return;
  
   EEPROM.write(G1, 1096 / 10);
   EEPROM.write(G2, 1964 / 10);
   EEPROM.write(T1, 1030 / 10);
   EEPROM.write(T2, 1892 / 10);
  
   EEPROM.write(GAIN_PARAM, 0.32 * 255);
   EEPROM.write(GAIN_MAX_PARAM, 1.0 * 255);
   EEPROM.write(ZUMM_PARAM, 0.85 * 255);     
   EEPROM.write(PROP_PARAM, 0.73 * 255);
   EEPROM.write(THR_PARAM, 0.39 * 255);
   EEPROM.write(THR_MIN_PARAM, 0.34 * 255);         
   
   buzz(1, 3000, 0);
}

// Reads EEPROM into memory.
void readEEPROM()
{
  g1 = EEPROM.read(G1) * 10;
  g2 = EEPROM.read(G2) * 10;  
  t1 = EEPROM.read(T1) * 10;
  t2 = EEPROM.read(T2) * 10;    

  curChannelVal = curChannel = EEPROM.read(0);
  
  readParams();
}

