#include <EEPROM.h>

volatile unsigned long gLapse = 0;
volatile unsigned long tLapse = 0;
volatile unsigned long gStart = 0;
volatile unsigned long tStart = 0;
volatile unsigned long sStart = 0;

const int numParams = 6;
volatile float param[numParams + 1];
int PG =     1;
int PGMAX =  2;
int PZ =     3;
int PV =     4;
int PT =     5;
int PTMIN =  6;

boolean progOn = true;
boolean buzzOn = true;
boolean startOn = true;

int curChannel = 0;
float curChannelVal = 0.0;
int oldCurChannel = 0;
unsigned long endProgTime = 0;
unsigned long channelChangeTime = 0;

// TROTTLE
int THR_OUT = 4;
int THR_IN = 3;
// int THR_INT = 0;
int t1 = 1030;
int t2 = 1892;
int tIn = t1;
int tOut = t1;
float tMin = 0.0;
volatile float ttIn  = 0.0;
float tOutA = 0.0;

// GAIN
int GOUT = 5;
int GAIN_IN = 2;
// int GAIN_INT = 1;
int g1 = 1096;
int g2 = 1964;
int gIn = g1;
volatile int gOut = g1;
float gAOut  = 1.0;

// SWITCH
int MOD_IN = 0;
volatile boolean sw = 0;
int oldSw = 0;
int swLevel = 1200;
volatile boolean mode2D = 0;

// Zummer
int BATT_IN = 1;  // analog
int ZUMM_OUT = 14;
float zIn = 1.0;
boolean lowBatt = false;

// PROP
int PROP_IN = 7;
// int PROP_INT = 4;
volatile int counter = 0;
volatile int rps = 0;
volatile float rpsK = 0.0;

// LED
int LED_OUT = 15;

void setup() {
  pinMode(GOUT, OUTPUT);
  pinMode(THR_OUT, OUTPUT);

  pinMode(LED_OUT, OUTPUT);

  digitalWrite(LED_OUT, LOW);

  pinMode(ZUMM_OUT, OUTPUT);
  digitalWrite(ZUMM_OUT, HIGH);

  readParams();

  curChannelVal = curChannel = EEPROM.read(0);

  oldSw = sw;

  TCCR1A = 0;
  TIMSK1 = (1 << TOIE1);  // Set CS10 bit so timer runs at clock speed:
  TCCR1B |= (1 << CS10);  // enable Timer1 overflow interrupt
  // Enabling interrupts
  cli();
  EICRA |= (1 << ISC00) | (1 << ISC10) | (1 << ISC20);
  EICRB |= (1 << ISC60) | (1 << ISC61);
  EIMSK |= (1 << INT6) | (1 << INT0) | (1 << INT1) | (1 << INT2);
  sei();

  startOn = false;

  // On start action.
  delay(1000);
  if (ttIn * (numParams + 1) < 1.0)  {
    progOn = false;
    if (curChannel)
      buzz(curChannel, 200, 100);
    else
      buzz(5, 10, 30);
  }

  disp(false);
}

void loop() {
  delay(100);

  // Programming mode.
  if (progOn) {
    if (startOn)
      oldCurChannel = curChannel;

    double newChannelVal = ttIn * (numParams + 1);
    double channelDelta = newChannelVal - curChannelVal;

    // Add some hysteresis.
    if (channelDelta > 0.2 || channelDelta < - 0.2) {
      curChannelVal = newChannelVal;
      curChannel = curChannelVal;
      if (curChannel > numParams)
        curChannel = numParams;
      disp(false);
    }

    // Save new curChannel.
    if (sw != oldSw) {
      if (sw) {
        EEPROM.write(0, curChannel);
        writeParam();
        buzz(5, 10, 30);
      }

      oldSw = sw;
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

    // On programming end;
    if (endProgTime && ( millis() - endProgTime > 2000)) {
      buzz(10, 10, 30);
      curChannel = EEPROM.read(0);
      readParams();
      progOn = false ;
    }

    // Display parameter change.
    disp(true);
  }


  //  Serial.print(curChannel);
  //  Serial.print(" ");
  //  if (curChannel)
  //    Serial.print(EEPROM.read(curChannel));
  //  Serial.println("");
  // Serial.println(param[PTMIN] / 5.0);
  Serial.println(ttIn);
  Serial.println(gIn);
  Serial.println(gOut);
  Serial.println(rpsK);
  Serial.println(gAOut);
  Serial.println("");

  startOn = false;
}

void readParams()
{
  for (int i = 1; i <= numParams; i++) {
    param[i] = EEPROM.read(i) / 256.0;
  }
}

void writeParam()
{
  if (curChannel > 0) {
    int paramInt = param[curChannel] * 256;

    if (paramInt < 0)
      paramInt = 0;
    else if (paramInt > 255)
      paramInt = 255;

    int oldParamInt = EEPROM.read(curChannel);

    if (paramInt != oldParamInt) {
      EEPROM.write(curChannel, paramInt);
    }
  }
}

ISR(INT6_vect)
{
  cli();
  counter++;
  sei();
}

ISR(TIMER1_OVF_vect)
{
  cli();
  bool isGain = digitalRead(THR_OUT) == HIGH;                                                                                                                                                                                                                                                                                       digitalRead(THR_IN) == HIGH;
 
  TCCR1A = 0;
  digitalWrite(GOUT, LOW);
  digitalWrite(THR_OUT, LOW);
  sei();

  if (isGain) {
    digitalWrite(GOUT, HIGH);    
    setTimer(gOut);
  }
}

// Throttle
ISR(INT0_vect)
{
  if (digitalRead(THR_IN) == HIGH) {        
    digitalWrite(THR_OUT, HIGH);
    tStart = micros();
    setTimer(tOut);
    sei();

    ttIn = getValue(t1, t2, tIn);

    if (progOn) {
      tOut = t1;
    } else if (mode2D) {

      tOutA = ttIn + (ttIn - rpsK) * 5.0 * param[PT];

      tMin = param[PTMIN] / 5.0;

      // In stabilizin set minimum gain as tMin.
      if ((ttIn > tMin) && (tOutA < tMin))
        tOutA = tMin;

      tOut = setValue(t1, t2, tOutA);
    } else {
      tOutA = ttIn;
      tOut  = setValue(t1, t2, tOutA);
    }
  } else {
    tIn = micros() - tStart + 20;
    sei();
  }
}

// Giroscope gain.-
ISR(INT1_vect)
{
  if (digitalRead(GAIN_IN) == HIGH) {
    gStart = micros();

    rps = rps + 5 * counter ;
    rps = rps * 5 / 6;
    counter = 0;
    sei();

    rpsK = (0.2 + 0.8 * param[PV]) * rps / 300;

    gAOut = param[PGMAX] / (1.0 + rpsK * param[PG] * 5.0);

    gOut = progOn ? g1 : setValue(g1, g2, gAOut);

    // Parameter adjustment.
    if (curChannel > 0) {
      param[curChannel] = getValue(g1, g2, gIn);
    }

    // Battery check.
    zIn = analogRead(BATT_IN) / 1024.0;

    if (!progOn && !buzzOn) {
      if (millis() % 5000 / 4500) {
        if (lowBatt) {
          digitalWrite(ZUMM_OUT, LOW);
        }
      } else {
        digitalWrite(ZUMM_OUT, HIGH);
        lowBatt =  (zIn > 0.1) && (zIn *  4.0 + tOutA * 0.35 < 2.0 + param[PZ] ) && (param[PZ] > 0.1) ;
      }
    }
  } else {
    gIn = micros() - gStart + 20;
    sei();
  }
}

ISR(INT2_vect)
{
  if (digitalRead(MOD_IN) == HIGH) {
    sStart = micros();
  } else {
    int sIn = micros() - sStart + 20;
    sei();
    sw = mode2D = sIn < swLevel;
  }
}


void setTimer(int lapse)
{
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = - lapse * 16;

  TCCR1B |= (1 << CS10);  // enable Timer1 overflow interrupt
}

float getValue(int v1, int v2, int v)
{
  if (v < v1)
    return 0.0;
  else if (v > v2)
    return 1.0;

  return (v - v1) * 1.0 / (v2 - v1);
}

float setValue(int v1, int v2, float value)
{
  int v = v1 + value * (v2 - v1);

  if (v < v1)
    return v1;

  if (v > v2)
    return v2;

  return v;
}


void buzz(int num, int millsOn, int millsOff)
{
  buzzOn = true;
  delay(millsOn);

  for (int i = 0; i < num; i++) {
    digitalWrite(ZUMM_OUT, LOW);
    delay(millsOn);
    digitalWrite(ZUMM_OUT, HIGH);
    delay(millsOff);
  }
  buzzOn = false;
}

boolean isLess = false;

void disp(boolean soundOn)
{
  if (curChannel == 0)
    return;

  int vOld = EEPROM.read(curChannel);
  int v = param[curChannel] * 256.0;

  if (!soundOn) {
    isLess = v < vOld;
    return;
  }

  if (v > vOld + 1) {
    if (isLess)
      buzz(10, 1, 1);
    digitalWrite(LED_OUT, LOW);
    isLess = false;
  } else if (v < vOld - 1) {
    if (!isLess)
      buzz(10, 1, 1);
    digitalWrite(LED_OUT, HIGH);
    isLess = true;
  }
}

