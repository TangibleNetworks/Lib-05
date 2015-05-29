/* 
  TN.cpp - Library for TN-05
  Tangible Networks
  Espen Knoop, 28th May 2015
*/


#include "Arduino.h"
#include "TN05.h"


// Constructor
TN::TN(double minVal, double maxVal) {
  _minVal = minVal;
  _maxVal = maxVal;
  _IN[0] = IN0;
  _IN[1] = IN1;
  _IN[2] = IN2;
  _IN[3] = IN3;
  _IN[4] = IN4;
  _IN[5] = IN5;
  
  for (int i=0; i<6; i++) {
    pinMode(_IN[i],INPUT);
  }
  pinMode(LED_R,OUTPUT);
  pinMode(LED_G,OUTPUT);
  pinMode(LED_B,OUTPUT);
  pinMode(SW,INPUT);
  pinMode(MSTR_D,INPUT);
  pinMode(DIP0,INPUT);
  pinMode(DIP1,INPUT);
  ::digitalWrite(DIP0,HIGH);
  ::digitalWrite(DIP1,HIGH);
  ::digitalWrite(SW,HIGH);
  _dacSetup();
  // Go through all the functions, to initialise state vars
  analogWrite(_minVal);
  for (int i=0; i<6; i++) {
    analogRead(i);
  }
  colour(0,0,0);
  dip0();
  dip1();
  masterRead();
  masterSw();
  pot();
  sw();
}


// For DAC
void TN::_dacSetup() {
  // Set pins to output
  sbi(I2C_CLK_DDR,I2C_CLK);
  sbi(I2C_DAT_DDR,I2C_DAT);
}

inline void TN::_startcond() {
  I2C_DATA_LO();
  delayMicroseconds(5);
  I2C_CLOCK_LO();
  ;
}

inline void TN::_stopcond() {
  I2C_CLOCK_HI();
  delayMicroseconds(5);
  I2C_DATA_HI();
  delayMicroseconds(5);
}

// Write value to output (will be clipped to range [minVal, maxVal]
void TN::analogWrite(double doubleVal) {
  doubleVal = MAX(doubleVal,_minVal);
  doubleVal = MIN(doubleVal,_maxVal);
  unsigned int value = floor( 205.0 + 3686.0* (doubleVal - _minVal)/(_maxVal - _minVal));
  value = MAX(value,205);
  value = MIN(value,3891);
  _out = doubleVal;
  _startcond();
  I2C_WRITEBIT(1);
  I2C_WRITEBIT(1);
  I2C_WRITEBIT(0);
  I2C_WRITEBIT(0);
  I2C_WRITEBIT(0);
  I2C_WRITEBIT(0);
  I2C_WRITEBIT(0);
  I2C_WRITEBIT(0);
  
  I2C_WRITEBIT(1);
  
  I2C_WRITEBIT(0);
  I2C_WRITEBIT(0);
  I2C_WRITEBIT(0);
  I2C_WRITEBIT(0);

  for (int i=11; i > 7; i--) {
    I2C_WRITEBIT(1 & (value >> i) );
  }
  
  I2C_WRITEBIT(1);
  
  for (int i=7; i >= 0; i--) {
    I2C_WRITEBIT(1 & (value >> i) );
  }
  
  I2C_WRITEBIT(1);
  _stopcond();
}


// Set LED to RGB colour (ints, [0, 255])
void TN::colour(int r, int g, int b) {
  r = MAX(r,0);
  g = MAX(g,0);
  b = MAX(b,0);
  r = MIN(r,255);
  g = MIN(g,255);
  b = MIN(b,255);
  
  _colour[0] = r;
  _colour[1] = g;
  _colour[2] = b;
  ::analogWrite(LED_R,r);
  ::analogWrite(LED_G,g);
  ::analogWrite(LED_B,b);
}


// Set LED to RGB colour (doubles, [0.0, 1.0])
void TN::colour(double r, double g, double b) {
  colour((int)(r*255.0),(int)(g*255.0),(int)(b*255.0));
}
 

// Check if an input is connected
boolean TN::isConnected(int input) {
  int idx = MIN(input,5);
  idx = MAX(idx,0);
  return (::analogRead(_IN[idx]) < 990); 
}


// Read the analog value from input (returns double in [minVal, maxVal])
// Returns minVal if input is not connected
double TN::analogRead(int input) {
  int idx = MAX(input,0);
  idx = MIN(idx,5);
  if (isConnected(input)) {
    int rawVal = ::analogRead(_IN[idx]);
    _ins[idx] = _minVal + (_maxVal - _minVal)*(rawVal-51)/922.0;
    _ins[idx] = MIN(_ins[idx],_maxVal);
    _ins[idx] = MAX(_ins[idx],_minVal);
         
  }
  else _ins[idx] = _minVal;
  return _ins[idx];
}


// Read the digital value (0 or 1) from input (returns 0 if not connected)
int TN::digitalRead(int input) {
  int idx = MIN(input,5);
  idx = MAX(idx,0);
  if (isConnected(input)) {
    _ins[idx] = ::digitalRead(_IN[idx]);
  }
  else _ins[idx] = 0;
  return _ins[idx];
}


// Write digital value (0 or 1) to output
void TN::digitalWrite(int value) {
  if (value) analogWrite(_maxVal);
  else analogWrite(_minVal);
}


// Get state of dip0 (1 is pressed)
boolean TN::dip0() {
  _dips[0] = ! ::digitalRead(DIP0);
  return (_dips[0]);
}

// Get state of dip1 (1 is pressed)
boolean TN::dip1() {
  _dips[1] = ! ::digitalRead(DIP1);
  return (_dips[1]);
}



// Get state of switch (1 is pressed)
boolean TN::sw() {
  _sw = ! ::digitalRead(SW);
  return (_sw);
}


// Get position of pot (double, [0.0, 1.0])
double TN::pot() {
  _pot = ::analogRead(POT)/1023.0;
  return _pot;
}


// 1 if master controller is connected
boolean TN::masterConnected() {
  return (::analogRead(MSTR_A) < 1010); 
}


// Value of master controller (double, [0.0, 1.0])
double TN::masterRead() {
  int mstrAraw = ::analogRead(MSTR_A);
  if (mstrAraw < 1010) {
    //mstrAraw = MINMAX(mstrAraw - 102,0,819);
    mstrAraw -= 102;
    mstrAraw = MIN(mstrAraw,819);
    mstrAraw = MAX(mstrAraw,0);
    _mstr_A = (mstrAraw)/819.0;
    _mstr_A = MAX(_mstr_A,0.0);
    _mstr_A = MIN(_mstr_A,1.0); 
  }
  else _mstr_A = 0.0;
  return _mstr_A;
}



// Get state of master switch (1 is pressed)
boolean TN::masterSw() {
  _mstr_D = ! ::digitalRead(MSTR_D);
  return (_mstr_D);
}


// Print current state to serial 
// (takes ~5 ms to execute.  Requires Serial.begin(115200) in setup(). )
void TN::printState() {
  Serial.print("RGB: ");
  for (int i=0; i<3; i++) {
    Serial.print(_colour[i]);
    Serial.print(", ");
  }
  Serial.print("Ins: ");
  for (int i=0; i<6; i++) {
    Serial.print(_ins[i]);
    Serial.print(", ");
  }
  Serial.print("Out: ");
  Serial.print(_out);
  Serial.print(", ");
  Serial.print("Pot: ");
  Serial.print(_pot);
  Serial.print(", mstr_A: ");
  Serial.print(_mstr_A);
  Serial.print(", mstr_D: ");
  Serial.print(_mstr_D);
  Serial.print(", DIPs: ");
  for (int i=0; i<2; i++) {
    Serial.print(_dips[i]);
    Serial.print(", ");
  }
  Serial.print("Sw: ");
  Serial.println(_sw);
}



    
