/* 
  TN.h - Library for TN-05
  Tangible Networks
  Espen Knoop, 25th Feb 2015
*/

#ifndef TN05_h
#define TN05_h

#include "Arduino.h"

// Clear bit, set bit
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif 


// Fast min, max
#define MIN(x,y) x < y ? x : y
#define MAX(x,y) x > y ? x : y


// Pins
#define LED_R 11
#define LED_G 6
#define LED_B 5
#define POT A7
#define SW 2
#define DIP2 12
#define DIP1 13
#define SPKR 9

#define IN0 A0
#define IN1 A1
#define IN2 A2
#define IN3 A3
#define IN4 A4
#define IN5 A5
#define MSTR_A A6
#define MSTR_D 4


// For DAC
#define I2C_CLK_DDR DDRB
#define I2C_DAT_DDR DDRD
#define I2C_CLK_PIN PINB
#define I2C_DAT_PIN PIND
#define I2C_CLK_PORT PORTB
#define I2C_DAT_PORT PORTD
#define I2C_CLK 0
#define I2C_DAT 7
#define I2C_CLOCK_LO()    cbi(I2C_CLK_PORT,I2C_CLK)
#define I2C_CLOCK_HI()    sbi(I2C_CLK_PORT,I2C_CLK) 
#define I2C_CLOCK_PULSE() {delayMicroseconds(5); I2C_CLOCK_HI(); delayMicroseconds(5); I2C_CLOCK_LO();}
#define I2C_DATA_LO()     cbi(I2C_DAT_PORT,I2C_DAT)
#define I2C_DATA_HI()     sbi(I2C_DAT_PORT,I2C_DAT)
#define I2C_DATA_SET(v)   if(v){I2C_DATA_HI();}else{I2C_DATA_LO();}
#define I2C_WRITEBIT(v)   {I2C_DATA_SET(v); I2C_CLOCK_PULSE();}



class TN {
  public:
    // Constructor
    TN(double minVal=0.0, double maxVal=1.0);
    
    // Set LED to RGB colour (ints, [0, 255])
    void colour(int r, int g, int b);
    
    // Set LED to RGB colour (doubles, [0.0, 1.0])
    void colour(double r, double g, double b);
    
    // Check if an input is connected
    boolean isConnected(int input);
    
    // Read the analog value from input (returns double in [minVal, maxVal])
    // Returns minVal if input is not connected
    double analogRead(int input);
    
    // Write value to output (will be clipped to range [minVal, maxVal]
    void analogWrite(double value);
    
    // Read the digital value (0 or 1) from input (returns 0 if not connected)
    int digitalRead(int input);

    // Write digital value (0 or 1) to output
    void digitalWrite(int value);
    
    // Get state of DIP switches (1 for on)
    boolean dip2();
    boolean dip1();
    
    // Get state of switch (1 is pressed)
    boolean sw();
    
    // Get position of pot (double, [0.0, 1.0])
    double pot();

    // 1 if master controller is connected
    boolean masterConnected();
    
    // Value of master controller (double, [0.0, 1.0])
    double masterRead();  

    // Read state of master sw (1 is pressed)
    boolean masterSw();

    // Print current state to serial
    void printState();
  private:
    
    // For dac
    void _dacSetup();
    void _startcond();
    void _stopcond();
  
    // Defines range of inputs/outputs (set in constructor)
    double _minVal;
    double _maxVal;

    // Stores current state (for printing)
    int _colour[3];
    double _out;
    double _ins[6];
    double _pot;
    double _mstr_A;
    boolean _mstr_D;
    boolean _dips[2];
    boolean _sw;
    int _IN[6];// = {IN0, IN1, IN2, IN3, IN4, IN5};
};


#endif


