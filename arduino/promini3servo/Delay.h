#ifndef NonBlockDelay_H
#define NonBlockDelay_H

#if defined(ARDUINO) && ARDUINO >= 100  
#include <Arduino.h>  
#else  
#include <WProgram.h>  
#endif

class NonBlockDelay
{  
	unsigned long iTimeout;  
public:  
	void Delay(unsigned long);  
	bool Timeout(void);  
	unsigned long Time(void);  
};

#endif

/*
https://www.element14.com/community/community/arduino/blog/2014/06/05/a-non-blocking-delay

Usage:

Without non blocking delay:

// ---
int led = 13;
  
void setup() {
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
}

void loop() {
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);               // wait for a second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);               // wait for a second
  //Code here runs once every 2 seconds
}
// ---

With non blocking delay:

// ---
#include "Delay.h"

int led = 13;
int ledstate = HIGH;
NonBlockDelay d;

void setup() {
  pinMode(led, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  if(d.Timeout()) {
    digitalWrite(led, ledstate);
    ledstate = ledstate == LOW ? HIGH : LOW; //toggle state
    d.Delay(1000);
  }
  // Code here runs frequently
}
// ---
*/
