/*
  Relay.h - Library for driving relay with delay
  Created by L.Juarez , August 21, 2019.
  Released into the public domain.
*/
/*
#ifndef Relay_h
#define Relay_h
*/
#include "Arduino.h"

class Relay
{
  public:
  // Class Member Variables
  // These are initialized at startup
  byte  gpio;
  int  state;
  unsigned long Temporisation;   // wait time between changes
  
  // These maintain the current state
  private:
  unsigned long long _lastChangeTime;
  unsigned long _waitTemporisation;
  
  public:
  Relay (byte pin = 0, unsigned long valTempo = 1000, int initState = LOW);
  void open();
  void close();
  void set(int newState);
  int get();   // return state
  int on();   
  int off();
  void switchS();   

  private:
  void openCloseRelay(int newState);
  unsigned long long superMillis();
};
