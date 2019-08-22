/*
  Relay.cpp - Library for driving relay with delay
  Created by L.Juarez , August 21, 2019.
  Released into the public domain.
*/

#include "Arduino.h"
#include "Relay.h"

Relay::Relay(byte pin, unsigned long valTempo, int initState)
{
    gpio = pin;
    state = initState;
    _lastChangeTime = superMillis();
    Temporisation = valTempo;
    _waitTemporisation = (unsigned long)valTempo / 10;
    //
    pinMode(gpio,OUTPUT);
    delay(500);
    digitalWrite(gpio,state); // Init all pin to LOW
    delay(_waitTemporisation);
};
void Relay::open()
{
    openCloseRelay(HIGH);
};
void Relay::close()
{
    openCloseRelay(LOW);
};
void Relay::set(int newState) 
{
    openCloseRelay(newState);
};
int Relay::get()
{
  if (state == LOW) return 1;     //relay active LOW
  if (state == HIGH)  return 0;
}

void Relay::switchS()
{
  if (state == LOW) 
  {
    openCloseRelay(HIGH);
  }else {
    if (state == HIGH) openCloseRelay(LOW);
  } 
  
}
void Relay::openCloseRelay(int newState)
{
  if (newState != state){
    while (_lastChangeTime + Temporisation >= superMillis()) {
      delay(_waitTemporisation);
    }
    state=newState;
    digitalWrite(gpio,state);
    _lastChangeTime = superMillis();  
  }
};

int Relay::on()
{
  return (LOW);   
}
int Relay::off()
{
  return (HIGH);
}

/** 
 * Retourne le nombre de millisecondes depuis le démarrage du programme.
 *
 * @return Le nombre de millisecondes depuis le démarrage du programme sous la forme d'un
 * nombre entier sur 64 bits (unsigned long long).
 */
unsigned long long Relay::superMillis() {
  static unsigned long nbRollover = 0;
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  
  if (currentMillis < previousMillis) {
     nbRollover++;
  }
  previousMillis = currentMillis;

  unsigned long long finalMillis = nbRollover;
  finalMillis <<= 32;
  finalMillis +=  currentMillis;
  return finalMillis;
};

#ifdef DEBUG
void print_unsigned_long_long(unsigned long long value) {
  // Serial.print ne gére pas directement les unsigned long long.
  // Cette fonction permet d'afficher une unsigned long long sur le port série.
  
  unsigned long msb = value >> 32;
  unsigned long lsb = value & 0xFFFFFFFF;
  if (msb) {
    char buf[21];
    sprintf(buf, "%lu%010lu", msb, lsb);
    Serial.println(buf);
  } else {
    Serial.println(lsb);
  }
};
#endif
