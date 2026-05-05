#include "encodeurs.h"
#include <Arduino.h>

// Ticks par cm (en ligne droite) (a verif)
#define TICKS_CM_R 127.277
#define TICKS_CM_L 126.238

// Ticks par radian pour rotation horaire et trigonométrique
#define TICKS_RAD_HR 2225.145
#define TICKS_RAD_HL 2331.779
#define TICKS_RAD_TR 2373.319
#define TICKS_RAD_TL 2340.214

#define ENC1_A 2
#define ENC1_B 4
#define ENC2_A 3
#define ENC2_B 5

volatile long countR = 0;
volatile long countL = 0;
long prevCountR = 0;
long prevCountL = 0;

float dR = 0;
float dL = 0;


void encodeur1() {
  countR += (digitalRead(ENC1_A) == digitalRead(ENC1_B)) ? 1 : -1;}

void encodeur2() {
  countL += (digitalRead(ENC2_A) == digitalRead(ENC2_B)) ? -1 : 1;}
  

void encodeurs_init() {
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1_A), encodeur1, CHANGE);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), encodeur2, CHANGE);
  Serial.println("Encodeurs ok !");}

void lire_encodeurs(long &c1, long &c2) {
  noInterrupts();
  long currentR = countR;
  long currentL = countL;
  interrupts();
  
  dR = currentR - prevCountR;
  dL = currentL - prevCountL;

  prevCountR = currentR;
  prevCountL = currentL;
  }

float distEncodeurs() {
  return (dR / TICKS_CM_R + dL / TICKS_CM_L) * 0.5;
}

float angleEncodeurs() {
  float dRrad = (dR > 0) ? dR / TICKS_RAD_HR : dR / TICKS_RAD_TR;
  float dLrad = (dL > 0) ? dL / TICKS_RAD_HL : dL / TICKS_RAD_TL;
  return (dRrad - dLrad) * 0.5;}
