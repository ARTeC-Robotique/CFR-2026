#include "encodeurs.h"
#include <Arduino.h>

#define ENC1_A 2
#define ENC1_B 4
#define ENC2_A 3
#define ENC2_B 5

volatile long count1 = 0;
volatile long count2 = 0;
const float distanceParTicks=50*3.1415/2000;

void isr_enc1() {
  static bool lastA = LOW;
  bool A = digitalRead(ENC1_A);
  bool B = digitalRead(ENC1_B);
  if (A != lastA) {
    count1 += (A != B) ? -1 : +1;
    lastA = A;
  }
}

void isr_enc2() {
  static bool lastA = LOW;
  bool A = digitalRead(ENC2_A);
  bool B = digitalRead(ENC2_B);
  if (A != lastA) {
    count2 += (A != B) ? +1 : -1;
    lastA = A;
  }
}

void encodeurs_init() {
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1_A), isr_enc1, CHANGE);

  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), isr_enc2, CHANGE);
}

void encodeurs_lire(long &c1, long &c2) {
  noInterrupts();
  c2 = count1*distanceParTicks;
  c1 = count2*distanceParTicks;//j'inverse pour le droite gauche
  interrupts();
}