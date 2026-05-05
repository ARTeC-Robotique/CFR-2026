#include <Arduino.h>
#include "Moteur.h"

extern float Te = 10;
extern float consigne = 0;

const float K_tension = 1.5; //DIFF TENSION ENTRE MOTEURS (a determiner)
const float RapportUV = 1;  //le rapport entre tension et vitesse de rotation (a determiner, eske il est constant ?)
// === Broches des moteurs ===
// Moteur droit (1)
#define M1A 10
#define M1B 11
// Moteur gauche (2)
#define M2A 9
#define M2B 6

void moteur_init() {
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);
  moteur_stop(); 
  Serial.println("Moteurs prets !");}

void moteur_set(int mot, float vitesse) {
  int PWM = constrain(round(vitesse), -255, 255);
  int pinA = 0, pinB = 0;

  if (mot == 1) { pinA = M2A; pinB = M2B; }
  else if (mot == 2) {pinA = M1A; pinB = M1B;}

  if (PWM > 0) {
    analogWrite(pinA, PWM);
    analogWrite(pinB, 0);
  } else if (PWM < 0) {
    analogWrite(pinA, 0);
    analogWrite(pinB, -PWM);
  } else {
    analogWrite(pinA, 0);
    analogWrite(pinB, 0);
  }}

void moteur_stop() {
  moteur_set(1, 0);
  moteur_set(2, 0);}

void avance(float vitesse) {
  vitesse = constrain(vitesse, -255, 255);
  moteur_set(1, vitesse);
  moteur_set(2, K_tension*vitesse);}

void tourne(float vitesse) {
  vitesse = constrain(vitesse, -255, 255);
  moteur_set(1, vitesse);
  moteur_set(2, K_tension*(-vitesse));
}
