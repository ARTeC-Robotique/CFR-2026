#include <Arduino.h>
#include "Moteur.h"


// === Broches des moteurs ===
// Moteur droit (1)
#define M1A 10
#define M1B 11
// Moteur gauche (2)
#define M2A 9
#define M2B 6

// === Initialisation des broches moteurs ===
void moteur_init() {
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);
  moteur_stop(); // sécurité : démarre à l'arrêt
}

// === Commande d'un seul moteur ===
// mot : 1 (gauche) ou 2 (droit)
// vitesse : [-255, 255]
float coefficientCorrectifD=1;
float coefficientCorrectifG=0.95;
void moteur_set(int mot, float vitesse) {
  int PWM = constrain(round(vitesse), -255, 255);
  float coefficientCorrectif;
  int pinA = 0, pinB = 0;
  if (mot == 2) { pinA = M1A; pinB = M1B; coefficientCorrectif=coefficientCorrectifD; }
  else if (mot == 1) { pinA = M2A; pinB = M2B; coefficientCorrectif=coefficientCorrectifG; }
  else return; // moteur invalide

  if (PWM > 0) {
    analogWrite(pinA, int(PWM*coefficientCorrectif));
    analogWrite(pinB, 0);
  } else if (PWM < 0) {
    analogWrite(pinA, 0);
    analogWrite(pinB, -int(PWM*coefficientCorrectif));
  } else {
    analogWrite(pinA, 0);
    analogWrite(pinB, 0);
  }
}
/*
void moteur_debuggage(float gauche, float droite){
  moteur_set(1, gauche/coefficientCorrectifG);
  moteur_set(2, droite/coefficientCorrectifD);
}*/


// === Arrêt d’urgence ===
void moteur_stop() {
  moteur_set(1, 0);
  moteur_set(2, 0);
}
