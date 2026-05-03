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
void moteur_set(int mot, float vitesse) {
  int PWM = constrain(round(vitesse), -255, 255);

  int pinA = 0, pinB = 0;
  if (mot == 2) { pinA = M1A; pinB = M1B; }
  else if (mot == 1) { pinA = M2A; pinB = M2B; }
  else return; // moteur invalide

  if (PWM > 0) {
    analogWrite(pinA, PWM);
    analogWrite(pinB, 0);
  } else if (PWM < 0) {
    analogWrite(pinA, 0);
    analogWrite(pinB, -PWM);
  } else {
    analogWrite(pinA, 0);
    analogWrite(pinB, 0);
  }
}



// === Arrêt d’urgence ===
void moteur_stop() {
  moteur_set(1, 0);
  moteur_set(2, 0);
}