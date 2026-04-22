#include "encodeurs.h"
#include "Moteur.h"


void setup() {
  initMoteurs(); //1 Droite, 2 Gauche
}

void loop() {
  // Exemple de test : faire avancer le robot à moitié vitesse pendant 2 secondes
  setMotor(1, 0); // Moteur droit et gauche à moitié vitesse
  setMotor(2, 0);
  delay(2000);
  
  // Puis arrêter les moteurs
  stopMoteurs();
  delay(2000);
}
