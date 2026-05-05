#include "trapeze.h"
#include <Arduino.h>
#include "encodeurs.h"
#include "Moteur.h"

const float accmax = 100;
const float vmax = 100;

float trapeze(float consigne, float consigneP) {
  if (consigne > vmax) consigne = vmax;
  if (consigne < -vmax) consigne = -vmax;
  float diff = consigne - consigneP;
  float direction = (diff > 0) ? 1 : -1;
  if (abs(diff) > accmax) {
   consigne = consigneP + direction * accmax * Te;
   return consigne;
  }
}