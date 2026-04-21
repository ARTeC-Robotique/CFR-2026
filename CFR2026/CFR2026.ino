#include "encodeurs.h"

void setup() {
  Serial.begin(115200);
  encodeurs_init();
  Serial.println("Pret.");
}

void loop() {
  long encodeurG, encodeurD;
  encodeurs_lire(encodeurG, encodeurD);

  Serial.print("EncG: ");
  Serial.print(encodeurG);
  Serial.print("  |  EncD: ");
  Serial.println(encodeurD);

  delay(200);
}