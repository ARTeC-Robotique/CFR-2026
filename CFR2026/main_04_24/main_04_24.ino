#include "encodeurs.h"
#include "Moteur.h"
#include "lecture_PortSerie.h"

void setup() {
  Serial.begin(115200);
  encodeurs_init();
  portSerie_init();
  moteur_init();
  Serial.println("Pret.");
}

void loop() {
  long encodeurG, encodeurD;
  encodeurs_lire(encodeurG, encodeurD);

  portSerie_lecture();

  /*
  Serial.print("EncG: ");
  Serial.print(encodeurG);
  Serial.print("  |  EncD: ");
  Serial.println(encodeurD);
  */

  if (nouveau_message){
  nouveau_message=false;
  Serial.print("commande : ");
  Serial.print(p.com);
  Serial.print(", X : ");
  Serial.println(p.x);}
  else
  {Serial.println("pas de nouveau message");}

  delay(3000);
}