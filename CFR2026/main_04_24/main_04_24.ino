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
  Serial.println(p.x);
  if (strcmp(p.com, "avance") == 0){moteur_set(1, p.x); moteur_set(2,p.x);Serial.println("on avance");}
  if (strcmp(p.com, "stop") == 0){moteur_stop();}
  
  }
  //else
  //{Serial.println("pas de nouveau message");}

  delay(1000);
}