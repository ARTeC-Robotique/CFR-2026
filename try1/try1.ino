#include "lecture_PortSerie.h"
#include "Moteur.h"
#include "encodeurs.h"

extern Point p;
bool nouveau_message = false;

void setup() {
  Serial.begin(115200);
  delay(2000); 
  Serial.println("Robot Pret.");

  portSerie_init();
  moteur_init();
  encodeurs_init();
}

void loop() {
  delay(Te);
  portSerie_lecture();
   if (nouveau_message == true) {

    if (strcmp(p.com, "roule") == 0) {
      Serial.print("Roule avec ");
      Serial.print(p.para1);
      Serial.println(" de tension !");
      avance(p.para1);
    }

    else if (strcmp(p.com, "tourne") == 0) {
      Serial.print("Tourne avec ");
      Serial.print(p.para1);
      Serial.println(" de tension !");
      tourne(p.para1);
    }

    else if (strcmp(p.com, "moteur") == 0) {
      Serial.print("Moteur ");
      Serial.print(p.para1);
      Serial.print(" tension de ");
      Serial.println(p.para2);
      moteur_set(p.para1,p.para2);
    }

    else if (strcmp(p.com, "stop") == 0) {
      if (p.para1 ==0) {moteur_stop();}
      else {Serial.print("Moteur ");
      Serial.print(p.para1);
      Serial.print(" stop");
      moteur_set(p.para1,0);}}

    else {Serial.println("Commande non prise en charge sorry");}
    nouveau_message = false;}
}
