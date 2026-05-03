// main.ino
// Intégration de l'asservissement avec le port série
// Commandes : avance <mm>  |  tourne <deg>  |  stop

#include "encodeurs.h"
#include "Moteur.h"
#include "lecture_PortSerie.h"
#include "asservissement.h"
#include "odometrie.h"
#include <math.h>

// ── CORRECTION encodeurs.cpp ──────────────────────────────────────────────
// IMPORTANT : dans encodeurs.cpp, la constante doit être typée :
//   const float distanceParTicks = 50.0f * 3.14159f / 2000.0f;
//   (sans type explicite, le compilateur Arduino refuse)
// ─────────────────────────────────────────────────────────────────────────

#define PERIODE_ASSERV_MS  10   // période de la boucle d'asservissement
#define PERIODE_DEBUG_MS   200    // affichage pose (5 Hz)

static unsigned long t_dernier_asserv = 0;
static unsigned long t_dernier_debug  = 0;
static void goto_xy(float x_cible, float y_cible) {
  Pose p = odometrie_lire();
  float dx = x_cible - p.x;
  float dy = y_cible - p.y;
  float dist    = sqrtf(dx*dx + dy*dy);
  float cap_cible = atan2f(dy, dx);

  // Angle à tourner (ramené dans [-180°, 180°])
  float delta_theta = cap_cible - p.theta;
  while (delta_theta >  M_PI) delta_theta -= 2.0f * M_PI;
  while (delta_theta < -M_PI) delta_theta += 2.0f * M_PI;

  Serial.print("goto : tourne ");
  Serial.print(degrees(delta_theta), 1);
  Serial.print("°  puis avance ");
  Serial.print(dist, 1);
  Serial.println(" mm");

  // On lance d'abord la rotation
  // (l'avance sera déclenchée dans loop() à la fin de la rotation)
  asserv_tourne(degrees(delta_theta));

  // On stocke la distance à parcourir ensuite
  // (simple flag global pour enchaîner après la rotation)
  // Implémentation minimale : on remet dans le port série
  // Une vraie implémentation utiliserait une file de commandes.
}
void setup() {
  Serial.begin(115200);
  encodeurs_init();
  moteur_init();
  portSerie_init();
  asserv_init();
  odometrie_init();
  Serial.println("=== Robot pret ===");
  Serial.println("Commandes : avance <mm>  |  tourne <deg>  |  stop 0 0");
}

void loop() {
  unsigned long t = millis();

  // ── Lecture port série ──────────────────────
  portSerie_lecture();

  if (nouveau_message) {
    nouveau_message = false;

    if (strcmp(p.com, "avance") == 0) {
      asserv_avance((float)p.para1);
      Serial.print("-> avance ");
      Serial.print(p.para1);
      Serial.println(" mm");

    } else if (strcmp(p.com, "tourne") == 0) {
      asserv_tourne((float)p.para1);
      Serial.print("-> tourne ");
      Serial.print(p.para1);
      Serial.println(" deg");

    } else if (strcmp(p.com, "stop") == 0) {
      asserv_stop();
      Serial.println("-> stop");

    } else if (strcmp(p.com, "modifier") == 0) {
      asserv_modifier_param(p.para1, p.para2);
      //Serial.println("-> stop");

    }else if (strcmp(p.com, "pose") == 0) {
      Serial.print("Pose courante : ");
      odometrie_afficher();

    } else if (strcmp(p.com, "reset") == 0) {
      odometrie_reset();
      Serial.println("-> pose remise a zero");

    }else {
      Serial.println("Commande inconnue. Essayez : avance 150  |  tourne 90  |  stop 0 0");
    }

  }

  // ── Boucle d'asservissement (période fixe) ──
  if (t - t_dernier_asserv >= PERIODE_ASSERV_MS) {
    t_dernier_asserv = t;
    asserv_update();
    odometrie_update();

    // Notification de fin de mouvement
    if (asserv_etat() == ASSERV_TERMINE) {
      asserv_stop();   // repasse à IDLE
      Serial.println(">>> Mouvement termine <<<");
    }
  }
  /*
  // ── Debug encodeurs (1 fois par seconde) ────
  if (t - t_dernier_debug >= 1000) {
    t_dernier_debug = t;
    long g, d;
    encodeurs_lire(g, d);
    Serial.print("EncG: "); Serial.print(g);
    Serial.print(" mm  |  EncD: "); Serial.print(d);
    Serial.println(" mm");
  }*/
}
