// asservissement.h
#pragma once

enum EtatAsserv {
  ASSERV_IDLE,       // rien en cours
  ASSERV_EN_COURS,   // mouvement en cours
  ASSERV_TERMINE     // cible atteinte (lecture unique, repasse à IDLE après)
};

void       asserv_init();
void       asserv_avance(float mm);          // mm > 0 : avant, < 0 : arrière
void       asserv_tourne(float angle_deg);   // degrés, + = horaire (vue dessus)
void       asserv_stop();
EtatAsserv asserv_etat();
void       asserv_update();                  // à appeler dans loop(), toutes les ~10 ms*
void       asserv_modifier_param(float val1, float valeu);
