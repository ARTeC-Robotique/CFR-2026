#include "api/Common.h"
// asservissement.cpp
// Asservissement PD en position avec rampe d'accélération
// Commandes : avance <mm>, tourne <degrés>, stop

#include "asservissement.h"
#include "encodeurs.h"
#include "Moteur.h"
#include <Arduino.h>
#include <math.h>

// ──────────────────────────────────────────────
//  Géométrie du robot
// ──────────────────────────────────────────────
// Voie (distance entre les deux roues, en mm)
// À MESURER et ajuster selon votre robot
#define VOIE_MM        224.0f

// ──────────────────────────────────────────────
//  Paramètres PD  (à calibrer)
// ──────────────────────────────────────────────
// Gain proportionnel : PWM par mm d'erreur
float KP_DIST=2; // correct  2.5
float KD_DIST=1.0;   // amortissement sur la vitesse (PWM / (mm/s))   correct 1, plus c'est plus lent
float KP_ANG=2.5;
float KD_ANG=1.0;

// ──────────────────────────────────────────────
//  Bridage de puissance
// ──────────────────────────────────────────────
// PWM maximum autorisé (0-255)
float PWM_MAX=80;
// Incrément max de PWM par appel (rampe d'accélération)
// À 10 ms de période de boucle → +12 PWM/10ms = 0→PWM_MAX en ~133 ms
float RAMPE_STEP=2;

// ──────────────────────────────────────────────
//  Seuils d'arrêt
// ──────────────────────────────────────────────
float SEUIL_DIST_MM=4;   // erreur résiduelle tolérée en mm
float SEUIL_ANG_MM=3.0;   // erreur angulaire tolérée (en mm de différentiel)

void asserv_modifier_param(float val1, float valeur) {
  int index = int(val1);
  switch (index) {
    case 1:  KP_DIST           = valeur; break;
    case 2:  KD_DIST         = valeur; break;
    case 3:  KP_ANG         = valeur; break;
    case 4:  KD_ANG            = valeur; break;
    case 5:  PWM_MAX         = valeur; break;
    case 6:  RAMPE_STEP         = valeur; break;
    case 7:  SEUIL_DIST_MM        = valeur; break;
    case 8:  SEUIL_ANG_MM     = valeur;break;/*
             pid_vit_G.sortie_max =  valeur; pid_vit_G.sortie_min = -valeur;
             pid_vit_D.sortie_max =  valeur; pid_vit_D.sortie_min = -valeur; break;
    case 9:  VIT_MAX_AVANCE    = valeur; break;
    case 10: VIT_MAX_TOURNE    = valeur; break;
    case 11: SEUIL_ARRET_MM    = valeur; break;
    case 12: SEUIL_ARRET_RAD   = valeur * (M_PI / 180.0f); break;
    case 13: DIST_DECEL_AVANCE = valeur; break;
    case 14: DIST_DECEL_TOURNE = valeur; break;*/
    default:
      Serial.println("[PARAM] index inconnu (1-14)");
      return;
  }
  Serial.print("[PARAM] "); Serial.print(index);
  Serial.print(" = "); Serial.println(valeur);
}




// ──────────────────────────────────────────────
//  Variables d'état internes
// ──────────────────────────────────────────────
static float  cible_dist  = 0.0f;   // distance à parcourir (mm, signée)
static float  cible_ang   = 0.0f;   // arc à parcourir par chaque roue pour tourner (mm)
static float  pos_dist_0  = 0.0f;   // position initiale au démarrage de la commande
static float  pos_ang_0   = 0.0f;   // cap initial

static float  err_dist_prec = 0.0f;
static float  err_ang_prec  = 0.0f;

static float  pwm_gauche_reel = 0.0f;  // PWM actuel après rampe
static float  pwm_droite_reel = 0.0f;

static EtatAsserv etat = ASSERV_IDLE;
static unsigned long t_precedent = 0;

// ──────────────────────────────────────────────
//  Lecture position instantanée
// ──────────────────────────────────────────────
static void lire_position(float &dist, float &ang) {
  long g, d;
  encodeurs_lire(g, d);
  // dist = avance moyenne, ang = différentiel (proportionnel à la rotation)
  dist = (g + d) * 0.5f;
  ang  = (d - g) * 0.5f;   // positif = rotation horaire
}

// ──────────────────────────────────────────────
//  API publique
// ──────────────────────────────────────────────

void asserv_init() {
  etat = ASSERV_IDLE;
  t_precedent = millis();
}

void asserv_avance(float mm) {
  float dist, ang;
  lire_position(dist, ang);
  pos_dist_0   = dist;
  pos_ang_0    = ang;          // on veut garder le cap courant
  cible_dist   = mm;
  cible_ang    = 0.0f;         // pas de rotation
  err_dist_prec = 0.0f;
  err_ang_prec  = 0.0f;
  pwm_gauche_reel = 0.0f;
  pwm_droite_reel = 0.0f;
  etat = ASSERV_EN_COURS;
}

// angle_deg > 0 : rotation horaire (vue du dessus)
void asserv_tourne(float angle_deg) {
  float dist, ang;
  lire_position(dist, ang);
  pos_dist_0  = dist;
  pos_ang_0   = ang;
  cible_dist  = 0.0f;
  // arc = (voie/2) * angle_rad
  cible_ang   = (VOIE_MM / 2.0f) * (angle_deg * M_PI / 180.0f);
  err_dist_prec = 0.0f;
  err_ang_prec  = 0.0f;
  pwm_gauche_reel = 0.0f;
  pwm_droite_reel = 0.0f;
  etat = ASSERV_EN_COURS;
}

void asserv_stop() {
  moteur_stop();
  pwm_gauche_reel = 0.0f;
  pwm_droite_reel = 0.0f;
  etat = ASSERV_IDLE;
}

EtatAsserv asserv_etat() {
  return etat;
}

// ──────────────────────────────────────────────
//  Boucle principale  (à appeler toutes les ~10 ms)
// ──────────────────────────────────────────────
void asserv_update() {
  if (etat == ASSERV_IDLE) return;

  // ── dt ──────────────────────────────────────
  unsigned long t_now = millis();
  float dt = (t_now - t_precedent) * 0.001f;  // en secondes
  if (dt <= 0.0f) return;
  t_precedent = t_now;

  // ── Lecture position ────────────────────────
  float dist, ang;
  lire_position(dist, ang);

  // ── Erreurs ─────────────────────────────────
  float err_dist = cible_dist - (dist - pos_dist_0);
  float err_ang  = cible_ang  - (ang  - pos_ang_0);

  // ── Dérivées (vitesses d'erreur) ─────────────
  float derr_dist = (err_dist - err_dist_prec) / dt;
  float derr_ang  = (err_ang  - err_ang_prec)  / dt;
  err_dist_prec = err_dist;
  err_ang_prec  = err_ang;

  // ── Commandes PD ────────────────────────────
  float u_dist = KP_DIST * err_dist + KD_DIST * derr_dist;
  float u_ang  = KP_ANG  * err_ang  + KD_ANG  * derr_ang;

  // ── Mix gauche / droite ──────────────────────
  //   avance : les deux roues dans le même sens
  //   rotation : roues en sens opposé
  float cmd_gauche = u_dist - u_ang;
  float cmd_droite = u_dist + u_ang;

  // ── Rampe d'accélération ─────────────────────
  // On limite le saut de PWM à RAMPE_STEP par cycle
  auto rampe = [](float cible, float actuel) -> float {
    float delta = cible - actuel;
    delta = constrain(delta, -(float)RAMPE_STEP, (float)RAMPE_STEP);
    return actuel + delta;
  };

  pwm_gauche_reel = rampe(cmd_gauche, pwm_gauche_reel);
  pwm_droite_reel = rampe(cmd_droite, pwm_droite_reel);

  // ── Saturation PWM ───────────────────────────
  float pg = constrain(pwm_gauche_reel, -(float)PWM_MAX, (float)PWM_MAX);
  float pd = constrain(pwm_droite_reel, -(float)PWM_MAX, (float)PWM_MAX);

  // ── Envoi aux moteurs ────────────────────────
  moteur_set(1, pg);   // 1 = gauche
  moteur_set(2, pd);   // 2 = droite

  // ── Détection de fin de mouvement ────────────
  if (fabsf(err_dist) < SEUIL_DIST_MM && fabsf(err_ang) < SEUIL_ANG_MM) {
    moteur_stop();
    pwm_gauche_reel = 0.0f;
    pwm_droite_reel = 0.0f;
    etat = ASSERV_TERMINE;
  }
}


