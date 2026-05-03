#include "api/Common.h"
// asservissement.cpp
// Asservissement PID en position avec rampe d'accélération
// Commandes : avance <mm>, tourne <degrés>, stop

#include "asservissement.h"
#include "encodeurs.h"
#include "Moteur.h"
#include <Arduino.h>
#include <math.h>

// ──────────────────────────────────────────────
//  Géométrie du robot
// ──────────────────────────────────────────────
#define VOIE_MM        224.0f

// ──────────────────────────────────────────────
//  Paramètres PID  (à calibrer)
// ──────────────────────────────────────────────
float KP_DIST = 2.0f;
float KI_DIST = 0.0f;    // commence à 0, augmenter si erreur statique
float KD_DIST = 1.0f;

float KP_ANG  = 2.5f;
float KI_ANG  = 0.0f;
float KD_ANG  = 1.0f;

// Saturation de l'intégrale (anti-windup)
float INTEGRAL_MAX = 30.0f;

// ──────────────────────────────────────────────
//  Bridage de puissance
// ──────────────────────────────────────────────
float PWM_MAX    = 80.0f;
float RAMPE_STEP = 2.0f;

// ──────────────────────────────────────────────
//  Seuils d'arrêt
// ──────────────────────────────────────────────
float SEUIL_DIST_MM = 4.0f;
float SEUIL_ANG_MM  = 3.0f;

// ──────────────────────────────────────────────
//  Modification de paramètres via port série
// ──────────────────────────────────────────────
void asserv_modifier_param(float val1, float valeur) {
  int index = int(val1);
  switch (index) {
    case 1:  KP_DIST       = valeur; break;
    case 2:  KI_DIST       = valeur; break;
    case 3:  KD_DIST       = valeur; break;
    case 4:  KP_ANG        = valeur; break;
    case 5:  KI_ANG        = valeur; break;
    case 6:  KD_ANG        = valeur; break;
    case 7:  INTEGRAL_MAX  = valeur; break;
    case 8:  PWM_MAX       = valeur; break;
    case 9:  RAMPE_STEP    = valeur; break;
    case 10: SEUIL_DIST_MM = valeur; break;
    case 11: SEUIL_ANG_MM  = valeur; break;
    default:
      Serial.println("[PARAM] index inconnu (1-11)");
      return;
  }
  Serial.print("[PARAM] "); Serial.print(index);
  Serial.print(" = "); Serial.println(valeur);
}

// ──────────────────────────────────────────────
//  Variables d'état internes
// ──────────────────────────────────────────────
static float  cible_dist  = 0.0f;
static float  cible_ang   = 0.0f;
static float  pos_dist_0  = 0.0f;
static float  pos_ang_0   = 0.0f;

static float  err_dist_prec  = 0.0f;
static float  err_ang_prec   = 0.0f;
static float  integral_dist  = 0.0f;
static float  integral_ang   = 0.0f;

static float  pwm_gauche_reel = 0.0f;
static float  pwm_droite_reel = 0.0f;

static EtatAsserv etat = ASSERV_IDLE;
static unsigned long t_precedent = 0;

// ──────────────────────────────────────────────
//  Lecture position instantanée
// ──────────────────────────────────────────────
static void lire_position(float &dist, float &ang) {
  long g, d;
  encodeurs_lire(g, d);
  dist = (g + d) * 0.5f;
  ang  = (d - g) * 0.5f;
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
  pos_dist_0      = dist;
  pos_ang_0       = ang;
  cible_dist      = mm;
  cible_ang       = 0.0f;
  err_dist_prec   = 0.0f;
  err_ang_prec    = 0.0f;
  integral_dist   = 0.0f;   // reset intégrale à chaque nouvelle commande
  integral_ang    = 0.0f;
  pwm_gauche_reel = 0.0f;
  pwm_droite_reel = 0.0f;
  etat = ASSERV_EN_COURS;
}

void asserv_tourne(float angle_deg) {
  float dist, ang;
  lire_position(dist, ang);
  pos_dist_0      = dist;
  pos_ang_0       = ang;
  cible_dist      = 0.0f;
  cible_ang       = (VOIE_MM / 2.0f) * (angle_deg * M_PI / 180.0f);
  err_dist_prec   = 0.0f;
  err_ang_prec    = 0.0f;
  integral_dist   = 0.0f;
  integral_ang    = 0.0f;
  pwm_gauche_reel = 0.0f;
  pwm_droite_reel = 0.0f;
  etat = ASSERV_EN_COURS;
}

void asserv_stop() {
  moteur_stop();
  integral_dist   = 0.0f;
  integral_ang    = 0.0f;
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
  float dt = (t_now - t_precedent) * 0.001f;
  if (dt <= 0.0f) return;
  t_precedent = t_now;

  // ── Lecture position ────────────────────────
  float dist, ang;
  lire_position(dist, ang);

  // ── Erreurs ─────────────────────────────────
  float err_dist = cible_dist - (dist - pos_dist_0);
  float err_ang  = cible_ang  - (ang  - pos_ang_0);

  // ── Intégrales (avec anti-windup) ────────────
  integral_dist += err_dist * dt;
  integral_dist  = constrain(integral_dist, -INTEGRAL_MAX, INTEGRAL_MAX);

  integral_ang  += err_ang * dt;
  integral_ang   = constrain(integral_ang,  -INTEGRAL_MAX, INTEGRAL_MAX);

  // ── Dérivées ─────────────────────────────────
  float derr_dist = (err_dist - err_dist_prec) / dt;
  float derr_ang  = (err_ang  - err_ang_prec)  / dt;
  err_dist_prec = err_dist;
  err_ang_prec  = err_ang;

  // ── Commandes PID ────────────────────────────
  float u_dist = KP_DIST * err_dist + KI_DIST * integral_dist + KD_DIST * derr_dist;
  float u_ang  = KP_ANG  * err_ang  + KI_ANG  * integral_ang  + KD_ANG  * derr_ang;

  // ── Mix gauche / droite ──────────────────────
  float cmd_gauche = u_dist - u_ang;
  float cmd_droite = u_dist + u_ang;

  // ── Rampe d'accélération ─────────────────────
  auto rampe = [](float cible, float actuel) -> float {
    float delta = cible - actuel;
    delta = constrain(delta, -(float)RAMPE_STEP, (float)RAMPE_STEP);
    return actuel + delta;
  };

  pwm_gauche_reel = rampe(cmd_gauche, pwm_gauche_reel);
  pwm_droite_reel = rampe(cmd_droite, pwm_droite_reel);

  // ── Saturation PWM ───────────────────────────
  float pg = constrain(pwm_gauche_reel, -PWM_MAX, PWM_MAX);
  float pd = constrain(pwm_droite_reel, -PWM_MAX, PWM_MAX);

  // ── Envoi aux moteurs ────────────────────────
  moteur_set(1, pg);
  moteur_set(2, pd);

  // ── Détection de fin de mouvement ────────────
  if (fabsf(err_dist) < SEUIL_DIST_MM && fabsf(err_ang) < SEUIL_ANG_MM) {
    moteur_stop();
    integral_dist   = 0.0f;
    integral_ang    = 0.0f;
    pwm_gauche_reel = 0.0f;
    pwm_droite_reel = 0.0f;
    etat = ASSERV_TERMINE;
  }
}
