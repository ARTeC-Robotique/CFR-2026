// odometrie.cpp
// Odométrie par intégration des encodeurs incrémentaux
//
// Hypothèses :
//   - Les encodeurs renvoient des comptes bruts (ticks, pas des mm)
//     → on lit count1 / count2 directement depuis encodeurs.h
//   - La conversion ticks→mm est faite ici pour travailler en float
//   - Le repère est : x vers l'avant, y vers la gauche
//     θ=0 = face au "nord", croît dans le sens trigo (anti-horaire)

#include "odometrie.h"
#include "encodeurs.h"   // pour count1, count2 (volatile long)
#include <math.h>

// ──────────────────────────────────────────────
//  Paramètres géométriques  (à ajuster)
// ──────────────────────────────────────────────

// Diamètre des roues en mm (mesurer au pied à coulisse)
#define DIAMETRE_ROUE_MM    50.0f

// Résolution encodeur : nombre de ticks par tour de roue
// (avec la quadrature sur un seul canal : 2000 flancs/tour = 2000 ticks)
#define TICKS_PAR_TOUR      2000.0f

// Voie : distance entre les deux points de contact sol, en mm
// À mesurer physiquement — c'est le paramètre le plus critique
#define VOIE_MM             150.0f

// Dérivé : distance parcourue par tick
static const float MM_PAR_TICK = (DIAMETRE_ROUE_MM * 3.14159265f) / TICKS_PAR_TOUR;

// ──────────────────────────────────────────────
//  État interne
// ──────────────────────────────────────────────

static Pose  pose_courante = {0.0f, 0.0f, 0.0f};
static long  prev_countG   = 0;
static long  prev_countD   = 0;

// ──────────────────────────────────────────────
//  Lecture des comptes bruts (thread-safe)
// ──────────────────────────────────────────────
// IMPORTANT : count1 = roue gauche, count2 = roue droite
//             (vérifier la correspondance avec votre câblage)
static void lire_counts_bruts(long &cG, long &cD) {
  noInterrupts();
  cG = count1;   // encodeur gauche
  cD = count2;   // encodeur droit
  interrupts();
}

// ──────────────────────────────────────────────
//  API publique
// ──────────────────────────────────────────────

void odometrie_init() {
  lire_counts_bruts(prev_countG, prev_countD);
  pose_courante = {0.0f, 0.0f, 0.0f};
}

void odometrie_reset(float x, float y, float theta) {
  // On mémorise la position courante des encodeurs comme référence
  lire_counts_bruts(prev_countG, prev_countD);
  pose_courante = {x, y, theta};
}

bool odometrie_update() {
  // ── 1. Lecture des comptes actuels ──────────
  long cur_cG, cur_cD;
  lire_counts_bruts(cur_cG, cur_cD);

  // ── 2. Incréments en ticks ──────────────────
  long dG_ticks = cur_cG - prev_countG;
  long dD_ticks = cur_cD - prev_countD;

  if (dG_ticks == 0 && dD_ticks == 0) return false;  // rien à faire

  prev_countG = cur_cG;
  prev_countD = cur_cD;

  // ── 3. Incréments en mm ─────────────────────
  float dG = dG_ticks * MM_PAR_TICK;
  float dD = dD_ticks * MM_PAR_TICK;

  // ── 4. Modèle cinématique différentiel ──────
  //
  //   d  = déplacement du centre (mm)
  //   dθ = variation d'angle (rad)
  //
  float d  = (dD + dG) * 0.5f;
  float dtheta = (dD - dG) / VOIE_MM;

  // ── 5. Intégration de la pose ────────────────
  //
  // On utilise l'angle moyen (θ + dθ/2) pour une meilleure précision
  // sur les arcs de cercle (méthode de Runge-Kutta ordre 2)
  //
  float theta_moy = pose_courante.theta + dtheta * 0.5f;

  pose_courante.x     += d * cosf(theta_moy);
  pose_courante.y     += d * sinf(theta_moy);
  pose_courante.theta += dtheta;

  // ── 6. Normalisation de l'angle [-π, +π] ────
  while (pose_courante.theta >  M_PI) pose_courante.theta -= 2.0f * M_PI;
  while (pose_courante.theta < -M_PI) pose_courante.theta += 2.0f * M_PI;

  return true;
}

Pose odometrie_lire() {
  return pose_courante;
}

void odometrie_afficher() {
  Pose p = pose_courante;
  Serial.print("X=");
  Serial.print(p.x, 1);
  Serial.print("mm  Y=");
  Serial.print(p.y, 1);
  Serial.print("mm  θ=");
  Serial.print(degrees(p.theta), 1);
  Serial.println("°");
}
