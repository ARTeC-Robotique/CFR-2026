// odometrie.h
// Localisation du robot par intégration des encodeurs (odométrie)
// Repère : x vers l'avant, y vers la gauche, θ en radians (trigonométrique)

#pragma once
#include <Arduino.h>

struct Pose {
  float x;      // position en mm
  float y;      // position en mm
  float theta;  // orientation en radians
};

// Initialise l'odométrie (pose = origine)
void odometrie_init();

// Remet la pose à zéro (ou à une valeur connue)
void odometrie_reset(float x = 0.0f, float y = 0.0f, float theta = 0.0f);

// À appeler dans loop() : intègre les encodeurs et met à jour la pose
// Retourne true si la pose a changé
bool odometrie_update();

// Lecture de la pose courante
Pose odometrie_lire();

// Affichage sur le port série (debug)
void odometrie_afficher();
