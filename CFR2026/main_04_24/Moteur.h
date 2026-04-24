#ifndef MOTEUR_H
#define MOTEUR_H

// Initialisation des moteurs
void moteur_init();

// Commande individuelle d'un moteur (1 = gauche, 2 = droite)
void moteur_set(int mot, float vitesse);

// Arrêt complet
void moteur_stop();

#endif