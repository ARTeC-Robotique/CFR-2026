#pragma once
#include <Arduino.h>

struct Point {
  char    com[16]; //ca c'est la commandes
  int16_t para1;
  int16_t para2;
};

extern Point p;
extern bool  nouveau_message;  // true quand un message vient d'arriver

void portSerie_init();
void portSerie_lecture();