#pragma once
#include <Arduino.h>

struct Point {
  char    com[16]; //ca c'est la commandes
  int16_t para1;
  int16_t para2;
  int16_t para3; };

extern bool  nouveau_message;

void portSerie_init();
void portSerie_lecture();