#include "lecture_PortSerie.h"

Point p = {"", 0, 0, 0};

static char buffer[48];
static uint8_t idx = 0;

void portSerie_init() {
  Serial.println("Port Série pret ! Format : 'fonction parametre1 paramtre2 ...\\n' ex: avance 100 0");
}

void traiter_message() {
  char* token = strtok(buffer, " ");
  if (token == NULL) return;
  char com[16];
  strncpy(p.com, token, sizeof(p.com) - 1);
  p.com[sizeof(p.com) - 1] = '\0';
  token = strtok(NULL," ");
  int para1 = token ? atoi(token) : 0;
  token = strtok(NULL," ");
  int para2 = token ? atoi(token) : 0;
  token = strtok(NULL," ");
  int para3 = token ? atoi(token) : 0;
  p.para1 = (int16_t)para1;
  p.para2 = (int16_t)para2;
  p.para3 = (int16_t)para3;
  nouveau_message = true;
}

void portSerie_lecture() {
  while (Serial.available() > 0) {

    char c = Serial.read();

    if (idx >= sizeof(buffer) - 1) {
      idx = 0;
      Serial.println("Erreur : message trop long");
      return;
    }

    if (c != '\n' && c != '\r') {
      buffer[idx++] = c;
    }

    else {
      if (idx > 0) {
        buffer[idx] = '\0';   
        traiter_message();    
        idx = 0;              
      }
    }
  }
}