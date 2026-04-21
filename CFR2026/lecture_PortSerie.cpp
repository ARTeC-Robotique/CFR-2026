#include "lecture_PortSerie.h"

Point p = {"", 0, 0};

static char buffer[48];  // un peu plus grand pour accueillir la string
static uint8_t idx = 0;

bool nouveau_message = false;

static void traiter_message() {
  char com[16];
  int x, y;
  if (sscanf(buffer, "%15s %d %d", com, &x, &y) == 3) {
    strncpy(p.com, com, sizeof(p.com) - 1);
    p.com[sizeof(p.com) - 1] = '\0';
    p.x = (int16_t)x;
    p.y = (int16_t)y;
    nouveau_message = true;   // <--
  } else {
    Serial.println("Format invalide. Exemple : avance 100 200");
  }
}

void portSerie_init() {
  Serial.setTimeout(2);
  Serial.println("Pret. Format : 'com x y\\n'  ex: avance 100 200");
}

void portSerie_lecture() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (idx > 0) {
        buffer[idx] = '\0';
        traiter_message();
        idx = 0;
      }
    } else {
      if (idx < sizeof(buffer) - 1) {
        buffer[idx++] = c;
      } else {
        idx = 0;
        Serial.println("Erreur : message trop long");
      }
    }
  }
}