#include <Arduino.h>
#include "Garra.h"

Garra garra;

void setup() {
 garra.begin();
}

void loop() {
garra.abrir();
delay(1000);
garra.fechar();
delay(1000);
garra.subir();
delay(1000);
garra.descer();
}
