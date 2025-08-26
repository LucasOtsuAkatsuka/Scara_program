#include <Arduino.h>
#include "Garra.h"

Garra garra;

void setup() {
    Serial.begin(9600);
   garra.begin(pinControlServo_1, pinTRIG, pinECHO);
}

void loop() {
//    garra.abrir(true);
//    delay(1000);
//    garra.fechar(true);
//    delay(1000);
//    garra.subir(true);
//    delay(1000);
//    garra.descer(true);
//    delay(1000);
   Serial.println(garra.distancia());
   delay(1000);
}