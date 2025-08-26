#include <Arduino.h>
#include "Garra.h"
#include "Cinematica.h"

Garra garra;

#define DEF_DIR11  20
#define DEF_STEP11 21
#define DEF_DIR12  18
#define DEF_STEP12 19
#define DEF_EN    8
#define DEF_DIR21  16
#define DEF_STEP21 17
#define DEF_DIR22  14
#define DEF_STEP22 15

Cinematica braco1(DEF_STEP11, DEF_DIR11, DEF_STEP12, DEF_DIR12, DEF_EN);

void setup() {
   Serial.begin(9600);
   garra.begin(pinControlServo_1, pinTRIG, pinECHO);
   braco1.begin(true);
   
   braco1.goToIndex(4);
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