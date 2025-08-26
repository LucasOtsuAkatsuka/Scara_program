#include <Arduino.h>
#include "Garra.h"

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

void setup() {
 Serial.begin(9600);
 garra.begin();
}

void loop() {
Serial.println(garra.distancia());
if(garra.distancia() > 10.0) {
   digitalWrite(LED_BUILTIN, HIGH);
} else {
   digitalWrite(LED_BUILTIN, LOW);
}

delay(1000);

}
