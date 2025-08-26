#include <Arduino.h>
#include "Garra.h"

Garra garra;

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
