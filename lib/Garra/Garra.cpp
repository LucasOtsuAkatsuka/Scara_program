#include "Garra.h"

void Garra::begin(int pinControl, int pinTrig, int pinEcho) {
   servo.attach(pinControl);
   servo.write(0);
   ultrasonic = Ultrasonic(pinTrig, pinEcho);
}

void Garra::abrir(bool enable)
{
   if (enable) {
       servo.write(angleOpen);
       Serial.println("abrir garra");
   }
}
void Garra::fechar(bool enable)
{
   if (enable) {
       servo.write(angleClose);
       Serial.println("fechar garra");
   }
}
void Garra::subir(bool enable)
{
   if (enable) {
       servo.write(angleUp);
       Serial.println("subir garra");
   }
}

void Garra::descer(bool enable)
{
   if (enable) {
       servo.write(angleDown);
       Serial.println("descer garra");
   }
}

float Garra::distancia()
{
   _distancia = ultrasonic.read();
   return _distancia;
}