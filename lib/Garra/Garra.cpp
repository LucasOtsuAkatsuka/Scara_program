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
   }
}
void Garra::fechar(bool enable)
{
   if (enable) {
       servo.write(angleClose);
   }
}
void Garra::subir(bool enable)
{
   if (enable) {
       servo.write(angleUp);
   }
}

void Garra::descer(bool enable)
{
   if (enable) {
       servo.write(angleDown);
   }
}

float Garra::distancia()
{
   _distancia = ultrasonic.read();
   return _distancia;
}