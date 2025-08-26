#include "Garra.h"

void Garra::begin() {
   servo.attach(pinControlServo_1);
   servo.write(0);
}

void Garra::abrir()
{
   servo.write(angleOpen);
}
void Garra::fechar()
{
   servo.write(angleClose);
}
void Garra::subir()
{
   servo.write(angleUp);
}

void Garra::descer()
{
   servo.write(angleDown);
}

float Garra::distancia()
{
   microseg = ultrasonic.timing();
   distancia = ultrasonic.convert(microseg, Ultrasonic::CM);
   return distancia;
}