#ifndef GARRA_H
#define GARRA_H

#define pinECHO 11
#define pinTRIG 13
#define pinControlServo_1 5
#define angleOpen 90
#define angleClose 0
#define angleUp 180
#define angleDown 45

#include <Arduino.h>
#include <Servo.h>
#include <Ultrasonic.h>

class Garra {
public:
    void begin();
    void abrir();
    void fechar();
    void subir();
    void descer();
    float distancia();

private:
    Servo servo;
    Ultrasonic ultrasonic = Ultrasonic(pinTRIG, pinECHO);
    float _distancia;
    long microseg;
};

#endif