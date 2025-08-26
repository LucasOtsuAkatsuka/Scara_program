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
    void begin(int pinControl, int pinTrig, int pinEcho);
    void abrir(bool enable);
    void fechar(bool enable);
    void subir(bool enable);
    void descer(bool enable);
    float distancia();

private:
    Servo servo;
    float _distancia;
    long microseg;
    Ultrasonic ultrasonic = Ultrasonic(0, 0);
};

#endif