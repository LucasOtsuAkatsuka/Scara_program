#ifndef GARRA_H
#define GARRA_H

#include <Arduino.h>
#include "variaveis.h"
#include <Servo.h>

class Garra {
public:
    void begin();
    void abrir();
    void fechar();
    void subir();
    void descer();
    void distancia();

private:
    Servo servo;
};

#endif