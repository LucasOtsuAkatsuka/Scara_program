#ifndef RECALIBRACAO_H
#define RECALIBRACAO_H

#define pinSw_1 A0
#define pinSw_2 A1

#include "Cinematica.h"
#include <Arduino.h>
class Recalibracao {
public:
    void begin();
    bool recalibrar(Cinematica* braco);
private:
    bool FirstRun = false;
    bool braco_1_recalibrado = false;
    bool braco_2_recalibrado = false;
};
#endif
