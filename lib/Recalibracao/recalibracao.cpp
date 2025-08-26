#include "recalibracao.h"

void Recalibracao::begin() {
    pinMode(pinSw_1, INPUT);
    pinMode(pinSw_2, INPUT);
}
bool Recalibracao::recalibrar(Cinematica* braco) {
    if(FirstRun)
    {
        braco->goToXY(0, 0);
        FirstRun = false;
    }
    if(!digitalRead(pinSw_1))
    {
        braco->moveSteps(1, -8);
        braco_1_recalibrado = false;
    }
    else
    {
        braco_1_recalibrado = true;
    }
    if(digitalRead(pinSw_2))
    {
        braco->moveSteps(1, -8);
        braco_2_recalibrado = false;
    }
    else
    {
        braco_2_recalibrado = true;
    }
    if(braco_1_recalibrado && braco_2_recalibrado)
    {
        braco->setOrigin();
        FirstRun = true;
        return true;
    }
    return false;
}