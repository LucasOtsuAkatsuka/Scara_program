#include "recalibracao.h"

void Recalibracao::begin() {
    pinMode(pinSw_1, INPUT_PULLUP);
    //pinMode(pinSw_2, INPUT_PULLUP);
}

bool Recalibracao::recalibrar(Cinematica* braco) {
    Serial.println("iniciando recalibracao");
    braco->goToXY(445, 0);

    if (digitalRead(pinSw_1) == LOW)
    {
        braco_1_recalibrado = true;
        Serial.println("braco 1 recalibrado");
        braco_2_recalibrado = true;
        Serial.println("braco 2 recalibrado");
    }
    
    while (!(braco_1_recalibrado && braco_2_recalibrado))
    {
        braco->moveSteps(1, -8);
        braco->moveSteps(2, -8);
        if(digitalRead(pinSw_1) == LOW)
        {
            braco_1_recalibrado = true;
            braco_2_recalibrado = true;
            Serial.println("braco 1 recalibrado");
            Serial.println("braco 2 recalibrado");
        }
        
        if(digitalRead(pinSw_1) == LOW){
            Serial.println("braco 2 recalibrado");
            braco_2_recalibrado = true;
            Serial.println("recalibrando...");
        }
        Serial.println("recalibrando...");
    }
    

    braco->setOrigin();
    Serial.println("recalibracao concluida");
    return true;

}