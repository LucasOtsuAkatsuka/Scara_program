#include <Arduino.h>
#include "Garra.h"
#include "Cinematica.h"
#include "recalibracao.h"



#define DEF_DIR11  20
#define DEF_STEP11 21
#define DEF_DIR12  18
#define DEF_STEP12 19
#define DEF_EN    8
#define DEF_DIR21  16
#define DEF_STEP21 17
#define DEF_DIR22  14
#define DEF_STEP22 15
#define pinControlServo_1 5
//#define pinControlServo_2 6
#define botao1 2
#define botao2 3
#define botao3 4
#define botao4 5
#define botao5 6
#define botao6 7

Garra garra1;
Garra garra2;
Cinematica braco1(DEF_STEP11, DEF_DIR11, DEF_STEP12, DEF_DIR12, DEF_EN);
Cinematica braco2(DEF_STEP21, DEF_DIR21, DEF_STEP22, DEF_DIR22, DEF_EN);
Recalibracao recalibracao;

void setup() {
   Serial.begin(9600);
   garra1.begin(pinControlServo_1, pinTRIG, pinECHO);
   //garra2.begin(pinControlServo_2, pinTRIG, pinECHO); 
   recalibracao.begin();
   braco1.begin(true);

   Serial.println("#####################");
   Serial.println("Braco Scara Iniciado!");
   Serial.println("#####################");

   delay(2000);

   recalibracao.recalibrar(&braco1);


   pinMode(botao1, INPUT_PULLUP);
   pinMode(botao2, INPUT_PULLUP);
   pinMode(botao3, INPUT_PULLUP);
   pinMode(botao4, INPUT_PULLUP);
   pinMode(botao5, INPUT_PULLUP);
   pinMode(botao6, INPUT_PULLUP);
   
}

void loop() {
   
   int botaoPressionado;

   botaoPressionado = 0;

   if (digitalRead(botao1) == LOW) botaoPressionado = 1;
   else if (digitalRead(botao2) == LOW) botaoPressionado = 2;
   else if (digitalRead(botao3) == LOW) botaoPressionado = 3;
   else if (digitalRead(botao4) == LOW) botaoPressionado = 4;
   else if (digitalRead(botao5) == LOW) botaoPressionado = 5;
   else if (digitalRead(botao6) == LOW) botaoPressionado = 6;

   switch (botaoPressionado) {
       case 1:
           Serial.println("botao 1 apertado");
           braco1.goToIndex(1);
           break;
       case 2:
           Serial.println("botao 2 apertado");
           braco1.goToIndex(2);           
           break;
       case 3:
           Serial.println("botao 3 apertado");  
           braco1.goToIndex(3);
           break;
       case 4:
           Serial.println("botao 4 apertado");
           recalibracao.recalibrar(&braco1);
           break;
       case 5:
           Serial.println("botao 5 apertado");
           
           break;
       case 6:
           Serial.println("botao 6 apertado");
           garra1.fechar(true);
           break;
       default:
           Serial.println("...");
   }

   delay(500);

   //int braco1_comando;
   //int braco2_comando;
   //int garra1_comando;
   //int garra2_comando;
//
   //switch (braco1_comando) {
   //    case 1:
   //        Serial.println("botao 1 apertado");
   //        braco1.goToIndex(1);
   //        break;
   //    case 2:
   //        Serial.println("botao 2 apertado");
   //        braco1.goToIndex(2);           
   //        break;
   //    case 3:
   //        Serial.println("botao 3 apertado");  
   //        braco1.goToIndex(3);
   //        break;
   //    case 4:
   //        Serial.println("botao 4 apertado");  
   //        braco1.goToIndex(4);
   //        break;
   //    case 5:
   //        Serial.println("botao 5 apertado");  
   //        braco1.goToIndex(5);
   //        break;
   //    case 6:
   //        Serial.println("botao 6 apertado");
   //        recalibracao.recalibrar(&braco1);
   //        break;
   //    default:
   //        Serial.println("...");
   //}
//
   //switch (braco2_comando) {
   //    case 1:
   //        Serial.println("botao 1 apertado");
   //        braco2.goToIndex(1);
   //        break;
   //    case 2:
   //        Serial.println("botao 2 apertado");
   //        braco2.goToIndex(2);           
   //        break;
   //    case 3:
   //        Serial.println("botao 3 apertado");
   //        braco2.goToIndex(3);
   //        break;
   //    case 4:
   //        Serial.println("botao 4 apertado");  
   //        braco2.goToIndex(4);
   //        break;
   //    case 5:
   //        Serial.println("botao 5 apertado");
   //        braco2.goToIndex(5);
   //        break;
   //    case 6:
   //        Serial.println("botao 6 apertado");
   //        recalibracao.recalibrar(&braco2);
   //        break;
   //    default:
   //        Serial.println("...");
   //}
//
   //switch (garra1_comando) {
   //    case 1:
   //        Serial.println("botao 1 apertado");
   //        garra1.abrir(true);
   //        break;
   //    case 2:
   //        Serial.println("botao 2 apertado");
   //        garra1.fechar(true);  
   //        break;
   //    case 3:
   //        Serial.println("botao 3 apertado");  
   //        garra1.descer(true);
   //        break;
   //    case 4:
   //        Serial.println("botao 4 apertado");  
   //        garra1.subir(true);
   //        break;
   //    default:
   //        Serial.println("...");
   //}
//
   //switch (garra2_comando) {
   //    case 1:
   //        Serial.println("botao 1 apertado");
   //        garra2.abrir(true);
   //        break;
   //    case 2:
   //        Serial.println("botao 2 apertado");
   //        garra2.fechar(true);  
   //        break;
   //    case 3:
   //        Serial.println("botao 3 apertado");  
   //        garra2.descer(true);
   //        break;
   //    case 4:
   //        Serial.println("botao 4 apertado");  
   //        garra2.subir(true);
   //        break;
   //    default:
   //        Serial.println("...");
   //}


}