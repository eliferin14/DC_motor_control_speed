#include <Arduino.h>

#define ENCA 18
#define ENCB 19

class RotaryEncoder {
    public:
        // Variabili per i valori attuali dei pin
        int a=0, b=0;

        // Variabili per i valori vecchi dei pin
        int prevA=0, prevB=0;

        // Contatori
        int countA=0, countB=0;

        // Variabile per la velocità
        float ang_velocity;

        // Costruttore
        RotaryEncoder();

    private:
        // Funzioni per gli interrupt
        //void IRAM_ATTR interruptA();
        //void IRAM_ATTR interruptB();
};

RotaryEncoder RE;

// Funzioni per gli interrupt
void IRAM_ATTR interruptA();
void IRAM_ATTR interruptB();