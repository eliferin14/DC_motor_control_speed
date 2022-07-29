#include <rotary_encoder.h>

// Costruttore
RotaryEncoder::RotaryEncoder() {
    // Inizializzo i pin come INPUT
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);

    // Associo gli interrupt alle funzioni
    // https://lastminuteengineers.com/handling-esp32-gpio-interrupts-tutorial/
    attachInterrupt( digitalPinToInterrupt(ENCA), interruptA, CHANGE );
}

void IRAM_ATTR interruptA() {
    RE.countA++;
}