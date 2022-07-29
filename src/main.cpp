#include <Arduino.h>
#include <rotary_encoder.h>

#define ENCA 18
#define ENCB 19

int a, b, olda, oldb;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);

    olda = oldb = 0;
}

void loop() {
    // put your main code here, to run repeatedly:
    a = digitalRead(ENCA);
    b = digitalRead(ENCB);

    if (olda!=a || oldb!=b) {
        Serial.printf("%d,%d\n", a, b);
        olda = a;
        oldb = b;
    }
}