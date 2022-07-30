#include <Arduino.h>
#include <ESP32Encoder.h>

#define RADS2RPM 9.5492965964254

// Encoder
#define ENCA 18     // Pin A dell'encoder
#define ENCB 19     // Pin B dell'encoder
#define HALF_QUAD 0 // Flag per decidere se usare half-quad o full-quad
const int pulsesPerRoute = HALF_QUAD ? 100 : 200;       // Numero di impulsi in un giro completo
const float radiansPerPulse = 2*PI / pulsesPerRoute;    // Radianti corrispondenti a 1 impulso
ESP32Encoder Enc;   // Oggetto per gestire l'encoder
long pulseCounter;  // Contatore degli impulsi dell'encoder

// Velocità angolare
float angVel_ps;    // Pulses per second
float angVel_rads;  // Radians per second
float angVel_rpm;   // Routes per minute

// Variabili temporali: printf(%lu)
unsigned long t=0, oldT;
float deltaT;
int period = 200;   // Deve essere abbastanza grande per avere abbastanza campioni

// Dichiarazioni
void getAngularVelocities(float);


// ================ SETUP ==================
void setup() {
    
    Serial.begin(115200);

    // Inizializzazione dell'encoder
    if ( HALF_QUAD ) {
        Enc.attachHalfQuad(ENCA, ENCB);
    }
    else {
        Enc.attachFullQuad(ENCA, ENCB);
    }
}

// ================ LOOP ===================
void loop() {

    // Calcolo l'intervallo di campionamento
    oldT = t;
    t = micros();
    deltaT = float(t - oldT) / 1000000;     // In secondi

    //Serial.printf("deltaT: %.5f s\t%lu us\n", deltaT, (t-oldT));
    getAngularVelocities(deltaT);

    //Serial.printf("%f, %f, %f\n", angVel_ps, angVel_rads, angVel_rpm);
    Serial.printf("%f\n", angVel_rpm);

    delay(period);
}

void getAngularVelocities(float deltaT) {
    
    // Conto quanti pulses ha fatto nel periodo di campionamento
    pulseCounter = Enc.getCount();
    //Serial.printf("%ld\n", pulseCounter);

    // Calcolo le velocità angolari nelle varie unità di misura
    angVel_ps = (float)pulseCounter / deltaT;
    angVel_rads = angVel_ps * radiansPerPulse;
    angVel_rpm = angVel_rads * RADS2RPM;

    // Azzero il contatore
    Enc.clearCount();
}