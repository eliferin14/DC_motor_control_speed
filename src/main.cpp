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

// Motore con driver L298n
#define EN 21       // Pin su cui mandare il segnale pwm
#define IN1 23      // Pin per la direzione
#define IN2 22      // Pin per la direzione
#define CW 1        // Senso orario
#define CCW 0       // Senso antiorario

// Parametri per PWM : https://randomnerdtutorials.com/esp32-pwm-arduino-ide/
#define CHANNEL 0       // Canale per la funzione ledc
#define FREQUENCY 5000  // Frequenza dell'onda quadra
#define RESOLUTION 8    // Bit di risoluzione del dutycycle => livelli = 2^RESOLUTION
int dutyCycle;
const int maxDutyCycle = pow(2, RESOLUTION) -1;


// Dichiarazioni
void getAngularVelocities(float);
void setDirection(const int);
void setDutyCycle(int);


// ================ SETUP ==================
void setup() {
    
    Serial.begin(115200);

    // Inizializzazione ledc channel
    ledcSetup(CHANNEL, FREQUENCY, RESOLUTION);
    ledcAttachPin(EN, CHANNEL);

    // Inizializzazione pin del motore per la direzione
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    setDirection(CW);

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

    // Mirror
    dutyCycle = 200 + angVel_rpm;
    setDutyCycle(dutyCycle);

    //Serial.printf("%f, %f, %f\n", angVel_ps, angVel_rads, angVel_rpm);
    Serial.printf("%f, %d\n", angVel_rpm, dutyCycle);

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

void setDirection(const int newDir) {
    if (newDir == CW) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    }
    else if (newDir == CCW) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    }
    else {
        perror("Direzione indicata non valida");
        exit(1);
    }
}

void setDutyCycle(int dt) {
    // Accetto anche un argomento negativo: il segno indica la direzione
    if (dt < 0) {
        setDirection(CCW);
        dt *= -1;
    }
    else {
        setDirection(CW);
    }

    // Controllo che il valore passato sia entro i limiti
    if (dt > maxDutyCycle) {
        dt = maxDutyCycle;
    }

    // Imposto il dutycycle
    ledcWrite(CHANNEL, dt);
}