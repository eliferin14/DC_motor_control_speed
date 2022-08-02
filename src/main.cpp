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
float angVel_rpm = 0;   // Routes per minute
float oldVel_rpm;   // Campione precedente

// Variabili temporali: printf(%lu)
unsigned long t=0, oldT;
float deltaT;
int period = 200;   // Deve essere abbastanza grande per avere abbastanza campioni

// Motore con driver L298n
#define EN 13       // Pin su cui mandare il segnale pwm
#define IN1 12      // Pin per la direzione
#define IN2 14      // Pin per la direzione
#define CW 1        // Senso orario
#define CCW 0       // Senso antiorario

// Parametri per PWM : https://randomnerdtutorials.com/esp32-pwm-arduino-ide/
#define CHANNEL 0       // Canale per la funzione ledc
#define FREQUENCY 20000  // Frequenza dell'onda quadra
#define RESOLUTION 11    // Bit di risoluzione del dutycycle => livelli = 2^RESOLUTION
int dutyCycle = 0;
int dutyCycle_print;
const int maxDutyCycle = pow(2, RESOLUTION) -1;

// Parametri del PID: funziona QB con 0.01 e 0.01
const float kp = 0.01;
const float ki = 0.01;
const float kd = 0;
float target = 24;    // Velocità angolare da raggiungere e mantenere [rpm]
float error = 0;
float errorP, errorI, errorD;   // Termini per gestire le varie componenti del PID

// Low-Pass filter
float angVel_filt;
#define N 4
float oldVel[N] = {0};

// Secondo encoder, gestito con interrupt
#define ENC2A 26
#define ENC2B 27
#define ENC2SWITCH 25
void IRAM_ATTR changeTarget();
void IRAM_ATTR startStop();
volatile bool rotate = false;   // indica se il motore deve girare 


// Dichiarazioni
void getAngularVelocities(float);
void setDirection(const int);
void setDutyCycle(int);
void lowPassFilter(float);


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

    // Memorizzo il campione precedente della velocità
    oldVel_rpm = angVel_rpm;

    // Misuro la velocità
    getAngularVelocities(deltaT);

    // Filtro
    lowPassFilter(angVel_rpm);

    // Calcolo l'errore
    error = target - angVel_filt;
    // Calcolo il dutyCycle usando i parametri del PID
    // NB: Ignoro il termine integrale

        // Termine proporzionale
        errorP = error;

        // Termine integrale: valore*deltaT
        errorI += error * deltaT;

        // Sommo le componenti 
        dutyCycle = (kp*errorP + ki*errorI + kd*errorD) * maxDutyCycle;


    // Imposto il dutycycle
    setDutyCycle(dutyCycle);

    //Serial.printf("%f, %f, %f\n", angVel_ps, angVel_rads, angVel_rpm);
    dutyCycle_print = map(dutyCycle, -maxDutyCycle, maxDutyCycle, -target, target);
    Serial.printf("%.2f, %.2f\n", target, angVel_filt);

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

void lowPassFilter(float newVel) {
    // Inserisco il nuovo valore nell'array
    for( int i=N-1; i>0; i--) {
        //printf("%.2f ---> ", oldVel[i]);
        oldVel[i] = oldVel[i-1];
        //printf("%.2f\n", oldVel[i]);
    }
    oldVel[0] = newVel;

    /*
    for(int i=0; i<N; i++) {
        Serial.printf("%.2f ", oldVel[i]);
    }
    Serial.printf("\n");
    */

    // Calcolo il valore filtrato
    float sum = 0;
    int count = N;
    for( int i=0; i<N; i++) {
        if (oldVel[i] != 0) {
            sum += oldVel[i];
        }
        else {
            count--;
        }
    }
    angVel_filt = count? sum/count : 0;
}