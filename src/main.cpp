#include <Arduino.h>
#include <ESP32Encoder.h>

#define RADS2RPM 9.5492965964254
#define RPM_GIRARROSTO 1

// Riduzione
//#define GIRARROSTO2ENCODER 19.37    // Girarrosto v1.0 - 500 / 16

/*
    Girarrosto v2.0
    - MotorGear: 75 denti
    - EncoderGear: 15 denti
    - WormWheel: 20 denti

    Riduzione = 75/15 * 20 = 100
*/
#define GIRARROSTO2ENCODER 100    // Girarrosto v2.0 - 

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
const int pwmSat_low = 1000;
const int pwmSat_high = maxDutyCycle;   // A vuoto consuma 1.25A, meglio non andare più su

// Parametri del PID: funziona QB con 0.01 e 0.01
const float kp = 0.01;
const float ki = 0.01;
const float kd = 0;
volatile float target = 0;    // Velocità angolare da raggiungere e mantenere [rpm]
volatile float memTarget = RPM_GIRARROSTO * GIRARROSTO2ENCODER, tempTarget;    // Variabili per accendere/spegnere la rotazione
float error = 0;
float errorP, errorI, errorD;   // Termini per gestire le varie componenti del PID

// Low-Pass filter
float angVel_filt;
#define N 3
float oldVel[N] = {0};

// Secondo encoder, gestito con interrupt
#define ENC2A 27
#define ENC2B 26
#define ENC2SWITCH 25           // Quando premo sulla manopola
#define KNOB_PULSES_PER_ROUTE 30    // Numero di impulsi per giro
ESP32Encoder knob;              // Oggetto per gestire la manopola
void IRAM_ATTR changeTarget();  // Modifica la velocità target
volatile bool aState, bState;   // Variabili per determinare la direzione
void IRAM_ATTR startStop();     // Accende/spegne la rotazione
volatile bool rotate = true;    // indica se il motore deve girare 
volatile unsigned long interruptT = 0;  // Timer per gli interrupt


// Dichiarazioni
void getAngularVelocities(float);
void setDirection(const int);
void setDutyCycle(int);
void lowPassFilter(float);
void changeSpeed();


// ================ SETUP ==================
void setup() {
    
    Serial.begin(115200);

    // Inizializzazione Encoder di controllo
    pinMode(ENC2A, INPUT);
    pinMode(ENC2B, INPUT);
    pinMode(ENC2SWITCH, INPUT);
    Serial.printf("# Controller: CLK: %d, DT: %d, SW: %d\n", ENC2A, ENC2B, ENC2SWITCH);
    //attachInterrupt(ENC2A, changeTarget, CHANGE);
    attachInterrupt(ENC2SWITCH, startStop, FALLING);
    aState = digitalRead(ENC2A);
    bState = digitalRead(ENC2B);

    // Inizializzazione ledc channel
    ledcSetup(CHANNEL, FREQUENCY, RESOLUTION);
    ledcAttachPin(EN, CHANNEL);
    Serial.printf("# PWM motor signal: EN: %d\n", EN);

    // Inizializzazione pin del motore per la direzione
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    setDirection(CW);
    Serial.printf("# Motor direction pins: IN3: %d, IN4: %d\n", IN1, IN2);

    // Inizializzazione dell'encoder
    if ( HALF_QUAD ) {
        Enc.attachHalfQuad(ENCA, ENCB);
    }
    else {
        Enc.attachFullQuad(ENCA, ENCB);
    }
    Serial.printf("# Encoder: pinA: %d, pinB: %d", ENCA, ENCB);

    // Inizializzazione della manopola
    knob.attachHalfQuad(ENC2A, ENC2B);
}

// ================ LOOP ===================
void loop() {

    // Se non c'è il segnale di partenza
    if (!rotate) {
        Serial.printf("#Resting\n");
        return;
    }

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
        if ( dutyCycle < pwmSat_high ) {    // Anti wind-up
            errorI += error * deltaT;
        }

        // Sommo le componenti 
        dutyCycle = (kp*errorP + ki*errorI + kd*errorD) * maxDutyCycle;


    // Imposto il dutycycle
    if (dutyCycle > pwmSat_high) dutyCycle = pwmSat_high;
    if (dutyCycle < pwmSat_low) dutyCycle = pwmSat_low;
    setDutyCycle(dutyCycle);
    //setDutyCycle( 1400 );

    //Serial.printf("%f, %f, %f\n", angVel_ps, angVel_rads, angVel_rpm);
    dutyCycle_print = dutyCycle / maxDutyCycle * 100;;
    Serial.printf("%.2f, %.2f, %.2f\n", target, angVel_rpm, angVel_filt);
    //Serial.printf("%.2f\n", error);

    changeSpeed();

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
    
    // Il controllo che sia entro i limiti viene fatto fuori

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
        sum += oldVel[i];
    }
    angVel_filt = count? sum/count : 0;
    oldVel[0] = angVel_filt;
}

void IRAM_ATTR startStop() {
    if( millis() - interruptT > 500) {
        rotate = !rotate;
        setDutyCycle(0);
        tempTarget = target;
        target = memTarget;
        memTarget = tempTarget;
        //setDutyCycle(0);
        interruptT = millis();
    }
}

void IRAM_ATTR changeTarget() {
    if ( millis() - interruptT < 100 ) return;

    aState = digitalRead(ENC2A);
    bState = digitalRead(ENC2B);
    if ( aState == bState ) {
        target *= 0.7;
    }
    else {
        target *= 1.4;
    }

    if ( target > 120 ) target = 120;

    interruptT = millis();
}

void changeSpeed() {
    // Conto gli impulsi
    int knobPulses = knob.getCount();
    if (knobPulses==0) return;
    //Serial.printf("%d\n", knobPulses);

    // Modifico la velocità target
    float multiplier = pow(0.9, knobPulses);
    target *= multiplier;

    if (target>120) target = 120;

    // Azzero il contatore
    knob.clearCount();
}