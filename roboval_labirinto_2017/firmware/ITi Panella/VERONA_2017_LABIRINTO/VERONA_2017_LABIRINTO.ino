/*
   @scuola: ITT. PANELLA - VALLAURI | REGGIO CALABRIA (RC)
   @autori: Pellegrino,Giciu,Boufakri,Nato,Barillà
   Squadra: Elettronica Panella
   Robot:   Roccoval
   @data:   20-21 Maggio 2017
   @evento: Roboval 2017

*/


#include <QTRSensors.h>

// PIN-OUT MOTORI
// A: MOTORE SINISTRO
// B: MOTORE DESTRO
#define MOTOR_A_IN1 4
#define MOTOR_A_PWM 5 //A motore sinistro
#define MOTOR_A_IN2 6
#define MOTOR_STBY  7
#define MOTOR_B_IN1 8
#define MOTOR_B_IN2 9
#define MOTOR_B_PWM 10 //B motore destro

// IDENTIFICAZIONE MOTORI
#define MOTOR_LEFT  0
#define MOTOR_RIGHT 1

//SENSORI
#define QTRLFar          0
#define QTRLNear         1
#define QTRLCenter       2
#define QTRRCenter       3
#define QTRRNear         4
#define QTRRFar          5

// IMPOSTAZIONI QTRSensors
#define QTR_NUM_SENSORS   6
#define QTR_TIMEOUT       2500
#define QTR_EMITTER_PIN   QTR_NO_EMITTER_PIN

//IMPOSTAZIONI
#define Nero                800 
#define Velocita_Rotazione  49
#define Velocita            90


// VARIABILI GLOBALI
int LastDebugTime = 0;
int Errore, Errore_Precedente = 0;
int M1_Velocita, M2_Velocita;
int Stato = 0;
int Velocita_Motori;
unsigned int position;

// CHIAMO LA LIBRERIA QTRSensorsRC
QTRSensorsRC QTRRC((unsigned char[]) {A0, A1, A2, A3, A4, A5}, QTR_NUM_SENSORS, QTR_TIMEOUT, QTR_EMITTER_PIN);
// VALORE DI TUTTI I SENSORI
unsigned int QTRSensorsValue[QTR_NUM_SENSORS];


void setup() {
  // SERIAL BEGIN (PER DEBUG)
  Serial.begin(9600);

  // INIZIALIZZAZIONE MOTORI
  Inizializzazione_Motori ();

  //CALIBRAZIONE SENSORI
  Serial.println("calib");
  for (int i = 0; i < 200; i++) {
    QTRRC.calibrate();

    //ATTIVAZIONE MOTORI
    Motor_standby(false);

  }
}

void loop() {

  switch (Stato) {
    case 0:
      position = QTRRC.readLine(QTRSensorsValue);
      Errore = position - 2500;
      Velocita_Motori = 0.1 * Errore + 5 * (Errore - Errore_Precedente);
      Errore_Precedente = Errore;
      M1_Velocita = Velocita + Velocita_Motori;
      M2_Velocita = Velocita - Velocita_Motori;
      if (M1_Velocita < 10)
        M1_Velocita = 10;
      if (M2_Velocita < 10)
        M2_Velocita = 10;
      if (M1_Velocita > 100)
        M1_Velocita = 80;
      if (M2_Velocita > 100)
        M2_Velocita = 80;
      Set_motor(MOTOR_LEFT , M1_Velocita);
      Set_motor(MOTOR_RIGHT, M2_Velocita);

      QTRRC.readLine(QTRSensorsValue);
      
      
     if (QTRSensorsValue[QTRLFar] > Nero && QTRSensorsValue[QTRLCenter] > Nero  ) {
        Stato = 3;    //ANGOLO A SINISTRA
        Motor_brake(MOTOR_LEFT );
        Motor_brake(MOTOR_RIGHT);
        break;
      }

      if (QTRSensorsValue[QTRLFar] < 200 && QTRSensorsValue[QTRLNear] < 200 
       && QTRSensorsValue[QTRLCenter] < 200 && QTRSensorsValue[QTRRFar] < 200 
       && QTRSensorsValue[QTRRNear] < 200 && QTRSensorsValue[QTRRCenter] < 200) {
        Stato = 2;    //FINE LINEA
        Motor_brake(MOTOR_LEFT );
        Motor_brake(MOTOR_RIGHT);
        break;
      }

      break;


    case 1: //ANGOLO A DESTRA
      Set_motor(MOTOR_LEFT , -Velocita_Rotazione);
      Set_motor(MOTOR_RIGHT, Velocita_Rotazione );
      while (QTRSensorsValue[QTRRFar] > Nero )
        QTRRC.readLine(QTRSensorsValue);
      while (QTRSensorsValue[QTRRFar] < Nero )
        QTRRC.readLine(QTRSensorsValue);
      delay(40);
      position = QTRRC.readLine(QTRSensorsValue);
      while ((position < 1000) || (position > 3500))
        position = QTRRC.readLine(QTRSensorsValue);
      Motor_brake(MOTOR_LEFT );
      Motor_brake(MOTOR_RIGHT);
      Stato = 0;
      break;

    case 2: //FINE LINEA
      Set_motor(MOTOR_LEFT , Velocita_Rotazione );
      Set_motor(MOTOR_RIGHT, -Velocita_Rotazione );
      while (QTRSensorsValue[QTRRFar] > Nero)
        QTRRC.readLine(QTRSensorsValue);
      while (QTRSensorsValue[QTRRFar] < Nero)
        QTRRC.readLine(QTRSensorsValue);
      delay(20);
      position = QTRRC.readLine(QTRSensorsValue);
      while ((position < 1000) || (position > 3500))
        position = QTRRC.readLine(QTRSensorsValue);
      Motor_brake(MOTOR_LEFT );
      Motor_brake(MOTOR_RIGHT);
      Stato = 0;
      break;

    case 3: //ANGOLO A SINISTRA
      Set_motor(MOTOR_LEFT , -Velocita_Rotazione );
      Set_motor(MOTOR_RIGHT, Velocita_Rotazione );
      while (QTRSensorsValue[QTRLFar] > Nero )
        QTRRC.readLine(QTRSensorsValue);
      while (QTRSensorsValue[QTRLFar] < Nero )
        QTRRC.readLine(QTRSensorsValue);
      position = QTRRC.readLine(QTRSensorsValue);
      while ((position < 1000) || (position > 3500))
        position = QTRRC.readLine(QTRSensorsValue);
      Motor_brake(MOTOR_LEFT );
      Motor_brake(MOTOR_RIGHT);
      Stato = 0;
      break;
  }
}

void Inizializzazione_Motori () {

  pinMode(MOTOR_STBY,  OUTPUT);
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

}


void Set_motor(boolean motor, char speed) { // imposta la velocità tra -100 (indietro) e +100 (avanti)
  byte PWMvalue = 0;
  PWMvalue = map(abs(speed), 0, 100, 50, 255);
  if (speed > 0)
    Motor_speed(motor, 0, PWMvalue);
  else if (speed < 0)
    Motor_speed(motor, 1, PWMvalue);
  else {
    Motor_coast(motor);
  }
}

void Motor_speed(boolean motor, boolean direction, byte speed) { // imposta la velocità tra 0 e 255
  if (motor == MOTOR_LEFT ) {
    if (direction == 0) {
      digitalWrite(MOTOR_A_IN1, HIGH);
      digitalWrite(MOTOR_A_IN2, LOW);
    }
    else {
      digitalWrite(MOTOR_A_IN1, LOW);
      digitalWrite(MOTOR_A_IN2, HIGH);
    }
    analogWrite(MOTOR_A_PWM, speed);
  }
  else {
    if (direction == 0) {
      digitalWrite(MOTOR_B_IN1, HIGH);
      digitalWrite(MOTOR_B_IN2, LOW);
    }
    else {
      digitalWrite(MOTOR_B_IN1, LOW);
      digitalWrite(MOTOR_B_IN2, HIGH);
    }
    analogWrite(MOTOR_B_PWM, speed);
  }
}

void Motor_standby(boolean Stato) { // abilita/disabilita i motori
  if (Stato == true)
    digitalWrite(MOTOR_STBY, LOW);
  else
    digitalWrite(MOTOR_STBY, HIGH);
}

void Motor_coast(boolean motor) { // motore in folle
  if (motor == MOTOR_LEFT ) {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_A_PWM, HIGH);
  }
  else {
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, LOW);
    digitalWrite(MOTOR_B_PWM, HIGH);
  }
}

void Motor_brake(boolean motor) { // freno motore
  if (motor == MOTOR_LEFT ) {
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, HIGH);
  }
  else {
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, HIGH);
  }
}

