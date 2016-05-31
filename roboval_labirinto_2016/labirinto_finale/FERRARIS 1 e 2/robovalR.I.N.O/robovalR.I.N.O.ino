

/*
* Squadra: Ferraris 2
* Robot:   Rino
* @autori: Carlini,Poletto,Cestone,Bertolasi
* @scuola: Ferraris
* @data: Maggio 2016
* @versione: -
* @evento: Roboval 2016
* 
*/




#include <QTRSensors.h>


#define black 600
#define rotation 27
#define line 88

#define out_STBY 7
#define out_B_PWM 10 //B motore destro
#define out_A_PWM 5 //A motore sinistro
#define out_A_IN2 6
#define out_A_IN1 4
#define out_B_IN1 8
#define out_B_IN2 9
#define left_motor 0
#define right_motor 1

//SENSORI
#define leftFar          0
#define leftNear         1
#define leftCenter       2
#define rightCenter      3
#define rightNear        4
#define rightFar         5
#define NUM_SENSORS      6
#define TIMEOUT       2500
#define EMITTER_PIN   QTR_NO_EMITTER_PIN


// dichiarazione variabili globali
int lastDebugTime = 0;
int error, lastError = 0;
int m1Speed, m2Speed;
int state = 0;
int motorSpeed;
unsigned int position;

QTRSensorsRC qtrrc((unsigned char[]) {
  A0, A1, A2, A3, A4, A5
}
, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

/**
  set di funzioni

*/
void set_motor(boolean motor, char speed) { // imposta la velocità tra -100 (indietro) e +100 (avanti)
  byte PWMvalue = 0;
  PWMvalue = map(abs(speed), 0, 100, 50, 255);
  if (speed > 0)
    motor_speed(motor, 0, PWMvalue);
  else if (speed < 0)
    motor_speed(motor, 1, PWMvalue);
  else {
    motor_coast(motor);
  }
}

void motor_speed(boolean motor, boolean direction, byte speed) { // imposta la velocità tra 0 e 255
  if (motor == left_motor) {
    if (direction == 0) {
      digitalWrite(out_A_IN1, HIGH);
      digitalWrite(out_A_IN2, LOW);
    }
    else {
      digitalWrite(out_A_IN1, LOW);
      digitalWrite(out_A_IN2, HIGH);
    }
    analogWrite(out_A_PWM, speed);
  }
  else {
    if (direction == 0) {
      digitalWrite(out_B_IN1, HIGH);
      digitalWrite(out_B_IN2, LOW);
    }
    else {
      digitalWrite(out_B_IN1, LOW);
      digitalWrite(out_B_IN2, HIGH);
    }
    analogWrite(out_B_PWM, speed);
  }
}

void motor_standby(boolean state) { // abilita/disabilita i motori
  if (state == true)
    digitalWrite(out_STBY, LOW);
  else
    digitalWrite(out_STBY, HIGH);
}

void motor_coast(boolean motor) { // motore in folle
  if (motor == left_motor) {
    digitalWrite(out_A_IN1, LOW);
    digitalWrite(out_A_IN2, LOW);
    digitalWrite(out_A_PWM, HIGH);
  }
  else {
    digitalWrite(out_B_IN1, LOW);
    digitalWrite(out_B_IN2, LOW);
    digitalWrite(out_B_PWM, HIGH);
  }
}

void motor_brake(boolean motor) { // freno motore
  if (motor == left_motor) {
    digitalWrite(out_A_IN1, HIGH);
    digitalWrite(out_A_IN2, HIGH);
  }
  else {
    digitalWrite(out_B_IN1, HIGH);
    digitalWrite(out_B_IN2, HIGH);
  }
}


void setup() {
  Serial.begin(9600);
  motor_standby(false);
  pinMode(out_STBY, OUTPUT);
  pinMode(out_A_PWM, OUTPUT);
  pinMode(out_A_IN1, OUTPUT);
  pinMode(out_A_IN2, OUTPUT);
  pinMode(out_B_PWM, OUTPUT);
  pinMode(out_B_IN1, OUTPUT);
  pinMode(out_B_IN2, OUTPUT);
  Serial.println("calib");
  for (int i = 0; i < 200; i++) {
    qtrrc.calibrate();
  }
}

void loop() {

  switch (state) {
    case 0:
      position = qtrrc.readLine(sensorValues);
      error = position - 2500;
      motorSpeed = 0.1 * error + 5 * (error - lastError);
      lastError = error;
      m1Speed = line + motorSpeed;
      m2Speed = line - motorSpeed;
      if (m1Speed < 10)
        m1Speed = 10;
      if (m2Speed < 10)
        m2Speed = 10;
      if (m1Speed > 100)
        m1Speed = 80;
      if (m2Speed > 100)
        m2Speed = 80;
      set_motor(left_motor, m1Speed + 16);
      set_motor(right_motor, m2Speed);

      qtrrc.readLine(sensorValues);

      if (sensorValues[rightFar] > black && sensorValues[rightNear] > black  ) {
        state = 3;    //ANGOLO A SINISTRA
        motor_brake(left_motor);
        motor_brake(right_motor);
        break;
      }

      if (sensorValues[leftFar] < 200 && sensorValues[leftNear] < 200 && sensorValues[leftCenter] < 200 && sensorValues[rightFar] < 200 && sensorValues[rightNear] < 200 && sensorValues[rightCenter] < 200) {
        state = 5;    //FINE LINEA
        motor_brake(left_motor);
        motor_brake(right_motor);
        break;
      }

      break;


    case 4: //ANGOLO A DESTRA
      set_motor(left_motor, rotation);
      set_motor(right_motor, -rotation);
      while (sensorValues[rightFar] > black )
        qtrrc.readLine(sensorValues);
      while (sensorValues[rightFar] < black )
        qtrrc.readLine(sensorValues);
      delay(40);
      position = qtrrc.readLine(sensorValues);
      while ((position < 1000) || (position > 3000))
        position = qtrrc.readLine(sensorValues);
      motor_brake(left_motor);
      motor_brake(right_motor);
      state = 0;
      break;

    case 5: //FINE LINEA
      set_motor(left_motor, -45-16);
      set_motor(right_motor, 45);
      while (sensorValues[leftFar] > black)
        qtrrc.readLine(sensorValues);
      while (sensorValues[leftFar] < black)
        qtrrc.readLine(sensorValues);
      delay(20);
      position = qtrrc.readLine(sensorValues);
      while ((position < 1500) || (position > 3500))
        position = qtrrc.readLine(sensorValues);
      motor_brake(left_motor);
      motor_brake(right_motor);
      state = 0;
      break;

    case 3: //ANGOLO A DESTRA
      set_motor(left_motor, rotation + 16);
      set_motor(right_motor, -rotation);
      while (sensorValues[rightFar] > black )
        qtrrc.readLine(sensorValues);
      while (sensorValues[rightFar] < black )
        qtrrc.readLine(sensorValues);
      position = qtrrc.readLine(sensorValues);
      while ((position < 1000) || (position > 3500))
        position = qtrrc.readLine(sensorValues);
      motor_brake(left_motor);
      motor_brake(right_motor);
      state = 0;
      break;
  }
}



