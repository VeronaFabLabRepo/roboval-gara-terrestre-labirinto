#include <QTRSensors.h>

/*
 *@author: edoardo mori
 * @author: matteo olivato
 *
 * @school: liceo informatico "le stimate"
 *
 * Roboval 2012
 * Firmware per il robot Easy
 * 
 * per informazioni www.roboval.it
 *
 */

//--------------------------------------------------------------
//          parametri di configurazione hardcoded
//--------------------------------------------------------------
// i parametri seguenti descrivono l'interfacciamento del microcontrollore
// con sensori e motori. Se modificati potrebbero compromettere il 
// funzionamento del robot.

// definizione dei pin di collegamento tra arduino e la scheda motori
#define out_STBY    7
#define out_B_PWM   10 //B motore destro
#define out_A_PWM   5  //A motore sinistro
#define out_A_IN2   6
#define out_A_IN1   4
#define out_B_IN1   8
#define out_B_IN2   9
#define left_motor  0
#define right_motor 1

// definizione dei pin a cui sono collegati i sensori di linea
#define leftFar          0
#define leftCenter       1
#define leftNear         2
#define rightNear        3
#define rightCenter      4
#define rightFar         5
#define NUM_SENSORS      6
#define TIMEOUT       2500
#define EMITTER_PIN   QTR_NO_EMITTER_PIN

//--------------------------------------------------------------
//          parametri di configurazione modificabili
//--------------------------------------------------------------
// i parametri seguenti possono essere modificati a piacimento
// per migliorare il comportamento del robot
//

#define full_speed          70 // velocita' massima (0-100)
#define correction_speed    10 // velocita' di correzione traiettoria rettilinea (0-100)
#define turn_speed          38// velocita' di curvatura (0-100)
#define rallenta            25 // velocità di rallentamento durante la correzione
#define debugPeriod       1000 // ogni quanti millisecondi inviare i messaggi di debug
#define soglia             200 // soglia di riconoscimento bianco/nero (0-1000)
#define attesaPerManovra   10// tempo di attesa tra il riconoscimento della curva e il comando ai motori
#define attesaPerMDx       10 // tempo di attesa tra il riconoscimento della curva e il comando ai motori di destra
#define attesaPerMSx       2// tempo di attesa tra il riconoscimento della curva e il comando ai motori di sinistra
#define inversion_speed     70 // velocità con cui inverte la direzione

// dichiarazione variabili globali
int leftCenterReading;
int leftNearReading;
int leftFarReading;
int rightCenterReading;
int rightNearReading;
int rightFarReading;
int lastDebugTime = 0;

QTRSensorsRC qtrrc((unsigned char[]) {
  A0, A1, A2, A3, A4, A5}
,NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

void setup(){
  pinMode(out_STBY,OUTPUT);
  pinMode(out_A_PWM,OUTPUT);
  pinMode(out_A_IN1,OUTPUT);
  pinMode(out_A_IN2,OUTPUT);
  pinMode(out_B_PWM,OUTPUT);
  pinMode(out_B_IN1,OUTPUT);
  pinMode(out_B_IN2,OUTPUT);

  for (int i = 0; i < 400; i++){
    qtrrc.calibrate();
  }
  motor_standby(false);
}

void loop(){
  readSensors();
  if(!leftFarReading && leftNearReading && rightNearReading && !rightFarReading){
    set_motor(right_motor, full_speed);
    set_motor(left_motor, full_speed);
    return;
  }

  readSensors();
  if(leftFarReading || rightFarReading || (!rightNearReading && !leftNearReading)){
    leftHandWall(); //gira
    return;
  }

  readSensors();
  if(!leftFarReading && !rightFarReading && ((leftNearReading && !rightCenterReading) || (rightNearReading && !leftCenterReading)) ){  // 0x1xx0 || 0xx1x0
    straight();
    return;
  }

}

void straight(){
  readSensors();
  if(!leftNearReading){// 0x01x0 il robot e' troppo a sinistra, deve andare piu' a destra quindi alzo il motore sinistro
    RadDx();
    return;
  }
  readSensors();
  if(!rightNearReading){// 0x10x0 il robot e' troppo a destra, deve andare piu' a sinistra quindi alzo il motore destro
    RadSx();
    return;
  }
}

void leftHandWall(){
  readSensors();
  if(leftFarReading){ // 1xxxx
    turnLeft();
  }

  readSensors();
  if(!leftFarReading && !leftCenterReading && rightFarReading){  // 00xxx1
    turnRight();
  }

  readSensors();
  if(!leftFarReading && !leftCenterReading && !leftNearReading
    && !rightFarReading && !rightCenterReading && !rightNearReading){ // 000000
    turnAround();
  }
}

// raddrizza a sinistra(aumento motore destro)
void RadSx(){
  set_motor(left_motor,correction_speed);
  set_motor(right_motor,full_speed - rallenta);
  readSensors();
  while(!rightNearReading && !leftFarReading && leftCenterReading){
    readSensors();
  }
}

// raddrizza a destra(aumento motore sinistro)
void RadDx(){
  set_motor(left_motor,full_speed - rallenta);
  set_motor(right_motor,correction_speed);
  readSensors();
  while(!leftNearReading && !rightFarReading && rightCenterReading){
    readSensors();
  }
}

void turnLeft(){
  delay(attesaPerMSx);
  set_motor(left_motor,-turn_speed);
  set_motor(right_motor,turn_speed);
  readSensors();
  while(!leftNearReading || !rightNearReading){
    readSensors();
  }
}
void turnRight(){
  delay(attesaPerMDx);
  set_motor(left_motor,turn_speed);
  set_motor(right_motor,-turn_speed);
  readSensors();
  while(!leftNearReading || !rightNearReading){  
    readSensors();
  }
}
void turnAround(){
  delay(attesaPerManovra);
  set_motor(left_motor,inversion_speed);
  set_motor(right_motor,-inversion_speed);
  readSensors();
  while(!leftNearReading || !rightNearReading){
    readSensors();
  }
}

void readSensors(){
  unsigned int position = qtrrc.readLine(sensorValues);
  leftFarReading     = sensorValues[leftFar]>soglia;
  leftCenterReading  = sensorValues[leftCenter]>soglia;
  leftNearReading    = sensorValues[leftNear]>soglia;
  rightNearReading   = sensorValues[rightNear]>soglia;
  rightCenterReading = sensorValues[rightCenter]>soglia;
  rightFarReading    = sensorValues[rightFar]>soglia;
}

/**
 * set di funzioni per la gestione dei motori
 *
 */

void set_motor(boolean motor, char speed) { // imposta la velocità tra -100 (indietro) e +100 (avanti)
  byte PWMvalue=0;
  PWMvalue = map(abs(speed),0,100,50,255);
  if (speed > 0)
    motor_speed(motor,0,PWMvalue);
  else if (speed < 0)
    motor_speed(motor,1,PWMvalue);
  else {
    motor_coast(motor);
  }
}
void motor_speed(boolean motor, boolean direction, byte speed) { // imposta la velocità tra 0 e 255
  if (motor == left_motor) {
    if (direction == 0) {
      digitalWrite(out_A_IN1,HIGH);
      digitalWrite(out_A_IN2,LOW);
    } 
    else {
      digitalWrite(out_A_IN1,LOW);
      digitalWrite(out_A_IN2,HIGH);
    }
    analogWrite(out_A_PWM,speed);
  } 
  else {
    if (direction == 0) {
      digitalWrite(out_B_IN1,HIGH);
      digitalWrite(out_B_IN2,LOW);
    } 
    else {
      digitalWrite(out_B_IN1,LOW);
      digitalWrite(out_B_IN2,HIGH);
    }
    analogWrite(out_B_PWM,speed);
  }
}
void motor_standby(boolean state) { // abilita/disabilita i motori
  if (state == true)
    digitalWrite(out_STBY,LOW);
  else
    digitalWrite(out_STBY,HIGH);
}

void motor_coast(boolean motor) { // motore in folle
  if (motor == left_motor) {
    digitalWrite(out_A_IN1,LOW);
    digitalWrite(out_A_IN2,LOW);
    digitalWrite(out_A_PWM,HIGH);
  } 
  else {
    digitalWrite(out_B_IN1,LOW);
    digitalWrite(out_B_IN2,LOW);
    digitalWrite(out_B_PWM,HIGH);
  }
}

void motor_brake(boolean motor) { // freno motore
  if (motor == left_motor) {
    digitalWrite(out_A_IN1,HIGH);
    digitalWrite(out_A_IN2,HIGH);
  } 
  else {
    digitalWrite(out_B_IN1,HIGH);
    digitalWrite(out_B_IN2,HIGH);
  }
}


