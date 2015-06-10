#include <QTRSensors.h>

/*
* @author: alberto valente alb.valente@gmail.com
 *
 * Roboval 2012
 * Firmware base per il robot Easy
 * Soluzione di un labirinto composto da una linea nera su fondo bianco.
 *
 * Contest Roboval 2012
 *
 * I concorrenti sono liberi di modificare a piacimento questo firmware per 
 * migliorare il comportamento del robot
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


#define max_speed          60 // velocita' massima di correzione (0-100) --> (default = 40)
#define rightmax_speed          57 // velocita' massima di correzione (0-100) --> (default = 40)
#define full_speed          40 // velocita' massima (0-100)
#define rightfull_speed          37 // velocita' massima di correzione (0-100) --> (default = 40)
#define correction_speed    25 // velocita' di correzione traiettoria rettilinea (0-100)--> (default = 38)
#define rightcorrection_speed    22 // velocita' di correzione traiettoria rettilinea (0-100)--> (default = 38) 
#define turn_speed          35 // velocita' di curvatura (0-100)
#define debugPeriod        500 // ogni quanti millisecondi inviare i messaggi di debug -->  (default = 1000)
#define soglia             200 // soglia di riconoscimento bianco/nero (0-1000) spostato da 200 a 300
#define attesaPerManovra    50 // tempo di attesa tra il riconoscimento della curva e il comando ai motori

#define Rspeed  (35/1.1)+20
#define Rcspeed (34/1.1)+20
#define Lspeed  (40/1.1)+20
#define Lcspeed (39/1.1)+20


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

  if(!leftFarReading && !rightFarReading){  // 0x1xx0 || 0xx1x0
    straight();
    //readSensors();
  }
  else{
    leftHandWall(); //gira
    //readSensors();
  }
}

void straight(){
  if(!leftCenterReading){// 0x0x10 il robot e' troppo a sinistra, deve andare piu' a destra quindi alzo il motore sinistro
    set_motor(left_motor, Lspeed);
    set_motor(right_motor, Rcspeed);
    //delay(10);
    //readSensors();
    return;
  }
  else if(!rightCenterReading){// 01x0x0 il robot e' troppo a destra, deve andare piu' a sinistra quindi alzo il motore destro
    set_motor(left_motor, Lcspeed);
    set_motor(right_motor, Rspeed);
    //delay(10);
    //readSensors();
    return;
  }
  else{ // 0x11x0 il robot e' sopra la linea
    set_motor(right_motor, Rspeed);
    set_motor(left_motor, Lspeed);
    //readSensors();
  }
}

void leftHandWall(){
  if(leftFarReading){ // 1xxxxx
    turnLeft();
  }
  else if(!leftFarReading && rightFarReading){  // 0xxxx1
    turnRight();
  }
  else if(!leftFarReading && !leftCenterReading && !leftNearReading && !rightFarReading && !rightCenterReading && !rightNearReading){ // 000000
    turnAround();
   //else if (!leftFarReading && !leftCenterReading && !leftNearReading && !rightFarReading && !rightCenterReading && !rightNearReading) back();
  }
}
void turnLeft(){
  //delay(attesaPerManovra);
  while(!leftNearReading || !rightNearReading){ 
    set_motor(left_motor,-turn_speed);
    set_motor(right_motor,turn_speed);
    readSensors();
  }
}
void turnRight(){
  //delay(attesaPerManovra);
  while(!leftNearReading || !rightNearReading){
    set_motor(left_motor,turn_speed);
    set_motor(right_motor,-turn_speed);
    readSensors();
  }
}
void turnAround(){
  delay(attesaPerManovra);
  while(!leftNearReading || !rightNearReading){
    set_motor(left_motor,-turn_speed);
    set_motor(right_motor,turn_speed);
    readSensors();
  }
}
void back(){
  //delay(attesaPerManovra);
  while(!leftFarReading || !rightFarReading){
    set_motor(left_motor,-Lspeed);
    set_motor(right_motor,-Rspeed);
    delay(10000); //vediamo oh oh oh
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










