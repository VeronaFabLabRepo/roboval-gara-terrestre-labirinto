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

/*Questo abb ok

#define full_speed          40 // velocita' massima (0-100)
#define correction_speed    39 // velocita' di correzione traiettoria rettilinea (0-100)
#define turn_speed          20 // velocita' di curvatura (0-100)
#define debugPeriod       1000 // ogni quanti millisecondi inviare i messaggi di debug
#define correction_time     50 // millisecondi per la correzione sul rettilineo
#define soglia             150 // soglia di riconoscimento bianco/nero (0-1000)
#define attesaPerManovra    50 // tempo di attesa tra il riconoscimento della curva e il comando ai motori
*/

/* Questo piu che ok
#define full_speed          20 // velocita' massima (0-100)
#define correction_speed    10 // velocita' di correzione traiettoria rettilinea (0-100)
#define turn_speed          10 // velocita' di curvatura (0-100)
#define debugPeriod       1000 // ogni quanti millisecondi inviare i messaggi di debug
#define correction_time     25 // millisecondi per la correzione sul rettilineo
#define soglia             150 // soglia di riconoscimento bianco/nero (0-1000)
#define attesaPerManovra    25 // tempo di attesa tra il riconoscimento della curva e il comando ai motori
*/

/* Questo a 15*/
#define full_speed          15 // velocita' massima (0-100)
#define correction_speed    7 // velocita' di correzione traiettoria rettilinea (0-100)
#define turn_speed          10 // velocita' di curvatura (0-100)
#define debugPeriod       1000 // ogni quanti millisecondi inviare i messaggi di debug ***non usato
#define correction_time     25 // millisecondi per la correzione sul rettilineo
#define soglia             150 // soglia di riconoscimento bianco/nero (0-1000)
#define attesaPerManovra    25 // tempo di attesa tra il riconoscimento della curva e il comando ai motori


/**/

/* Insomma....
#define full_speed          10 // velocita' massima (0-100)
#define correction_speed    5 // velocita' di correzione traiettoria rettilinea (0-100)
#define turn_speed          10 // velocita' di curvatura (0-100)
#define debugPeriod       1000 // ogni quanti millisecondi inviare i messaggi di debug
#define correction_time     25 // millisecondi per la correzione sul rettilineo
#define soglia             150 // soglia di riconoscimento bianco/nero (0-1000)
#define attesaPerManovra    25 // tempo di attesa tra il riconoscimento della curva e il comando ai motori
*/

/* Otto

#define full_speed          8 // velocita' massima (0-100)
#define correction_speed    8 // velocita' di correzione traiettoria rettilinea (0-100)
#define turn_speed          10 // velocita' di curvatura (0-100)
#define debugPeriod       1000 // ogni quanti millisecondi inviare i messaggi di debug
#define correction_time     0 // millisecondi per la correzione sul rettilineo
#define soglia             150 // soglia di riconoscimento bianco/nero (0-1000)
#define attesaPerManovra    21 // tempo di attesa tra il riconoscimento della curva e il comando ai motori
*/

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

  Serial.begin(9600);
  Serial.println("Calibro il sensore...");

  calibrazione_riki();
  Serial.println("VAI!!");
  motor_standby(false);
}
void loop(){
  sensorsDebug();
  readSensors();

  if(!leftFarReading && !rightFarReading && (leftNearReading || rightNearReading) ){  // 0x1xx0 || 0xx1x0
    straight();
  }
  else{
    leftHandWall(); //gira
  }
   //Consigliato su internet, x una migliore lettura dei sensori
 //delay(1);
}

void straight(){
  if(!leftNearReading){// 0x01x0 il robot e' troppo a sinistra, deve andare piu' a destra quindi alzo il motore sinistro
    Serial.println("Correzione SX");
    set_motor(left_motor, full_speed);
    set_motor(right_motor, correction_speed);
    delay(correction_time);
    set_motor(right_motor, full_speed);
    return;
  }
  else if(!rightNearReading){// 0x10x0 il robot e' troppo a destra, deve andare piu' a sinistra quindi alzo il motore destro
    Serial.println("Correzione DX");
    set_motor(left_motor, correction_speed);
    set_motor(right_motor, full_speed);
    delay(correction_time);
    set_motor(left_motor, full_speed);
    return;
  }
  else{ // 0x11x0 il robot e' sopra la linea
    set_motor(right_motor, full_speed);
    set_motor(left_motor, full_speed);
  }
}

void leftHandWall(){
  //Se è tutto nero
  /*if(leftFarReading && leftCenterReading && leftNearReading && rightFarReading && rightCenterReading && rightNearReading){ // 11111
    turnAroundIncrocio();
  }*/

  if(leftFarReading){ // 1xxxxx
    turnLeft();
  }
  if(!leftFarReading && rightFarReading){  // 0xxxx1
    turnRight();
  }
  if(!leftFarReading && !leftCenterReading && !leftNearReading
    && !rightFarReading && !rightCenterReading && !rightNearReading){ // 000000
    turnAround();
  }
}
void turnLeft(){
  Serial.println("Giro a SX");
  delay(attesaPerManovra);
  while(!leftNearReading || !rightNearReading){
    set_motor(left_motor,-turn_speed);
    set_motor(right_motor,turn_speed);
    readSensors();
  }
}
void turnRight(){
  delay(attesaPerManovra);
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

void turnAroundIncrocio(){
  set_motor(left_motor,turn_speed);
  set_motor(right_motor,turn_speed);
  delay(attesaPerManovra);
  while(!leftNearReading || !rightNearReading){
    set_motor(left_motor,-turn_speed);
    set_motor(right_motor,turn_speed);
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
void readSensors_calibrazione(){
  unsigned int position = qtrrc.readLine(sensorValues);
  leftFarReading     = sensorValues[leftFar];
  leftCenterReading  = sensorValues[leftCenter];
  leftNearReading    = sensorValues[leftNear];
  rightNearReading   = sensorValues[rightNear];
  rightCenterReading = sensorValues[rightCenter];
  rightFarReading    = sensorValues[rightFar];
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


void sensorsDebug(){
  Serial.print("Lettura:...");
  Serial.print(leftFarReading);
  Serial.print(" ");
  Serial.print(leftNearReading);
  Serial.print(" ");
  Serial.print(leftCenterReading);
  Serial.print(" ");
  Serial.print(rightCenterReading);
  Serial.print(" ");
  Serial.print(rightNearReading);
  Serial.print(" ");
  Serial.println(rightFarReading);
}


void calibrazione_riki(){
  sensorsDebug();
  int calibcount=0;
  while(calibcount<20){
 // for(int i=0; i<4000; i++){
    qtrrc.calibrate();
    readSensors_calibrazione();
    sensorsDebug();
    readSensors();
    sensorsDebug();
    //delay(200);
    if((leftNearReading==1) && (rightNearReading==1) && (leftFarReading == 0 ) && (rightFarReading ==0)){
      
      // A volte legge il valore 1000 il sensore e sballa tutto
     // readSensors_calibrazione();
     // if((leftCenterReading!=1000) && (rightCenterReading!=1000)){
        calibcount=calibcount+1;
      //}*/
    }
  }
  Serial.println("Calibrazione OK!!!!!");
  
  
  //Accendo il led 13 integrato in arduino per indicare che la calibrazione è ok!
  /* pinMode(13, HIGH);
  for(int i=0; i<10; i++){
    digitalWrite(13, HIGH);
    delay(200);
    digitalWrite(13, LOW);
    delay(200);
  }*/
}









