#include <QTRSensors.h>

/*
* autore: Albanese Andrea
 ITIS "G. Ferraris" 4B.
 
 * Contest Roboval 2012
 * www.roboval.it
 * 
 * 
 * 
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
// 
//

#define full_speed          50 // velocita' massima (0-100)
#define correction_speed    40 // velocita' di correzione traiettoria rettilinea (0-100)
#define super_correction_speed  10
#define turn_speed          30 // velocita' di curvatura (0-100)
#define debugPeriod       3000 // ogni quanti millisecondi inviare i messaggi di debug
#define soglia             200 // soglia di riconoscimento bianco/nero (0-1000)
#define attesaPerManovra    10 // tempo di attesa tra il riconoscimento della curva e il comando ai motori


// dichiarazione variabili globali
int S1;
int S2;
int S0;
int S4;
int S3;
int S5;
int lastDebugTime = 0;
int cont;

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

  for (int i = 0; i < 100; i++){
    qtrrc.calibrate();
    cont=0;
  }
  motor_standby(false);
}
void loop(){
  readSensors();
  cont++;

  if(!S0 && !S5 && (S2 || S3 )){  // 0x1xx0 || 0xx1x0
   straight();
  }
  else{
       if(S5 && S4 && !S0)
         turnRight();
       if(S0 && S1)
         turnLeft();
       if(S5 && S0)
       leftHandWall();
      if(!S0 && !S2 && !S4)
       turnAround(); 
  }
}

void straight(){
  if(!S2){// 0x01x0 il robot e' troppo a sinistra, deve andare piu' a destra quindi alzo il motore sinistro
     if(cont>100)
     {  
       
       cont=0;
        set_motor(left_motor, full_speed);
    set_motor(right_motor, correction_speed);
  }

  else
  {
   
     set_motor(left_motor, full_speed);
    set_motor(right_motor, super_correction_speed);
    
  }
    return;
  }
  else if(!S3){// 0x10x0 il robot e' troppo a destra, deve andare piu' a sinistra quindi alzo il motore destro
      if(cont>100)
      {
        
        cont=0;
        set_motor(left_motor, correction_speed);
    set_motor(right_motor, full_speed);
    
  }
    else
    {
     
      set_motor(left_motor, super_correction_speed);
    set_motor(right_motor, full_speed);
   
  }
    return;
  }
  else{ // 0x11x0 il robot e' sopra la linea
    set_motor(right_motor, full_speed);
    set_motor(left_motor, full_speed);
  }
}

void leftHandWall(){
  if(S0 && S5){
   turnLeft();
}
}
void turnLeft(){
  delay(attesaPerManovra);
  while(S0 ){
    set_motor(left_motor,-turn_speed);
    set_motor(right_motor,turn_speed);
    readSensors();
  }
}
void turnRight(){
  delay(attesaPerManovra);
  while(S5 && !S0 ){
    set_motor(left_motor,turn_speed);
    set_motor(right_motor,-turn_speed);
    readSensors();
  }
}
void turnAround(){
  delay(attesaPerManovra);
  while(!S2 || !S3){
    set_motor(left_motor,-turn_speed);
    set_motor(right_motor,turn_speed);
    readSensors();
  }
}
void readSensors(){
  unsigned int position = qtrrc.readLine(sensorValues);
  S0     = sensorValues[leftFar]>soglia;
  S1  = sensorValues[leftCenter]>soglia;
  S2    = sensorValues[leftNear]>soglia;
  S3   = sensorValues[rightNear]>soglia;
  S4 = sensorValues[rightCenter]>soglia;
  S5    = sensorValues[rightFar]>soglia;
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

