#include <QTRSensors.h>

/*
 * @author: Baviero Enrico & Sacco Nicolò 
 * @istituto: IIS "Dal Cero" di San Bonifacio
 * Roboval 2012
 * 
 * Soluzione di un labirinto composto da una linea nera su fondo bianco.
 *
 * Contest Roboval 2012
 *
 * Il fw da noi utilizzato è quello fornito da Alberto Valente considerato fw di base,
 * ovviamente abbiamo appportato delle modifiche ai valori dei #define per migliorare
 * precisione e velocità.Il fw è stato utilizzato in entrambe le sessioni di gara
 * (lo diciamo perchè si erano alimentati sospetti di una modifica apportata al fw 
 * tra una sessione e l'altra per far superare al robot l'ostacolo incontrato nella 
 * prima sessione)
 * Grazie per la bellissima giornata è speriamo di rivedervi a Roboval 2013!!
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
#define TIMEOUT          2500
#define EMITTER_PIN   QTR_NO_EMITTER_PIN

//--------------------------------------------------------------
//          parametri di configurazione modificabili
//--------------------------------------------------------------
// i parametri seguenti possono essere modificati a piacimento
// per migliorare il comportamento del robot
//

#define full_speed          42// velocita' massima (0-100)
#define correction_speed    1// velocita' di correzione traiettoria rettilinea (0-100)
#define turn_speed          32 // velocita' di curvatura (0-100)
#define debugPeriod         1 // ogni quanti millisecondi inviare i messaggi di debug
#define soglia             180 // soglia di riconoscimento bianco/nero (0-1000)
#define attesaPerManovra    0 // tempo di attesa tra il riconoscimento della curva e il comando ai motori


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

  if(!leftFarReading && !rightFarReading && (leftNearReading || rightNearReading) ){  
    straight();
  }
  else{
    leftHandWall(); 
  }
}

void straight(){
  if(!leftNearReading){
    set_motor(left_motor, full_speed);
    set_motor(right_motor, correction_speed);
    return;
  }
  else if(!rightNearReading){
    set_motor(left_motor, correction_speed);
    set_motor(right_motor, full_speed);
    return;
  }
  else{ 
    set_motor(right_motor, 45);
    set_motor(left_motor, 45);
  }
}

void leftHandWall(){
  if(leftFarReading){ 
    turnLeft();
  }
  if(!leftFarReading && rightFarReading){  
    turnRight();
  }
  if(!leftFarReading && !leftCenterReading && !leftNearReading
    && !rightFarReading && !rightCenterReading && !rightNearReading){ 
    turnAround();
  }
}
void turnLeft(){
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

void set_motor(boolean motor, char speed) { 
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
void motor_speed(boolean motor, boolean direction, byte speed) { 
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
void motor_standby(boolean state) { 
  if (state == true)
    digitalWrite(out_STBY,LOW);
  else
    digitalWrite(out_STBY,HIGH);
}

void motor_coast(boolean motor) { 
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

void motor_brake(boolean motor) { 
  if (motor == left_motor) {
    digitalWrite(out_A_IN1,HIGH);
    digitalWrite(out_A_IN2,HIGH);
  } 
  else {
    digitalWrite(out_B_IN1,HIGH);
    digitalWrite(out_B_IN2,HIGH);
  }
}










