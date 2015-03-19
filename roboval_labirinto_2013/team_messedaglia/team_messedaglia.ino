#include <QTRSensors.h>

/*
* @authors: Simone de Blasiis, Francesco Ambrosini e Matteo Iervasi
* @team: Team Messedaglia
* @sponsor: 
 *
 * Roboval 2013
 * Firmware base per il robot Easy
 * Soluzione di un labirinto composto da una linea nera su fondo bianco.
 *
 * Contest Roboval 2013
 *
 */
#define out_STBY    7
#define out_B_PWM   10
#define out_A_PWM   5 
#define out_A_IN2   6
#define out_A_IN1   4
#define out_B_IN1   8
#define out_B_IN2   9
#define left_motor  0
#define right_motor 1

#define leftFar          0
#define leftCenter       1
#define leftNear         2
#define rightNear        3
#define rightCenter      4
#define rightFar         5
#define NUM_SENSORS      6
#define TIMEOUT       3000
#define EMITTER_PIN   QTR_NO_EMITTER_PIN

#define full_speed         35
#define correction_speed    20
#define turn_speed          10 
#define soglia              80
#define attesaPerManovra    100 


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

  for (int i = 0; i < 800; i++){
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
    set_motor(right_motor, full_speed);
    set_motor(left_motor, full_speed);
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
  set_motor(left_motor,-turn_speed);
  set_motor(right_motor,turn_speed);
  delay(500);
  while(!leftNearReading || !rightNearReading){
    //set_motor(left_motor,-turn_speed);
    //set_motor(right_motor,turn_speed);
    readSensors();
  }
}
void turnRight(){
  delay(attesaPerManovra);
  set_motor(left_motor,turn_speed);
  set_motor(right_motor,-turn_speed);
  delay(200);
  while(!leftNearReading || !rightNearReading){
    //set_motor(left_motor,turn_speed);
    //set_motor(right_motor,-turn_speed);
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
