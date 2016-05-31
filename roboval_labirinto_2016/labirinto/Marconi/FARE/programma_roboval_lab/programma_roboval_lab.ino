 
/*
 * @scuola:	ITIS Marconi
 * @autori:	Federico Ciresola, Andrea Masili, Elia Giusti
 * Squadra: Marconi 1
 * Robot:   FARE
 * @data:   Maggio 2016
 * Versione: - 
 * @evento: Roboval 2016
 * 
 */

#include <QTRSensors.h>
float Kp = 0.07; // 0.3//2.5  0.85
float Ki = 0; //0.005*/
float Kd = 1.5;

#define out_STBY    7
#define out_B_PWM   10 //B motore destro
#define out_A_PWM   5  //A motore sinistro
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
#define TIMEOUT       2500
#define EMITTER_PIN   QTR_NO_EMITTER_PIN

#define DEBFULL 0
#define SOGLIA 180

#define MAXSP  100
#define MINSP  25

#define soglia 200

#define SOG_ST 10


int maxSpeed=100; //di default 100 max speed of the robot

int baseSpeed=65;//di default 50 this is the speed at which the motors should spin when the robot is perfectly on the line

int lc,ln,lf,rc,rn,rf;
int lastDebugTime = 0;
int lastError = 0;
int I=0;
int time=0;
int rightMotorSpeed;
int leftMotorSpeed;
int scarto=200;

QTRSensorsRC qtrrc((unsigned char[]) {
  A0, A1, A2, A3, A4, A5}
,NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

int matrice[6][6];
int indM=0;//valore per la matrice
int sensori[6];
int ps=0;
char riga[100];
unsigned long lastMs,currMs;  //Per gestire numero di ril al sec
int numCamp=0;          // num campionamenti


void setup(){
  Serial.begin(9600);
  pinMode(out_STBY,OUTPUT);
  pinMode(out_A_PWM,OUTPUT);
  pinMode(out_A_IN1,OUTPUT);
  pinMode(out_A_IN2,OUTPUT);
  pinMode(out_B_PWM,OUTPUT);
  pinMode(out_B_IN1,OUTPUT);
  pinMode(out_B_IN2,OUTPUT);
  Serial.println("inizio");
  for (int i = 0; i < 400; i++){
    qtrrc.calibrate();
  }
  Serial.print("finio");
  motor_standby(false);
  for(int r=0;r<6;r++){
    for(int c=0;c<6;c++){
      matrice[r][c]=0;
    }
  }

  numCamp=0;
  lastMs=millis();
}

// costanti che definiscono lo stato di lettura
#define ST_N 0
#define ST_T 1
#define ST_DX 2
#define ST_SX 3 
#define ST_VU 4

int stp=ST_VU;  //stato precedente
int sta=ST_N; // stato attuale

void loop()
{
  sta=ST_N;

  numCamp++;
  currMs=millis();
  if (currMs-lastMs >=1000){
    Serial.print("NC");
    Serial.println(numCamp);
    lastMs=currMs;
    numCamp=0;    

  }


  unsigned int sensors[6];
  boolean sen[6];
  int aposition = qtrrc.readLine(sensors); 
  mediaoCircolare(sensors);
  //PID
  int valRead = aposition;

  //Serial.println(valRead);

  /* algoritmo:
   normalmente PID
   in condizioni particolari si fanno azioni particolari
   T o INCROCIO si va a dx
   D si va a dx
   S per capire che è solo S e non una T a sx si deve verificare la condizione 
   prima sx e poi no 
   
   */

  /*if( ((valRead>=2500-scarto) && (valRead<=2500+scarto)) && ((sensori[0] && sensori[5]))){ // leggo una T
   sta=ST_T;
   }*/
  if( sensori[0] && sensori[1] && sensori[4] && sensori[5]){ // leggo una T
    sta=ST_T;
  }
  else if(!sensori[0]&& sensori[3] && sensori[4] && sensori[5]){ //destra
    sta=ST_DX;
  }
  else if(!sensori[5]&& sensori[0] && sensori[1] && sensori[2]){ //sinistra
    sta=ST_SX;
  }
  else if(!(sensori[0]&& sensori[1] && sensori[2] && sensori[3] && sensori[4] && sensori[5])){ //vuoto
    sta=ST_VU;
  }
  /*else if( (valRead >=3500-scarto) && (valRead  <= 4000+scarto) && ((sensori[5] && sensori[4]))){ //destra
   sta=ST_DX;
   }
   else if( (valRead >= 1000-scarto) && (valRead <= 1500+scarto) && ((sensori[0] && sensori[1]))){ //sinistra
   sta=ST_SX;
   }
   else if( valRead >=0 && valRead <=scarto){ //vuoto
   sta=ST_VU;
   }*/

  /*if(sta ==ST_DX) Serial.println("D");
   if(sta ==ST_SX) Serial.println("S");
   if(sta ==ST_T) Serial.println("T");
   if(sta ==ST_VU) Serial.println("V");
   */
  Serial.print(".");
  if(sta == ST_DX || sta == ST_T){ //destra e T
    motors(70,-70);
    delay(20);
    /*    if(sta ==ST_DX)
     Serial.println("D");
     else if(sta== ST_T)
     Serial.println("T");*/
  }   
  else if(sta  == ST_VU && stp == ST_SX ){ //sinistra
    motors(-70,70);
    delay(20);

    //Serial.println("S");
  }
  else{
    int error = aposition - 2500;

    int motorSpeed = (int)(Kp * error + Kd * (error - lastError));
    lastError = error;

    rightMotorSpeed = baseSpeed - motorSpeed;
    leftMotorSpeed = baseSpeed + motorSpeed;

    if (rightMotorSpeed > maxSpeed ) rightMotorSpeed = maxSpeed; 
    if (leftMotorSpeed > maxSpeed ) leftMotorSpeed = maxSpeed; 
    if (rightMotorSpeed < -maxSpeed) rightMotorSpeed = -maxSpeed; 
    if (leftMotorSpeed < -maxSpeed) leftMotorSpeed = -maxSpeed; 

    motors(leftMotorSpeed,rightMotorSpeed);

    sta=ST_N;
    //Serial.println("motore");
  }

  if (Serial.available()>0){
    int c=Serial.read();
    if (c!='\n')
      riga[ps++]=c;
    else{
      riga[ps]=0;
      //Serial.print(riga);
      float f=atof(riga+1);
      ps=0;
      if (riga[0]=='I') {
        Ki=f; 
        Serial.println("\nPIDM");
        Serial.println(Kp);
        Serial.println(Ki);
        Serial.println(Kd);
        Serial.println(maxSpeed);

      }
      if (riga[0]=='D') {
        Kd=f;
        Serial.println("\nPIDM");
        Serial.println(Kp);
        Serial.println(Ki);
        Serial.println(Kd);
        Serial.println(maxSpeed);
      }
      if (riga[0]=='P') {
        Kp=f;
        Serial.println("\nPIDM");
        Serial.println(Kp);
        Serial.println(Ki);
        Serial.println(Kd);
        Serial.println(maxSpeed);
      }
      if (riga[0]=='M') {
        maxSpeed=(int)f;
        Serial.println("\nPIDM");
        Serial.println(Kp);
        Serial.println(Ki);
        Serial.println(Kd);
        Serial.println(maxSpeed);

      }
      if (riga[0]=='B') {
        baseSpeed=(int)f;
        Serial.println("\nPIDM");
        Serial.println(Kp);
        Serial.println(Ki);
        Serial.println(Kd);
        Serial.println(maxSpeed);
        Serial.println(baseSpeed);

      }      
    }        

  }
  stp=sta;
}

void mediaoCircolare(unsigned int val[]){
  //inserisco i valori nella matrice
  for(int col=0;col<6;col++){
    matrice[indM][col]= val[col];
  }
  indM++;
  //quando ho fatto le 6  righe riporto a 0 il contatore
  if(indM>=6){
    indM=0;
  }
  //variabili per la media dei valori
  int mediaRiga=0;
  int mediaColonna=0;
  int mediaTot[6];
  //media della matrice
  //int c=0;
  for(int valC=0;valC<6;valC++){
    mediaColonna = 0;
    for(int valR=0;valR<6;valR++ ){
      mediaColonna+=matrice[valR][valC];
    }
    mediaTot[valC]= (mediaColonna / 6) > soglia;
    //c++;
  }

  for(int z=0;z<6;z++){
    sensori[z] = mediaTot[z];
  }
}

void motors(int l, int r){
  set_motor(right_motor,r);
  set_motor(left_motor,l);
}

void readSensors(){
  unsigned int position = qtrrc.readLine(sensorValues);
  Serial.print(sensorValues[leftFar]);
  Serial.print("\t");
  Serial.print(sensorValues[leftCenter]);
  Serial.print("\t");
  Serial.print(sensorValues[leftNear]);
  Serial.print("\t");
  Serial.print(sensorValues[rightNear]);
  Serial.print("\t");
  Serial.print(sensorValues[rightCenter]);
  Serial.print("\t");
  Serial.println(sensorValues[rightFar]);

  lf = sensorValues[leftFar]>soglia;
  lc = sensorValues[leftCenter]>soglia;
  ln = sensorValues[leftNear]>soglia;
  rn = sensorValues[rightNear]>soglia;
  rc = sensorValues[rightCenter]>soglia;
  rf = sensorValues[rightFar]>soglia;
}

void set_motor(boolean motor, char speed) { // imposta la velocitÃ  tra -100 (indietro) e +100 (avanti)
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
void motor_speed(boolean motor, boolean direction, byte speed) { // imposta la velocitÃ  tra 0 e 255
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












