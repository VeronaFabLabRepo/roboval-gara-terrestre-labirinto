//#####################################################
//            Firmware Robot Easy by BBM             //
//#####################################################
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//[    Alunni della 3AI dell' I.T.I.S. G.Marconi      ]
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//#--@ Bussola Riccardo                              *#
//#--@ Biondani Mattia                               *#
//#--@ Montagna Federico                             *#
//#--@ Giacopuzzi Marco                              *#
//#--@ Victor Annunziata                             *#
//#--@ Lovato Luca                                   *#
//#--@ Valdo Riccardo                                *#
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//#####################################################
/*
------------------------------------------------------------------------------
          _____                    _____                    _____
         /\    \                  /\    \                  /\    \
        /::\    \                /::\    \                /::\____\
       /::::\    \              /::::\    \              /::::|   |
      /::::::\    \            /::::::\    \            /:::::|   |
     /:::/\:::\    \          /:::/\:::\    \          /::::::|   |
    /:::/__\:::\    \        /:::/__\:::\    \        /:::/|::|   |
   /::::\   \:::\    \      /::::\   \:::\    \      /:::/ |::|   |
  /::::::\   \:::\    \    /::::::\   \:::\    \    /:::/  |::|___|______
 /:::/\:::\   \:::\ ___\  /:::/\:::\   \:::\ ___\  /:::/   |::::::::\    \
/:::/__\:::\   \:::|    |/:::/__\:::\   \:::|    |/:::/    |:::::::::\____\
\:::\   \:::\  /:::|____|\:::\   \:::\  /:::|____|\::/    / ~~~~~/:::/    /
 \:::\   \:::\/:::/    /  \:::\   \:::\/:::/    /  \/____/      /:::/    /
  \:::\   \::::::/    /    \:::\   \::::::/    /               /:::/    /
   \:::\   \::::/    /      \:::\   \::::/    /               /:::/    /
    \:::\  /:::/    /        \:::\  /:::/    /               /:::/    /
     \:::\/:::/    /          \:::\/:::/    /               /:::/    /
      \::::::/    /            \::::::/    /               /:::/    /
       \::::/    /              \::::/    /               /:::/    /
        \::/    /                \::/    /                \::/    /
         \/____/                  \/____/                  \/____/

------------------------------------------------LABIRINTO VERSION------------

*/
#include <QTRSensors.h>
float kp = 0.07;
float ki = 0;
float kd = 1.5;

#define out_STBY    7
#define out_B_PWM   10 //B motore destro
#define out_A_PWM   5  //A motore sinistro
#define out_A_IN2   6
#define out_A_IN1   4
#define out_B_IN1   8
#define out_B_IN2   9
#define left_motor  0
#define right_motor 1

#define sensor_lf 0 //leftFar
#define sensor_lc 1 //leftCenter
#define sensor_ln 2 //leftNear
#define sensor_rn 3 //rightNear
#define sensor_rc 4 //rightCnter
#define sensor_rf 5 //rightFar
#define num_sensors 6
#define TIMEOUT 2500
#define emitter_pin QTR_NO_EMITTER_PIN

#define debfull 0
#define soglia 180

#define maxsp 100
#define minsp 25

#define soglia 200
#define sog_st 10

int maxSpeed=100; //velocità massima di default 100
int baseSpeed=75; //velocià di base quando si trova al centro
int curvatura=60;

int lc,ln,lf,rc,rn,rf;
int DEBUG = 0;
int lastError = 0;
int I = 0;
int time = 0;
int rightMotorSpeed;
int leftMotorSpeed;
int scarto=200;

QTRSensorsRC qtrrc((unsigned char[]) {
  A0, A1, A2, A3, A4, A5}
,num_sensors,TIMEOUT,emitter_pin);
unsigned int sensorValues[num_sensors];

int history[6][6];
int indice_h = 0;
int sensori[6];
int ps = 0;
char riga[100];
unsigned long lastMs,currMs;
int num_camp = 0;

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
      history[r][c]=0;
    }
  }

  num_camp=0;
  lastMs=millis();
}

//definizione degli stati di lettura
#define st_n 0
#define st_t 1
#define st_dx 2
#define st_sx 3
#define st_vu 4

int last_st = st_vu; //memorizazione dello stato precedente
int st = st_n;  //memorizazione dello stato attuale

void loop() {
  st = st_n;

  num_camp++;
  currMs=millis();
  if(currMs-lastMs >= 1000){
    Serial.print("NC");
    Serial.println(num_camp);
    lastMs = currMs;
    num_camp = 0;

  }

  unsigned int sensors[6];
  boolean sen[6];
  int apposition = qtrrc.readLine(sensors);
  mediaCircolare(sensors);
  // Implementazione dell'Algoritmo PID
  int val_read = apposition;

   if( sensori[0] && sensori[1] && sensori[4] && sensori[5]){ // leggo una T
     st=st_t;
   }
   else if(!sensori[0]&& sensori[3] && sensori[4] && sensori[5]){ //destra
     st=st_dx;
   }
   else if(!sensori[5]&& sensori[0] && sensori[1] && sensori[2]){ //sinistra
     st=st_sx;
   }
   else if(!(sensori[0]&& sensori[1] && sensori[2] && sensori[3] && sensori[4] && sensori[5])){ //vuoto
     st=st_vu;
   }
/*
    if(st ==st_dx) Serial.println("D");
    if(st ==st_sx) Serial.println("S");
    if(st == st_t) Serial.println("T");
    if(st == st_vu) Serial.println("V");
*/

    if(st == st_dx || st == st_t){ //destra e T
        motors(curvatura,-curvatura);
        delay(20);
      }
      else if(st  == st_vu && last_st == st_sx ){ //sinistra
        motors(-curvatura,curvatura);
        delay(20);
      }
      else{
        int error = apposition - 2500;

        int motorSpeed = (int)(kp * error + kd *(error - lastError));
        lastError = error;

        rightMotorSpeed = baseSpeed - motorSpeed;
        leftMotorSpeed = baseSpeed + motorSpeed;

        if (rightMotorSpeed > maxSpeed ) rightMotorSpeed = maxSpeed;
        if (leftMotorSpeed > maxSpeed ) leftMotorSpeed = maxSpeed;
        if (rightMotorSpeed < -maxSpeed ) rightMotorSpeed = -maxSpeed;
        if (leftMotorSpeed < -maxSpeed ) leftMotorSpeed = -maxSpeed;

        motors(leftMotorSpeed,rightMotorSpeed);
        st = st_n;
      }

  last_st = st;

Serial.print(last_st);
Serial.print(" ");
Serial.println(st);

}

void mediaCircolare(unsigned int val[]){
  //inserisco i valori nella matrice
  for(int col=0;col<6;col++){
    history[indice_h][col]= val[col];
  }
  indice_h++;
  //quando ho fatto le 6  righe riporto a 0 il contatore
  if(indice_h>=6){
    indice_h=0;
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
      mediaColonna+=history[valR][valC];
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
  lf = sensorValues[sensor_lf]>soglia;
  lc = sensorValues[sensor_lc]>soglia;
  ln = sensorValues[sensor_ln]>soglia;
  rn = sensorValues[sensor_rn]>soglia;
  rc = sensorValues[sensor_rc]>soglia;
  rf = sensorValues[sensor_rf]>soglia;
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
