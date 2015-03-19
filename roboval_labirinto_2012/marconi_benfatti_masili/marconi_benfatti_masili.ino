/*
 * MARCONI ROBOCUP TEAM
 * Il programma:
 *   La logica di questo programma non è complessa: 
 *   legge l'input (e lo media con gli ultimi 10 input)
 *   reagisce, tenendo conto, alcune volte, di quello che stava
 *   facendo prima (ex: se trovo bianco mentre correggo verso destra, gira a destra)
 *    
 *   Come algoritmo risolutivo del labirinto usa quello della mano sinistra.
 *   
 *
 * Il team:
 *   Il team è composto da Andrea Benfatti e Andrea Masili
 *   della 4Bi Informatica dell' ITIS G. Marconi di Verona.
 *   Dopo l'esperienza della Robocup abbiamo partecipato anche alla Roboval
 *   con questo programma.
 *   
 *    
 *
 *
 *
 */


#include <QTRSensors.h>


#define out_STBY    5
#define out_B_PWM   0 //B motore destro
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

#define full_speed            68          //38 def
#define correction_speed      45          //25
#define turn_speed            35 
#define debugPeriod           1000 
#define soglia                250 
#define attesaPerManovra      50 
#define nearFullSpeed         20
#define plus_speed            40
#define brake_speed           05

#define D00 1       //Quando il robottino va dritto.
#define R00 2
#define D01 3
#define D02 4
#define R01 5
#define B00 6

#define NCAMP 15

int camp[NCAMP][6];
int val[6];
byte j,i,k=0;

int stato;
int lc, ln, lf,rc, rn, rf; 
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
  Serial.begin(9600);
  pinMode(out_STBY,OUTPUT);
  pinMode(out_A_PWM,OUTPUT);
  pinMode(out_A_IN1,OUTPUT);
  pinMode(out_A_IN2,OUTPUT);
  pinMode(out_B_PWM,OUTPUT);
  pinMode(out_B_IN1,OUTPUT);
  pinMode(out_B_IN2,OUTPUT);

  Serial.println("calibra");

  for (int i = 0; i < 400; i++){
    qtrrc.calibrate();
  }
  
  Serial.println("fine calibra");

  motor_standby(false);
  stato=D00;  
}

long unsigned int time=0;

void loop(){
 readSensors();
 if(lf && lc && ln){   //Nel caso legga nero su tre sensori a sinistra, curva a sinistra tenendo ferma una ruota
   boolean trov= false;  
   giraSx(); 
   delay(350);
   while(!trov){
     readSensors();
     giraSx(); 
     if(!lf && (ln || rc) && !rf){
       trov = true;
     }
   }
 }else if(lf || lc){ //se legge nero su uno dei due sensori più a sinistra corregge a sinistra.
   corrSx();
   stato=0;
 }else if(rf || rc){ //se legge nero su uno dei due sensori più a destra corregge a destra.
   corrDx();
   stato=1;
 }else if(!lf && !lc && !ln && !rn && !rc && !rf){ //Se legge bianco su tutti i sensori, gira a sinistra se correggeva a sinistra, gira a destra in ogni caso fino a che non trova la linea nera un'altra volta. 
   if(stato==0){
     boolean trov=false;
     while(!trov){
       readSensors();
       if(ln || rn){
         trov = true;
       }else{
         turnSx();
       }
     }
   }else{
     boolean trov=false;
     while(!trov){
       readSensors();
       if(ln || rn){
         trov = true;
       }else{
         turnDx();
       }
     }
   }
 }else if(ln || rn){ //Se legge nero nei due sensori centrali va avanti dritto.
   avanti();
 }
}
//Controlli motore
void avanti(){
  set_motor(left_motor, full_speed);
  set_motor(right_motor, full_speed);
}
void corrDx(){
  set_motor(left_motor, full_speed);
  set_motor(right_motor, correction_speed);
}
void corrSx(){
  set_motor(left_motor, correction_speed);
  set_motor(right_motor, full_speed);
}
void turnDx(){
  set_motor(left_motor, turn_speed);
  set_motor(right_motor, -turn_speed);
}
void turnSx(){
  set_motor(left_motor, -turn_speed);
  set_motor(right_motor, turn_speed);
}
void giraSx(){
  set_motor(left_motor, -brake_speed);
  set_motor(right_motor, turn_speed);
}

void giraDx(){
  set_motor(right_motor, -brake_speed);
  set_motor(left_motor, turn_speed);
}


// Lettura Sensori
void readSensors(){
  unsigned int position = qtrrc.readLine(sensorValues);	
  for (i=0;i<6;i++) {
    camp[k][i]=sensorValues[i];
  }
  k=(k+1)%NCAMP;
  for (i=0;i<6;i++){
    val[i]=0;
    for (j=0;j<NCAMP;j++)
      val[i]+=camp[j][i];
    val[i]/=NCAMP;
  }
  // fa la media tra gli ultimi NCAMP lettura
  
  leftFarReading     = val[leftFar]>soglia;
  leftCenterReading  = val[leftCenter]>soglia;
  leftNearReading    = val[leftNear]>soglia;
  rightNearReading   = val[rightNear]>soglia;
  rightCenterReading = val[rightCenter]>soglia;
  rightFarReading    = val[rightFar]>soglia;
  
  lf = leftFarReading;
  lc = leftCenterReading;
  ln = leftNearReading;
  
  rn = rightNearReading;
  rc = rightCenterReading;
  rf = rightFarReading;
}

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
