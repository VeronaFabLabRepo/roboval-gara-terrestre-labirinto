#include <QTRSensors.h>
#define Kp 0.1
#define Kd 1.0
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

int maxSpeed=100; //di default 100 max speed of the robot

#define baseSpeed 50 //di default 50 this is the speed at which the motors should spin when the robot is perfectly on the line

//versione 2
int stato,sp;
int scu;  // stati consecutivi uguali
#define SOG_ST 10
#define ST_N  0
#define ST_CD 1
#define ST_CS 2
#define ST_T 3
#define ST_V 4


int leftCenterReading;
int leftNearReading;
int leftFarReading;
int rightCenterReading;
int rightNearReading;
int rightFarReading;
int lastDebugTime = 0;
int lastError = 0;

int time=0;
int rightMotorSpeed;
int leftMotorSpeed;
int s1;
int s2;

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
  for (int i = 0; i < 400; i++){
    qtrrc.calibrate();
  }
  Serial.print("finio");
  motor_standby(false);
  /* gestione stato per evitare i rumori*/
  stato=0;
  sp=1000;
  scu=0;

}



void loop()
{

  boolean cd= false; //curva a destra
  boolean cs = false;//curva a sinistra
  boolean t = false;//e t
  boolean v= false;//vuoto

  unsigned int sensors[6];
  boolean sen[6];
  int aposition = qtrrc.readLine(sensors); 
  int error = aposition - 2500;
  for(int i=0;i<6;i++){
    sen[i]=sensors[i]>SOGLIA;
  }
  time++;

  if (time%100==0) Serial.println("+"); 
#if DEBFULL  
  Serial.print(aposition);
  Serial.print("\t");
  Serial.print(error);
  Serial.print("\t");
#endif  


  int motorSpeed = (int)(Kp * error + Kd * (error - lastError));
  lastError = error;

  rightMotorSpeed = baseSpeed - motorSpeed;
  leftMotorSpeed = baseSpeed + motorSpeed;

  if (rightMotorSpeed > maxSpeed ) rightMotorSpeed = maxSpeed; 
  if (leftMotorSpeed > maxSpeed ) leftMotorSpeed = maxSpeed; 
  if (rightMotorSpeed < -maxSpeed) rightMotorSpeed = -maxSpeed; 
  if (leftMotorSpeed < -maxSpeed) leftMotorSpeed = -maxSpeed; 
#if DEBFULL  
  Serial.print("\t");
  Serial.print(leftMotorSpeed);
  Serial.print("\t");
  Serial.print(rightMotorSpeed);
#endif
/*
  set_motor(right_motor,rightMotorSpeed);
  set_motor(left_motor,leftMotorSpeed);*/
  
  motors(leftMotorSpeed,rightMotorSpeed);
  
  stato=ST_N;
  if (sen[4] && sen[5] && !sen[0])
    stato=ST_CD;
  if (sen[0] && sen[1] && !sen[5])
    stato=ST_CS;
  if (sen[4] && sen[5] && sen[1] && sen[0])
    stato=ST_T;
  if (!sen[4] && !sen[5] && !sen[0] && !sen[3] && !sen[2] && !sen[1])
    stato=ST_V;
    
  if (stato!=ST_N && maxSpeed > MINSP)
    maxSpeed=maxSpeed*95/100;  // decelera 5%
   
  if (stato==ST_N && maxSpeed < MAXSP)
      maxSpeed=maxSpeed*105/100;  // decelera 5%
    
    
  if (stato==sp)
    scu++;
  else
    scu=0;
  sp=stato;

  // delay(30); //non mi interessa nulla e vado avanti.
  // aposition = qtrrc.readLine(sensors); 
  if(scu >SOG_ST){
    scu=0;
    if (stato==ST_CS) {
      Serial.println("CS  ");
      sx();
    }

    if (stato==ST_CD) {
      Serial.println("CD  ");
      dx();      
    }
    if (stato==ST_T) {
      Serial.println("T  ");
      dx();
    }
    if (stato==ST_V) {
      Serial.println("V  ");
      vuoto();
    }
    
    
    
    
    
    s2=s1;  // gestione della coda degli stati
    s1=stato;
  }
}
void dx(){
    //maxSpeed=30;
    motors(-20,-20);
    delay(20);  
    leftMotorSpeed = maxSpeed;
    rightMotorSpeed = -maxSpeed/2;
    motors(leftMotorSpeed,rightMotorSpeed);
    delay(300);
}
void sx(){
    //maxSpeed=30;
    motors(-20,-20);
    delay(20);  

    leftMotorSpeed = -maxSpeed/2;
    rightMotorSpeed = maxSpeed;
    motors(leftMotorSpeed,rightMotorSpeed);
    delay(300);
}
void vuoto(){
    //maxSpeed=30;
    motors(-20,-20);
    delay(20);  

    leftMotorSpeed = maxSpeed/2;
    rightMotorSpeed = -maxSpeed/2;
    motors(leftMotorSpeed,rightMotorSpeed);
    delay(350);
}


char safemo(char foundl,char foundr, char founds){
  if(foundl){
    motors(50,50);
    delay(50);
    motors(-50,50);
    delay(250);
    return '<';
  }/*
    else if(founds){
   return '|';
   }
   else if(foundr){
   return '>';
   }
   else{
   
   return 'B';
   }*/
}

void motors(int l, int r){
  set_motor(right_motor,r);
  set_motor(left_motor,l);
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


