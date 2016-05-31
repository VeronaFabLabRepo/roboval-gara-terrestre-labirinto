

/*
* Squadra: MISSING
* Robot:   Pierino il robottino
* @autori: Lorenzetti Assaker
* @scuola: Aleardo Aleardi
* @data: Maggio 2016
* @versione: -
* @evento: Roboval 2016
* 
*/


#include <QTRSensors.h>
#define Kp 0.1
#define Kd 0.5
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



#define maxSpeed 55 // max speed of the robot

#define baseSpeed 69 // this is the speed at which the motors should spin when the robot is perfectly on the line


int stato;
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
  for (int i = 0; i < 400; i++){
    qtrrc.calibrate();
  }
  motor_standby(false);

}
int lastError = 0;


void loop()
{
  
  unsigned char found_left     = 0;
  unsigned char found_straight = 0;
  unsigned char found_right    = 0;
  unsigned int sensors[6];
  int aposition = qtrrc.readLine(sensors); 
  int error = aposition - 2500;
  Serial.print(aposition);
  Serial.print("\t");
  Serial.print(error);
  Serial.print("\t");
  
  
  
  int motorSpeed = (int)(Kp * error + Kd * (error - lastError));
  lastError = error;
 
  int rightMotorSpeed = baseSpeed - motorSpeed;
  int leftMotorSpeed = baseSpeed + motorSpeed;
  
  if (rightMotorSpeed > maxSpeed ) rightMotorSpeed = maxSpeed; 
  if (leftMotorSpeed > maxSpeed ) leftMotorSpeed = maxSpeed; 
  if (rightMotorSpeed < -maxSpeed) rightMotorSpeed = -maxSpeed; 
  if (leftMotorSpeed < -maxSpeed) leftMotorSpeed = -maxSpeed; 
  
  Serial.print("\t");
  Serial.print(leftMotorSpeed);
  Serial.print("\t");
  Serial.print(rightMotorSpeed);
  
  set_motor(right_motor,rightMotorSpeed);
  set_motor(left_motor,leftMotorSpeed);
  if(sensors[0] > 200) found_left = 1;
  if(sensors[5] > 200) found_right = 1;
 // delay(30); //non mi interessa nulla e vado avanti.
 // aposition = qtrrc.readLine(sensors); 
  
  Serial.print("\t");
  Serial.println(safemo(found_left,found_right,found_straight));
  /*
  if(sensors[2] > 200 || sensors[3] > 200) found_straight = 1;
  
  
  Serial.print("\t");
  Serial.println(describe(found_left,found_right,found_straight));
  
  */
  

}

char describe(char foundl,char foundr, char founds){
  byte cosa = (foundl<<2)|(foundr<<1)|(founds<<0);
  switch(cosa){
    case 0: return 'B';
    case 1: return '|';
    case 2: return '>';
    case 3: return 'R';
    case 4: return '<';
    case 5: return 'L';
    case 6: return 'T';
    case 7: return '+';
  }
}

char safemo(char foundl,char foundr, char founds){
    if(foundl){
      motors(50,50);
      delay(50);
      motors(-40,40);
      delay(250);
      return '<';
    }
    else if(founds){
        return '|';
    }
    else if(foundr){
      return '>';
    }
    else{
      
        return 'B';
    }
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















