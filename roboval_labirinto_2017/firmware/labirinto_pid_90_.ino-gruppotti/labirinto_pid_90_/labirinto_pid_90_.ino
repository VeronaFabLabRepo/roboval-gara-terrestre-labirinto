//Bonaldo Federico,Sasso Niccolò, Piludu Riccardo (LO GRUPPOTTI)
#include <QTRSensors.h>
#define leftFar          0
#define leftCenter       2
#define leftNear         1
#define rightNear        4
#define rightCenter      3
#define rightFar         5
#define NUM_SENSORS      6
#define TIMEOUT       2500
#define EMITTER_PIN   QTR_NO_EMITTER_PIN
#define out_STBY    7
#define out_B_PWM   10 //B motore destro
#define out_A_PWM   5  //A motore sinistro
#define out_A_IN2   6
#define out_A_IN1   4
#define out_B_IN1   8
#define out_B_IN2   9
#define left_motor  0
#define right_motor 1
float Kp = 0.07; // 0.3//2.5  0.85
float Ki = 0; //0.005*/
float Kd = 1.5;
#define rightMaxSpeed 90
#define leftMaxSpeed 90
#define rightBaseSpeed 80
#define leftBaseSpeed 80

#define DEB 0
#define DEB2 0

#define soglia       180 // soglia di riconoscimento bianco/nero (0-1000)

// dichiarazione variabili globali
int ES;
int S;
int CS;
int CD;
int D;
int ED;

int ps=0;
char riga[100];
int lastError = 0;
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

  motor_standby(false);

  Serial.begin(9600);
  Serial.println("calib");
  for (int i = 0; i < 400; i++){
    qtrrc.calibrate();
  }
  Serial.print("3..");
  delay(600);
  Serial.print("2..");
  delay(600);
  Serial.print("1..");
  delay(600);
  Serial.println("GO!");
  delay(600);



}


 unsigned int position;
void loop(){
  letturaSensori();
  direzioneEzBot();
#if DEB
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
        //Serial.println(maxSpeed);

      }
      if (riga[0]=='D') {
        Kd=f;
        Serial.println("\nPIDM");
        Serial.println(Kp);
        Serial.println(Ki);
        Serial.println(Kd);
        //Serial.println(maxSpeed);
      }
      if (riga[0]=='P') {
        Kp=f;
        Serial.println("\nPIDM");
        Serial.println(Kp);
        Serial.println(Ki);
        Serial.println(Kd);
        //Serial.println(maxSpeed);
      }   
    }        

  }
  //delay(1000);
  #endif
}


void letturaSensori(){
  position= qtrrc.readLine(sensorValues);
  ES  = sensorValues[leftFar]>soglia ?1 :0;
  S  = sensorValues[leftNear]>soglia ?1 :0;
  CS = sensorValues[leftCenter]>soglia ?1 :0;
  CD = sensorValues[rightCenter]>soglia ?1 :0;
  D  = sensorValues[rightNear]>soglia ?1 :0;
  ED = sensorValues[rightFar]>soglia ?1 :0;

#if DEB2
  Serial.print(ES);
  Serial.print(" ");
  Serial.print(S);
  Serial.print(" ");
  Serial.print(CS);
  Serial.print(" ");
  Serial.print(CD);
  Serial.print(" ");
  Serial.print(D);
  Serial.print(" ");
  Serial.print(ED);
  Serial.println();
#endif

}


void direzioneEzBot(){

  
  if(CD == 1 && D == 1 &&ED == 1){
    letturaSensori();
    delay(200);
    
    while(ED == 1){
      set_motor(left_motor, 70);
      set_motor(right_motor, -40);
      letturaSensori();
    }
  }

  else if(ES == 1 && CD == 1 && CS == 1 && ED == 1){
    //delay(270);
    set_motor(left_motor, 40);
    set_motor(right_motor, -60);
    letturaSensori();
  }

  
  else if(ES = 1 && S == 1 && CS == 1){
    while(CD == 1 && CS == 1){
      pid();
      letturaSensori();
    }
    letturaSensori();
    if(CD == 0 && CS == 0){
      letturaSensori();
      while(CS == 0 && CD == 0){
        set_motor(left_motor, -40);
        set_motor(right_motor, 60);
        letturaSensori();
      }
    }
  }
  
  else if(ES == 0 && ED == 0 && CS == 0 && CD == 0){
    letturaSensori();
    while(CD == 0 && CS == 0){
      set_motor(left_motor, 60);
      set_motor(right_motor, -60);
      letturaSensori();
    }
  }
  else{
    pid();
  }
}

void set_motor(boolean motor, int speed) { // imposta la velocità tra -100 (indietro) e +100 (avanti)
  int PWMvalue=0;
  PWMvalue = map(abs(speed),0,100,50,255);
  if (speed > 0)
    motor_speed(motor,0,PWMvalue);
  else if (speed < 0)
    motor_speed(motor,1,PWMvalue);
}

void motor_speed(boolean motor, boolean direction, int speed) { // imposta la velocità tra 0 e 255
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

void pid(){

  unsigned int sensors[6];
  int position = qtrrc.readLine(sensors); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
  int error = position - 2500;
  //Serial.println(position);
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;
  
  int rightMotorSpeed = rightBaseSpeed - motorSpeed;
  int leftMotorSpeed = leftBaseSpeed + motorSpeed;
  
  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < -rightMaxSpeed) rightMotorSpeed -rightMaxSpeed; // keep the motor speed positive
  if (leftMotorSpeed < -leftMaxSpeed) leftMotorSpeed = -leftMaxSpeed; // keep the motor speed positive

   set_motor(right_motor,rightMotorSpeed);
   set_motor(left_motor,leftMotorSpeed);
}
