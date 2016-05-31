
/*
 * @scuola:	ITIS Marconi
 * @autori:	Cacciatori,Gorizio,La Manna,Ligozzi (4bi)
 * Squadra: i protectron
 * Robot:   r2-d2
 * @data:   Maggio 2016
 * Versione: - 
 * @evento: Roboval 2016
 * 
 */


#include <QTRSensors.h>

#define Kp 0.1
#define Kd 2 //0.85
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
#define correction_speed   100 // velocita' di correzione traiettoria rettilinea (0-100)
#define soglia             200 // soglia di riconoscimento bianco/nero (0-1000)

int maxSpeed=100; //di default 100 max speed of the robot

#define baseSpeed 80 //di default 50 this is the speed at which the motors should spin when the robot is perfectly on the line




int leftCenterReading;
int leftNearReading;
int leftFarReading;
int rightCenterReading;
int rightNearReading;
int rightFarReading;
int lastDebugTime = 0;
int lastError = 0;

int rightMotorSpeed;
int leftMotorSpeed;


QTRSensorsRC qtrrc((unsigned char[]){    A0, A1, A2, A3, A4, A5},NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];


void setup()
{
    Serial.begin(9600);
    pinMode(out_STBY,OUTPUT);
    pinMode(out_A_PWM,OUTPUT);
    pinMode(out_A_IN1,OUTPUT);
    pinMode(out_A_IN2,OUTPUT);
    pinMode(out_B_PWM,OUTPUT);
    pinMode(out_B_IN1,OUTPUT);
    pinMode(out_B_IN2,OUTPUT);
    for (int i = 0; i < 400; i++)
    {
        qtrrc.calibrate();
    }
    Serial.println("inizio ");
    motor_standby(false);



}

void gestSensors_old(unsigned int sensorValues[])
{

    leftFarReading     = sensorValues[leftFar]>soglia;
    leftCenterReading  = sensorValues[leftCenter]>soglia;
    leftNearReading    = sensorValues[leftNear]>soglia;
    rightNearReading   = sensorValues[rightNear]>soglia;
    rightCenterReading = sensorValues[rightCenter]>soglia;
    rightFarReading    = sensorValues[rightFar]>soglia;
}

#define DIM 6
unsigned int bufv[DIM][6];
int rc=0; 


// la gestione dei sensori è fatta con la media delle ultime DIM letture
void gestSensors()
{
    int s,i;
    for(s=0,i=0; i<DIM; i++)
        s=s+bufv[i][leftFar];
    leftFarReading     = (s/DIM) >soglia;


    for(s=0,i=0; i<DIM; i++)
        s=s+bufv[i][leftCenter];
    leftCenterReading  = (s/DIM) >soglia;

    for(s=0,i=0; i<DIM; i++)
        s=s+bufv[i][leftNear];
    leftNearReading  = (s/DIM) >soglia;

    for(s=0,i=0; i<DIM; i++)
        s=s+bufv[i][rightNear];
    rightNearReading  = (s/DIM) >soglia;

    for(s=0,i=0; i<DIM; i++)
        s=s+bufv[i][rightCenter];
    rightCenterReading  = (s/DIM) >soglia;

    for(s=0,i=0; i<DIM; i++)
        s=s+bufv[i][rightFar];
    rightFarReading  = (s/DIM) >soglia;
}
void dump_sensors(){
  Serial.print(leftFarReading);
  Serial.print(leftCenterReading);
  Serial.print(leftNearReading);
  Serial.print(rightNearReading);
  Serial.print(rightCenterReading);
  Serial.println(rightFarReading);
  
}

int statoP=0;   /* stato precedente
    0 NORMALE
    1 DX
    2 T
    3 SX
    4 V
    


*/

void loop()
{
// gestione cont rilevazione
    unsigned int sensors[6];
    boolean sen[6];
    int aposition = qtrrc.readLine(sensors);
    // memorizza i valori grezzi in una matrice di DIM righe
    for (int k=0; k<6; k++)
        bufv[rc][k]=sensors[k];

    //gestione matrice come buffer circolare
    rc++;
    if (rc==DIM)
        rc=0;

    // gestione binaria dei sensori
    // ogni sensore viene visto come media deglle ultime dim letture
    gestSensors();
   
/*   gestSensors_old(sensors);
*/

// gestione delle curve)
    int error = aposition - 2500;

    //  dump_sensors();
    
    
//#if 0    //abilitazione curve
    //0**111
    if(!leftFarReading && !leftCenterReading && leftNearReading && rightNearReading && rightCenterReading && rightFarReading)
    {
      Serial.println("D");
       dx();
       statoP=1;  
    }
    //111**0
    if(leftFarReading && leftCenterReading && leftNearReading && rightNearReading && !rightCenterReading && !rightFarReading)
    {
      
    Serial.println("S");
    //    sx();
        error=0;  // inganna il PID
        statoP=3;
    }
    //111111
    if(leftFarReading && leftCenterReading && leftNearReading && rightNearReading && rightCenterReading && rightFarReading)
    {
      Serial.println("T");
        dx();
        statoP=2;
    }
    
    //111110
    if(leftFarReading && leftCenterReading && leftNearReading && rightNearReading && rightCenterReading && !rightFarReading)
    {
      Serial.println("T");
        dx();
        statoP=2;
    }
    
    //011111
    if(!leftFarReading && leftCenterReading && leftNearReading && rightNearReading && rightCenterReading && rightFarReading)
    {
      Serial.println("T");
        dx();
        statoP=2;
    }
    
    
    //000000
    /*if(!leftFarReading && !leftCenterReading && !leftNearReading && !rightNearReading && !rightCenterReading && !rightFarReading)
    {
       Serial.println("V");
       if (statoP==3){ // se imm prima c'era sx allora sx
        sx();
       }
       else{
        dx();
        dx();
       } 
       statoP=4;
    }*/

//#endif

    int motorSpeed = (int)(Kp * error + Kd * (error - lastError));
    lastError = error;

    rightMotorSpeed = baseSpeed - motorSpeed;
    leftMotorSpeed = baseSpeed + motorSpeed;

    if (rightMotorSpeed > maxSpeed ) rightMotorSpeed = maxSpeed;
    if (leftMotorSpeed > maxSpeed ) leftMotorSpeed = maxSpeed;
    if (rightMotorSpeed < -maxSpeed) rightMotorSpeed = -maxSpeed;
    if (leftMotorSpeed < -maxSpeed) leftMotorSpeed = -maxSpeed;

    /*
      set_motor(right_motor,rightMotorSpeed);
      set_motor(left_motor,leftMotorSpeed);*/

    motors(leftMotorSpeed,rightMotorSpeed);
}

void straight()
{
    if(!leftNearReading) // 0x01x0 il robot e' troppo a sinistra, deve andare piu' a destra quindi alzo il motore sinistro
    {
        set_motor(left_motor,  maxSpeed);
        set_motor(right_motor, correction_speed);
        return;
    }
    else if(!rightNearReading) // 0x10x0 il robot e' troppo a destra, deve andare piu' a sinistra quindi alzo il motore destro
    {
        set_motor(left_motor, correction_speed);
        set_motor(right_motor, maxSpeed);
        return;
    }
    else  // 0x11x0 il robot e' sopra la linea
    {
        set_motor(right_motor,  maxSpeed);
        set_motor(left_motor,  maxSpeed);
    }
}


// gestione rotazione a dx

void dx()
{
    //maxSpeed=30;
    motors(60,-60);
    delay(80); // prima 120
//    leftMotorSpeed = maxSpeed;
//    rightMotorSpeed = -maxSpeed/2;
//    motors(maxSpeed,-maxSpeed/2);
//    delay(300);
}
void sx()
{
    //maxSpeed=30;
    motors(-60,60);
    delay(80);

//    leftMotorSpeed = -maxSpeed/2;
//    rightMotorSpeed = maxSpeed;
//    motors(-maxSpeed/2,maxSpeed);
//    delay(300);
}
void vuoto()
{
    //maxSpeed=30;
    motors(-20,-20);
    delay(20);

    leftMotorSpeed = maxSpeed/2;
    rightMotorSpeed = -maxSpeed/2;
    motors(leftMotorSpeed,rightMotorSpeed);
    delay(350);
}

void motors(int l, int r)
{
    set_motor(right_motor,r);
    set_motor(left_motor,l);
}
void set_motor(boolean motor, char speed)   // imposta la velocitÃ  tra -100 (indietro) e +100 (avanti)
{
    byte PWMvalue=0;
    PWMvalue = map(abs(speed),0,100,50,255);
    if (speed > 0)
        motor_speed(motor,0,PWMvalue);
    else if (speed < 0)
        motor_speed(motor,1,PWMvalue);
    else
    {
        motor_coast(motor);
    }
}
void motor_speed(boolean motor, boolean direction, byte speed)   // imposta la velocitÃ  tra 0 e 255
{
    if (motor == left_motor)
    {
        if (direction == 0)
        {
            digitalWrite(out_A_IN1,HIGH);
            digitalWrite(out_A_IN2,LOW);
        }
        else
        {
            digitalWrite(out_A_IN1,LOW);
            digitalWrite(out_A_IN2,HIGH);
        }
        analogWrite(out_A_PWM,speed);
    }
    else
    {
        if (direction == 0)
        {
            digitalWrite(out_B_IN1,HIGH);
            digitalWrite(out_B_IN2,LOW);
        }
        else
        {
            digitalWrite(out_B_IN1,LOW);
            digitalWrite(out_B_IN2,HIGH);
        }
        analogWrite(out_B_PWM,speed);
    }
}
void motor_standby(boolean state)   // abilita/disabilita i motori
{
    if (state == true)
        digitalWrite(out_STBY,LOW);
    else
        digitalWrite(out_STBY,HIGH);
}

void motor_coast(boolean motor)   // motore in folle
{
    if (motor == left_motor)
    {
        digitalWrite(out_A_IN1,LOW);
        digitalWrite(out_A_IN2,LOW);
        digitalWrite(out_A_PWM,HIGH);
    }
    else
    {
        digitalWrite(out_B_IN1,LOW);
        digitalWrite(out_B_IN2,LOW);
        digitalWrite(out_B_PWM,HIGH);
    }
}



