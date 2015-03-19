#include <QTRSensors.h>

/*
 * @author: alberto valente alb.valente@gmail.com
 *
 * Roboval 2012
 * Firmware di test per il sensore di linea pololu QTRRC8
 * 
 * per informazioni www.roboval.it
 *
 */

//--------------------------------------------------------------
//          parametri di configurazione hardcoded
//--------------------------------------------------------------
// i parametri seguenti descrivono l'interfacciamento del 
// microcontrollore con i sensori.
// Se modificati potrebbero compromettere il funzionamento del robot.

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
// i parametri seguenti possono essere modificati a piacimento
// per migliorare il comportamento del robot
//
#define soglia       200 // soglia di riconoscimento bianco/nero (0-1000)

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
void loop(){
  readSensorsDbg();
}

void readSensorsDbg(){
  unsigned int position = qtrrc.readLine(sensorValues);
  leftFarReading     = sensorValues[leftFar];
  leftCenterReading  = sensorValues[leftCenter];
  leftNearReading    = sensorValues[leftNear];
  rightNearReading   = sensorValues[rightNear];
  rightCenterReading = sensorValues[rightCenter];
  rightFarReading    = sensorValues[rightFar];

  Serial.print(leftFarReading);
  Serial.print(" ");
  Serial.print(leftCenterReading);
  Serial.print(" ");
  Serial.print(leftNearReading);
  Serial.print(" ");
  Serial.print(rightNearReading);
  Serial.print(" ");
  Serial.print(rightCenterReading);
  Serial.print(" ");
  Serial.println(rightFarReading);
  delay(1000);
}


