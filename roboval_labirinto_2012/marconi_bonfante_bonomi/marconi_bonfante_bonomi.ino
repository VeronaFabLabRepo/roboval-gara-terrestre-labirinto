#include <QTRSensors.h>
#include <avr/pgmspace.h>
/*
 * @author: margherita bonfante
 * @author: viola bonomi
 * @school: ITIS G.Marconi Verona
 *
 * per informazioni www.roboval.it
 *
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
#define TIMEOUT       2500
#define EMITTER_PIN   QTR_NO_EMITTER_PIN

//--------------------------------------------------------------
//          parametri di configurazione modificabili
//--------------------------------------------------------------
// i parametri seguenti possono essere modificati a piacimento
// per migliorare il comportamento del robot
//

#define full_speed          30  // velocita' massima (0-100)
#define FS  full_speed
#define correction_speed     8+8 // velocita' di correzione traiettoria rettilinea (0-100)
#define CSN  (FS/2)
#define CSM  (FS/4)
#define turn_speed          30+8 // velocita' di curvatura (0-100)
#define TS turn_speed
#define debugPeriod       1000 // ogni quanti millisecondi inviare i messaggi di debug
#define soglia             250 // soglia di riconoscimento bianco/nero (0-1000)
#define attesaPerManovra    50 // tempo di attesa tra il riconoscimento della curva e il comando ai motori
#define nearFullSpeed       20+8
#define plus_speed          40+8
#define R                -20
#define A                50
//Stati di avanzamento.
#define D00 0       //descr Quando il robottino va dritto.
#define D01 1
#define D02 2
#define D03 3
#define D04 4
#define D05 5
#define D06 6
#define D07 7
#define R00 8
#define R01 9
#define R02 10
#define D08 11
#define F1 12


// durata dello stato 
#define DT  0
#define DC DT
#define DF DT
#define DR 40 



//--------------------------------------------------------------
//   tabella degli stati
//--------------------------------------------------------------
// ogni cella della matrice contiene
// stato successivo, mot sx, mot dx, durata 

// velocita motori FS   CSN  (corr min) CSM  (massima)
PROGMEM  prog_char mat [][11][4]= {
/* ingressi     110000                011000              001100               000110                000011                000000               111111               001111                111100                100000               000001 */
/*stato 0*/{{D03,CSM,FS,DT},     {D01,CSN,FS,DT},     {D00,FS,FS,DT},      {D02,FS,CSN,DT},      {D04,FS,CSM,DT},      {R00,FS,-FS,DT},      {F1,FS,R,DR},        {D05,R,FS,DC},        {R02,R,FS,DC},        {D06,R,FS,DT},       {D07,FS,R,DT} },
/*stato 1*/{{D03,CSM,FS,DT},     {D01,CSN,FS,DT},     {D00,FS,FS,DT},      {D02,FS,CSN,DT},      {D04,FS,CSM,DT},      {R00,FS,-FS,DT},      {F1,FS,R,DR},        {D05,R,FS,DC},        {R02,R,FS,DC},        {D06,R,FS,DT},       {D07,FS,R,DT} },
/*stato 2*/{{D03,CSM,FS,DT},     {D01,CSN,FS,DT},     {D00,FS,FS,DT},      {D02,FS,CSN,DT},      {D04,FS,CSM,DT},      {R00,FS,-FS,DT},      {F1,FS,R,DR},        {D05,R,FS,DC},        {R02,R,FS,DC},        {D06,R,FS,DT},       {D07,FS,R,DT} },
/*stato 3*/{{D03,CSM,FS,DT},     {D01,CSN,FS,DT},     {D00,FS,FS,DT},      {D02,FS,CSN,DT},      {D04,FS,CSM,DT},      {D03,FS,-FS,DT},      {F1,FS,R,DR},        {D05,R,FS,DC},        {R02,R,FS,DC},        {D06,R,FS,DT},       {D07,FS,R,DT} },
/*stato 4*/{{D03,CSM,FS,DT},     {D01,CSN,FS,DT},     {D00,FS,FS,DT},      {D02,FS,CSN,DT},      {D04,FS,CSM,DT},      {D04,-FS,FS,DT},      {F1,FS,R,DR},        {D05,R,FS,DC},        {R02,R,FS,DC},        {D06,R,FS,DT},       {D07,FS,R,DT} },
/*stato 5*/{{D05,FS,R,DC},       {D01,CSN,FS,DC},     {D00,FS,FS,DC},      {D02,FS,CSN,DC},      {D05,FS,R,DC},        {D05,FS,R,DC},        {D05,FS,R,DR},       {D05,FS,R,DC},        {D05,FS,R,DC},        {D05,FS,R,DC},       {D05,FS,R,DC} },
/*stato 6*/{{D03,CSM,FS,DT},     {D01,CSN,FS,DT},     {D00,FS,FS,DT},      {D02,FS,CSN,DT},      {D04,FS,CSM,DT},      {R00,FS,-FS,DT},      {F1,FS,R,DR},        {D05,R,FS,DC},        {R02,R,FS,DC},        {D06,R,FS,DT},       {D07,FS,R,DT} },
/*stato 7*/{{D03,CSM,FS,DT},     {D01,CSN,FS,DT},     {D00,FS,FS,DT},      {D02,FS,CSN,DT},      {D04,FS,CSM,DT},      {R00,-FS,FS,DT},      {F1,FS,R,DR},        {D05,R,FS,DC},        {R02,R,FS,DC},        {D06,R,FS,DT},       {D07,FS,R,DT} },
/*stato 8*/{{R00,FS,-FS,DT},     {D01,CSN,FS,DT},     {D00,FS,FS,DT},      {D02,FS,CSN,DT},      {R00,FS,-FS,DT},      {R00,FS,-FS,DT},      {R00,FS,-FS,DT},     {R00,FS,-FS,DT},      {R00,FS,-FS,DT},      {R00,FS,-FS,DT},     {R00,FS,-FS,DT} },
/*stato 9*/{{R01,FS,R,DR},       {D01,CSN,FS,DT},     {D00,FS,FS,DT},      {D02,FS,CSN,DT},      {R01,FS,R,DR},        {R01,FS,R,DR},        {R01,FS,R,DR},       {R01,FS,R,DR},        {R01,FS,R,DR},        {R01,FS,R,DR},       {R01,FS,R,DR} },
/*stato 10*/{{D08,FS,FS,DC},     {D08,FS,FS,DC},      {D08,FS,FS,DC},      {D08,FS,FS,DC},       {D08,FS,FS,DC},       {D08,FS,FS,DC},       {D08,FS,FS ,DC},     {D08,FS,FS,DC},       {D08,FS,FS,DC},       {D08,FS,FS,DC},      {D08,FS,FS,DC} },
/*stato 11*/{{R02,R,FS,DT},      {D01,CSN,FS,DT},     {D00,FS,FS,DT},      {D02,FS,CSN,DT},      {R02,R,FS,DT},        {R02,R,FS,DT},        {R02,R,FS,DT},       {R02,R,FS,DC},        {R02,R,FS,DC},        {R02,R,FS,DT},       {R02,R,FS,DT} },
/*stato 12*/{{R01,FS,R,DR},      {R01,FS,R,DR},       {R01,FS,R,DR},       {R01,FS,R,DR},        {R01,FS,R,DR},        {R01,FS,R,DR},        {R01,FS,R,DR},       {R01,FS,R,DR},        {R01,FS,R,DR},        {R01,FS,R,DR},       {R01,FS,R,DR} }, 
};



//-----------------------------------------------------------



#define NCAMP 10

int camp[NCAMP][6];
int val[6];
byte j,i,k=0;



// dichiarazione variabili globali
int sta;
int ing;
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
  Serial.begin(115200);
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
  sta=D00; // stato iniziale 
  ing=0;
  
}

long unsigned int time=0;


byte ting[][2]={
  {0x30,8},//110000
  {0x38,8},//111000
  {0x18,1},//011000
  {0x1c,1},//011100
  {0x0c,2},//001100
  {0x06,3},//000110
  {0x0e,3},//001110
  {0x03,7},//000011
  {0x07,7},//000111
  {0x00,5},//000000
  {0x3f,6},//111111
  {0x0f,7},//001111
  {0x1f,7},//011111
  {0x3c,8},//111100
  {0x3e,8},//111110
  {0x20,9},//100000
  {0x01,10} // 000001 
};



void loop(){
  byte ingbit;
  /*
  if((millis()-time)>1000){
    Serial.println(cont);
    delay(1000);
    cont=0;
    time=millis();  
  }*/
  
  readSensors();
  lf = leftFarReading;
  lc = leftCenterReading;
  ln = leftNearReading;
  
  rn = rightNearReading;
  rc = rightCenterReading;
  rf = rightFarReading;
  ingbit=(lf<<5) | (lc<<4)| (ln<<3)|(rn<<2)|(rc<<1) | (rf);
  
// cerca gli ingressi a bit nella tabella 
// se trovati ing = ingresso   0..9
// se non trova ing rimane invariato
  for (i=0;i<sizeof(ting)/sizeof(ting[0]);i++)
    if (ting[i][0]==ingbit)
      break;
  if (i<sizeof(ting)){
    ing=ting[i][1];    
  }
  //debug
  char mex[5];
  
  Serial.print(ingbit);
  Serial.print(' ');

  Serial.print(ing);
  Serial.print(' ');
Serial.println(sta);
    

  // gestione del contenuto della tabella
    
  byte statoSuccessivo,motoreSinistro,motoreDestro,durata;
  statoSuccessivo=pgm_read_byte_near(mat[sta][ing]+0);
  motoreSinistro=pgm_read_byte_near(mat[sta][ing]+1);
  motoreDestro=pgm_read_byte_near(mat[sta][ing]+2);
  durata=pgm_read_byte_near(mat[sta][ing]+3);
  set_motor(left_motor, motoreSinistro);
  set_motor(right_motor,motoreDestro);
  delay(temp);
  sta=a0;
}

//chiusura motori 
//      motor_brake(left_motor);
//      motor_brake(right_motor);

/*
 *legge i sensori
 *
 */
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
  for (i=0;i<6;i++) {
  //Serial.print(val[i]>soglia);
  //Serial.print("\t");
  }
  
  leftFarReading     = val[leftFar]>soglia;
  leftCenterReading  = val[leftCenter]>soglia;
  leftNearReading    = val[leftNear]>soglia;
  rightNearReading   = val[rightNear]>soglia;
  rightCenterReading = val[rightCenter]>soglia;
  rightFarReading    = val[rightFar]>soglia;
}
 


/**
 * set di funzioni per la gestione dei motori
 *
 */

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


/*
  if (stato==D00){

    
    if (!leftFarReading && !leftCenterReading && !leftNearReading && !rightFarReading && !rightCenterReading && !rightNearReading){ //Superata la linea 000000.
      stato=R00;
      motor_brake(left_motor);
      motor_brake(right_motor);
      
    }
    
    else if(!rightNearReading){// 0x10x0 il robot e' troppo a destra, deve andare piu' a sinistra quindi alzo il motore destro
      set_motor(left_motor, nearFullSpeed);
      set_motor(right_motor, full_speed);
    }
    else if(!leftNearReading){// 0x01x0 il robot e' troppo a sinistra, deve andare piu' a destra quindi alzo il motore sinistro
      set_motor(left_motor, full_speed);
      set_motor(right_motor, nearFullSpeed);
    }

    
    else{ // 0x11x0 il robot e' sopra la linea
      set_motor(right_motor, full_speed);
      set_motor(left_motor, full_speed);
    }
  }
  else if(stato==R00){

    if(!leftFarReading && leftNearReading && !rightFarReading && rightNearReading){//0x11x0
      motor_brake(left_motor);
      motor_brake(right_motor);
      stato=D00;
     
    }
    else {
      set_motor(left_motor,-turn_speed);
      set_motor(right_motor,turn_speed);
    }
  }
  
}











//Parte ancora da modificare.



















*/
void leftHandWall(){
  if((leftFarReading && leftNearReading) || (leftFarReading)|| (leftNearReading)){ // 11xxxx o 1xxxxx o x1xxxx
    turnLeft();
  }
  else{
    if((!leftFarReading && rightFarReading)||(!leftFarReading && rightFarReading && rightNearReading) ){  // 0xxxx1 o 0xxx11
      turnRight();
    }
  }
  if(!leftFarReading && !leftCenterReading && !leftNearReading
    && !rightFarReading && !rightCenterReading && !rightNearReading){ // 000000
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









/*
  if(stato==D00)
    digitalWrite(12,1);
  else
    digitalWrite(12,0);
  if(stato==R00)
    digitalWrite(13,1);
  else
    digitalWrite(13,0);
  if(leftFarReading)
    digitalWrite(6,1);
  else
    digitalWrite(6,0);
  if(leftCenterReading)
    digitalWrite(7,1);
  else
    digitalWrite(7,0);
  if(leftNearReading)
    digitalWrite(8,1);
  else
    digitalWrite(8,0);
  if(rightNearReading)
    digitalWrite(9,1);
  else
    digitalWrite(9,0);
  if(rightCenterReading)
    digitalWrite(10,1);
  else
    digitalWrite(10,0);
  if(rightFarReading)
    digitalWrite(11,1);
  else
    digitalWrite(11,0);
    */
  /*  
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
  Serial.print(sensorValues[rightFar]);
  Serial.println();
  delay(500);
  */
