//---------------------------------------------------------------------------//
//                           Fracastoro Hacking Lab                          //
//                                roboval.it                                 //
//                                   2012                                    //
//                                  Cherie                                   //
//---------------------------------------------------------------------------//

// Copyright (c) 2011 Daniele Zambelli
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 3 as
// published by the Free Software Foundation
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

/*
Fracastoro laboratorio di robotica
*/

#include <QTRSensors.h>
//////
// pin di collegamento tra arduino, motori e sensori
#define MOTOR_STBY             7
#define MOTOR_RIGHT_PWM       10
#define MOTOR_LEFT_PWM         5
#define MOTOR_LEFT_IN2         6
#define MOTOR_LEFT_IN1         4
#define MOTOR_RIGHT_IN1        8
#define MOTOR_RIGHT_IN2        9
#define MOTOR_L                0
#define MOTOR_R                1
#define NUM_SENSORS            6

//////
// Parametri globali, modificano il comportamento del robt
#define SP_ZERO                0
#define SP_FULL              200     // from 128 to 255
#define ADJUST       250-SP_FULL
#define DAMPING                 .2
//#define SP_HALF               25
//#define SP_ADJUST             25
#define THRESHOLD     	     200     // soglia di riconosc. bianco/nero
//////
// globals
byte sensors;
//byte last_sensors;
//int state = START;
int statetime = 1;
int speed_l;
int speed_r;
void (*state)() = &start;
//long unsigned int time=0;   // only for debug

//                dx               sx
byte PIN_LED[] = {1, 2, 3, 11, 12, 13};
unsigned int sensorValues[NUM_SENSORS];

QTRSensorsRC qtrrc((unsigned char[]) {A0, A1, A2, A3, A4, A5},
                   NUM_SENSORS); //, TIMEOUT, EMITTER_PIN); 

void setup(){
//  Serial.begin(9600);
  pinMode(MOTOR_STBY, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);

  for (int i = 0; i < 6; i++)
    pinMode(PIN_LED[i], OUTPUT);
  int numcalibrations = 200;
  for (int i = 0; i < numcalibrations; i++){
    digitalWrite(PIN_LED[int(6*i/numcalibrations)], HIGH);
    qtrrc.calibrate();}
  motors_go();
}

void loop() {
  delay(10);
  readSensors();
  print_on_led(sensors);
  (*state)();
  statetime++;
}

void pass(){}

void damping(){
  int delta = (speed_r-speed_l)*DAMPING;
  speed_l += delta;
  speed_r -= delta;
  set_motors(speed_l, speed_r);
}

void setspeed(int sp_l, int sp_r){
  speed_l = sp_l;
  speed_r = sp_r;
  set_motors(speed_l, speed_r);
  statetime = 1;
}

void jump(int sp_l, int sp_r, void (*new_state)()){ // transition to new state
  speed_l = sp_l;
  speed_r = sp_r;
  set_motors(speed_l, speed_r);
  state = new_state;
  statetime = 1;
}

void unknown(int leds){
  motors_stop();
  print_on_led(leds);
  delay(3000);
  state = &pass;
}

void start(){
  if (! (sensors & B110011))       jump(SP_FULL, SP_FULL, &straight);
  else if (sensors & B110000)      jump(SP_ZERO, SP_FULL*.7, &center_l_ini);
  else if (sensors & B000011)      jump(SP_FULL*.7, SP_ZERO, &center_r_ini);
  else                             unknown(B010101);
}

void center_l_ini(){ 
  if (sensors == B000110)          jump(SP_FULL*.6, SP_ZERO, &center_end);
}

void center_r_ini(){
  if (sensors == B011000)          jump(SP_ZERO, SP_FULL*.6, &center_end);
}

void center_end(){
  if (sensors == B001100){
    motors_stop(); delay(2000); motors_go();
    jump(SP_FULL, SP_FULL, &straight);}
}

void straight(){
  if (sensors == B000000){
    jump(SP_FULL*.7, -SP_FULL*.6, &turn_r_ini);}
  else if (! (sensors & B110011))  damping();
  else if (sensors & B100000){
    jump(SP_FULL, SP_FULL, &turn_l_ini);}
  else if (sensors & B000001)      {}
  else if (sensors & B010000){
    setspeed(SP_FULL-ADJUST/statetime, SP_FULL+ADJUST/statetime);}
  else if (sensors & B000010){
    setspeed(SP_FULL+ADJUST/statetime, SP_FULL-ADJUST/statetime);}
  else                             unknown(sensors);
}

void turn_r_ini(){
  if (sensors == B000110)
    jump(SP_FULL, SP_FULL, &straight);
}

void turn_l_ini(){
  if (! (sensors & B100000))
    jump(-SP_FULL*.6, SP_FULL*.7, &turn_l);
}

void turn_l(){
  if (sensors == B011000)
    jump(SP_FULL, SP_FULL, &straight);
}

//////
// leds functions
void print_on_led(int leds) {
  for (int i = 0; i < 6; i++)
    digitalWrite(PIN_LED[i], (1<<i) & leds);
}

//////
// sensors functions
void readSensors() {
  unsigned int position = qtrrc.readLine(sensorValues);
  sensors = 0;
  for (int i = 0; i < 6; i++){
    sensors = sensors << 1;
    sensors = sensors + (sensorValues[i] > THRESHOLD);}
}

//////
// motors functions
void motors_go() {
  digitalWrite(MOTOR_STBY, HIGH);
}

void motors_stop() {
  digitalWrite(MOTOR_STBY, LOW);
}

void set_motors(int sp_l, int sp_r) {
  int dir = int (sp_l >= 0);
  digitalWrite(MOTOR_LEFT_IN1, dir);
  digitalWrite(MOTOR_LEFT_IN2, !dir);
  analogWrite(MOTOR_LEFT_PWM, abs(sp_l)); //map(abs(sp_l),0,100,0,255));
  dir = int (sp_r >= 0);
  digitalWrite(MOTOR_RIGHT_IN1, dir);
  digitalWrite(MOTOR_RIGHT_IN2, !dir);
  analogWrite(MOTOR_RIGHT_PWM, abs(sp_r)); //map(abs(sp_r),0,100,0,255));
}



