//---------------------------------------------------------------------------//
//                          Fracastoro Hacking Lab                           //
//                               roboval.it                                  //
//                                  2012                                     //
//                        Jianr (Jianr Is A Nice Robot)                      //
//									     //
//									     //
//	      		    Fracastoro Hacking Lab :	     		     //
//									     //
//			      - Andrea Schiona				     //
//			      - Riccardo Mori				     //
//			      - Davide Gurnari				     //
//			      - Davide Nicolis				     //
//									     //
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

#include <QTRSensors.h>

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

// Parametri globali, modificano il comportamento del robt
#define SP_ZERO                0
#define SP_FULL              180
#define THRESHOLD   	     300     // soglia di riconosc. bianco/nero

// globals
byte sensors;
int speed_l;
int speed_r;
int cont;
int flag = 0;
void (*state)() = &start;

//                dx               sx
byte PIN_LED[] = {1, 2, 3, 11, 12, 13};
unsigned int sensorValues[NUM_SENSORS];

QTRSensorsRC qtrrc((unsigned char[]) {A0, A1, A2, A3, A4, A5},
                   NUM_SENSORS);

void setup(){
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
  readSensors();
  if (state == &start)
    print_on_led(B111111);
  else if (state == &straight)
    print_on_led(B001100);
  else if (state == &center_l_ini)
    print_on_led(B011000);
  else if (state == &center_r_ini)
    print_on_led(B000110);
  else if (state == &center_end)
    print_on_led(B010010);
  else if (state == &turn_l)
    print_on_led(B100000);
  else if (state == &turn_around)
    print_on_led(B000001);
  else if (state == &turn_end)
    print_on_led(B110011);
  else if (state == &turn_adv)
    print_on_led(B101010);
  (*state)();
}


void pass(){}

void damping(){
  speed_l = SP_FULL;
  speed_r = SP_FULL;
  set_motors(speed_l, speed_r);
}

void setspeed(int sp_l, int sp_r){
  speed_l = sp_l;
  speed_r = sp_r;
  set_motors(speed_l, speed_r);
}

void jump(int sp_l, int sp_r, void (*new_state)()){ // transition to new state
  if (new_state == &turn_adv && flag == 1) {
    motors_stop();
    delay(10000);
  }
  speed_l = sp_l;
  speed_r = sp_r;
  set_motors(speed_l, speed_r);
  state = new_state;
}

void unknown(int leds){
  motors_stop();
  print_on_led(leds);
  delay(3000);
  state = &pass;
}

void start(){
  if (!(sensors & B110011))         jump(SP_FULL, SP_FULL, &straight);
  else if (sensors & B110000)       jump(SP_ZERO, SP_FULL*.7, &center_l_ini);
  else if (sensors & B000011)       jump(SP_FULL*.7, SP_ZERO, &center_r_ini);
  else                              unknown(B010101);
}

void center_l_ini(){
  if (sensors & B000110)            jump(SP_FULL*.8, SP_ZERO, &center_end);
}

void center_r_ini(){
  if (sensors & B011000)            jump(SP_ZERO, SP_FULL*.8, &center_end);
}

void center_end(){
  if (sensors & B001100)	    jump(SP_FULL, SP_FULL, &straight);
}

void straight(){
  if (sensors == B000000)           jump(SP_FULL*.8, -SP_FULL*.8, &turn_around);
  else if (sensors == B001000)      setspeed(150, 130);
  else if (sensors == B000100)      setspeed(130, 150);
  else if (sensors == B001100)      setspeed(SP_FULL, SP_FULL);
  else if (sensors & B100000 && sensors & B001000) {
    jump(SP_FULL*.7, SP_FULL*.7, &turn_l);
    flag = 0;
  } else if (sensors == B010000)    setspeed(SP_ZERO, 130);
  else if (sensors == B011000)      setspeed(70, 130);
  else if (sensors == B000010)      setspeed(130, SP_ZERO);
  else if (sensors == B000110)      setspeed(130, 70);
}

void turn_l(){
  if (!(sensors & B100000))
    if (sensors == B000000)
      jump(-SP_FULL*.7, SP_FULL*.8, &turn_end);
    else
      if (flag == 0)
        jump(-SP_FULL*.7, SP_FULL*.8, &turn_adv);
}

void turn_around(){
  if (sensors  & B001100)
    jump(SP_FULL*0.8, SP_FULL*0.8, &straight);
}

void turn_end(){
  if (sensors & B001100) {
    jump(SP_FULL*0.8, SP_FULL*0.8, &center_correction);
    cont = 0;
  }
}

void turn_adv() {
  if (!(sensors & B001100)) {
    jump(speed_l, speed_r, &turn_end);
    flag = 1;
  }
}

void center_correction() {
  if (sensors & B110000)
    jump(SP_ZERO, SP_FULL*.8, &center_l_ini);
  else if (sensors & B000011)
    jump(SP_FULL*.8, SP_ZERO, &center_r_ini);
  else if (cont > 5) {
    jump(SP_FULL, SP_FULL, &straight);
    cont = 0;
  } else
    cont++;
}


// leds functions
void print_on_led(int leds) {
  for (int i = 0; i < 6; i++)
    digitalWrite(PIN_LED[i], (1<<i) & leds);
}

// sensors functions
void readSensors() {
  unsigned int position = qtrrc.readLine(sensorValues);
  sensors = 0;
  for (int i = 0; i < 6; i++){
    sensors = sensors << 1;
    sensors = sensors + (sensorValues[i] > THRESHOLD);}
}

// motors functions
void motors_go() {
  digitalWrite(MOTOR_STBY, HIGH);
}

void motors_stop() {
  digitalWrite(MOTOR_STBY, LOW);
}

void set_motors(int sp_l, int sp_r) {
  if (abs(sp_l)%255 < 75 && abs(sp_r)%255 < 75) {
    sp_l += 30;
    sp_r += 30;
  }
  int dir = int (sp_l >= 0);
  digitalWrite(MOTOR_LEFT_IN1, dir);
  digitalWrite(MOTOR_LEFT_IN2, !dir);
  analogWrite(MOTOR_LEFT_PWM, abs(sp_l));
  dir = int (sp_r >= 0);
  digitalWrite(MOTOR_RIGHT_IN1, dir);
  digitalWrite(MOTOR_RIGHT_IN2, !dir);
  analogWrite(MOTOR_RIGHT_PWM, abs(sp_r));
}
