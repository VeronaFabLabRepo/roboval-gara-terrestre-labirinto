//---------------------------------------------------------------------------//
//                           Fracastoro Hacking Lab                          //
//                                roboval.it                                 //
//                              2016 Sequencer                               //
//                                  Labi                                     //
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

//Lib.
#include <QTRSensors.h>

//Connection PIN
#define MOTOR_STBY             7
#define MOTOR_RIGHT_PWM       10
#define MOTOR_LEFT_PWM         5
#define MOTOR_RIGHT_IN1        8
#define MOTOR_RIGHT_IN2        9
#define MOTOR_LEFT_IN2         6
#define MOTOR_LEFT_IN1         4

//Global parameters
#define NUM_SENSORS            6
#define NUM_CALI             200

#define MAX_SPEED            250    
#define STEERING_SPEED         MAX_SPEED*.65 

#define K_PROPORTIONAL         0.12//1/30.
#define K_DERIVATIVE           1.4//1/2.
#define K_INTEGRAL             0.0001//1/30000.
#define K_LINEAR               2

#define TRESHOLD             300  

unsigned int sensorValues[NUM_SENSORS];
QTRSensorsRC qtrrc((unsigned char[]) {A0, A1, A2, A3, A4, A5}, NUM_SENSORS); 

int proportional=0, last_proportional;
long integral;
int derivative;
int correction;
int std_speed;

byte digi_sensors;

void(*state)() = &straight;

void setup() {
	Serial.begin(9600);
	pinMode(MOTOR_STBY, OUTPUT);
	pinMode(MOTOR_LEFT_PWM, OUTPUT);
	pinMode(MOTOR_LEFT_IN1, OUTPUT);
	pinMode(MOTOR_LEFT_IN2, OUTPUT);
	pinMode(MOTOR_RIGHT_PWM, OUTPUT);
	pinMode(MOTOR_RIGHT_IN1, OUTPUT);
	pinMode(MOTOR_RIGHT_IN2, OUTPUT);
  
	motors_go();

	//Start calibration 
	motors_test();
	for (int i = 0; i < NUM_CALI; i++) qtrrc.calibrate();
	motors_test(); 

	delay(2000);
}

void loop() {
	//Sensors update and execution of the current state.
	read_sensors();
	(*state)();
}

// CORRECTION

void forward() {
	//Proportional, Integral, Derivative (PID) correction system.
  integral += proportional;
	derivative = proportional - last_proportional;
	correction = proportional * K_PROPORTIONAL + derivative * K_DERIVATIVE + integral * K_INTEGRAL;
	std_speed = constrain(MAX_SPEED - (K_LINEAR * constrain(abs(correction), 0, 250)), 50, 250);

	set_motors(constrain(std_speed + correction, -MAX_SPEED, MAX_SPEED),
			   constrain(std_speed - correction, -MAX_SPEED, MAX_SPEED));
}

// STATE

void jump(int left_speed, int right_speed, void(*new_state)()) {
	//Transition to new state
	set_motors(left_speed, right_speed);
	state = new_state;
}

//void straight() {
//	//Goes straight following the rule of the left hand.
//	if (digi_sensors == B000000) 
//		jump(STEERING_SPEED, -STEERING_SPEED, &turn_right);
//	else if (digi_sensors & B100000) 
//		jump(0, 0, &unknow);
//	else if (digi_sensors & B000001) 
//		jump(0, 0, &unknow);
//	else forward();
//}

void straight() {
	//Goes straight following the rule of the left hand.
	if (digi_sensors == B000000) 
		jump(STEERING_SPEED, -STEERING_SPEED, &turn_right);
	else if (digi_sensors & B100000) 
		jump(STEERING_SPEED, STEERING_SPEED, &turn_left_init);
	else forward();
}

//void straight() {
//	//Goes straight following the rule of the left hand.
//	if (digi_sensors == B000000) 
//		jump(STEERING_SPEED, -STEERING_SPEED, &turn_right);
//	else if (digi_sensors & B100000) 
//		jump(-STEERING_SPEED, STEERING_SPEED, &turn_left);
//	else forward();
//}

void unknow(){
  delay(2000);
  jump(STEERING_SPEED, STEERING_SPEED, &straight);
}

void turn_right() {
	//Turn right until is on the line.
	if (digi_sensors & B000010){
                integral = 0;
		jump(STEERING_SPEED, STEERING_SPEED, &straight);}
}

//void turn_left_init() {
//	//Before turn left check if it has reached the end.
//	if (!(digi_sensors &   B100000))
//		jump(0, 0, &unknow);
//}

void turn_left_init() {
	//Before turn left check if it has reached the end.
	if (!(digi_sensors & B100000)){
		jump(-STEERING_SPEED, STEERING_SPEED, &turn_left);}
}

void turn_left() {
	//Turn right until is on the line.
	if (digi_sensors & B010000){
                integral = 0;
		jump(STEERING_SPEED, STEERING_SPEED, &straight);}//jump(0, 0, &unknow);}
}

//void turn_left() {
//	//Turn right until is on the line.
//	if (digi_sensors == B011000)
//                integral = 0;
//		jump(STEERING_SPEED, STEERING_SPEED, &straight);
//}

// SENSORS

void read_sensors() {
	//Analogic (range: -2500; +2500)
	last_proportional = proportional;
	proportional = qtrrc.readLine(sensorValues) - 2500;
  
	//Digital
	digi_sensors = 0;
	for(int i=0; i<NUM_SENSORS; i++){
	digi_sensors <<= 1;
	digi_sensors += (sensorValues[i] > TRESHOLD);
	}
}

// MOTORS

void motors_test(){
	//0.5 second motors test
	set_motors(MAX_SPEED, MAX_SPEED);
	delay(500);
	set_motors(0, 0);
}

void motors_go() {
	//Exit from Standby mode
	digitalWrite(MOTOR_STBY, HIGH);
}

void motors_stop() {
	//Standby mode
	digitalWrite(MOTOR_STBY, LOW);
}

void set_motors(int sp_l, int sp_r) {
	//Set right and left motors speeds (-255, 255)
	int dir = int (sp_l >= 0);
	digitalWrite(MOTOR_LEFT_IN1, dir);
	digitalWrite(MOTOR_LEFT_IN2, !dir);
	analogWrite(MOTOR_LEFT_PWM, abs(sp_l)); 
	dir = int (sp_r >= 0);
	digitalWrite(MOTOR_RIGHT_IN1, dir);
	digitalWrite(MOTOR_RIGHT_IN2, !dir);
	analogWrite(MOTOR_RIGHT_PWM, abs(sp_r)); 
}
