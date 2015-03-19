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

void setup()
{
  pinMode(out_STBY,OUTPUT);
  pinMode(out_A_PWM,OUTPUT);
  pinMode(out_A_IN1,OUTPUT);
  pinMode(out_A_IN2,OUTPUT);
  pinMode(out_B_PWM,OUTPUT);
  pinMode(out_B_IN1,OUTPUT);
  pinMode(out_B_IN2,OUTPUT);

  motor_standby(false);

  set_motor(left_motor,40);
  set_motor(right_motor,40);
  delay(3000);
  set_motor(left_motor,-40);
  set_motor(right_motor,-40);
  delay(3000);
  set_motor(left_motor,40);
  set_motor(right_motor,-40);
  delay(3000);
  set_motor(left_motor,-40);
  set_motor(right_motor,40);
  delay(3000);
  set_motor(left_motor,100);
  set_motor(right_motor,100);
  delay(3000);
  motor_coast(left_motor);
  motor_coast(right_motor);
  delay(3000);
  set_motor(left_motor,100);
  set_motor(right_motor,100);
  delay(3000);
  motor_brake(left_motor);
  motor_brake(right_motor);

  motor_standby(true);
}

void loop()
{
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

