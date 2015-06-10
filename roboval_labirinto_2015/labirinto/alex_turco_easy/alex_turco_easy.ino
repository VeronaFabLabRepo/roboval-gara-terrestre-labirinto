 /*  
    Scuola:      "ITT SAN ZENO_ISSZ"
    Squadra:       ALEX
    
    Programmatori: ALESSANDRO TURCO
       
    
    Progetto Roboval_EASY ROBOT
    ROBOVAL 2015
    Grezzana
    www.roboval.it
    
   */ 
   
   
    //includo libreria QTRSensor.h
    #include <QTRSensors.h> 
    //utilizzo la configurazione di pin e la numerazione dei sensori predefinita di Roboval - roboval_maze_v_1_0.zip  
    #define out_STBY    7
    #define out_B_PWM   10
    #define out_A_PWM   5     
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
    // definisco costanti e variabili
    #define full_speed          30 //velocita massima
    #define outline_speed    30    //velocita quando il robot non è perfettamente sulla strada (=velocita massima -> non serve a niente :) )
    #define turn_speed       20    //velocita in manovra (rotazione a destra , sinistra)
    #define blind_speed          22 //velocita motori in inversione di marcia
    #define debugPeriod       1000  //costanti Roboval non modificate
    #define soglia             210 //soglia tra 0 e 1000 per commutare il valore alto/basso analogico proveniente dai sensori in digitale 
    //variabili non modificate Roboval
    int leftFarReading;
    int rightFarReading;
    int leftCenterReading;
    int leftNearReading;
    int rightCenterReading;
    int rightNearReading;
    unsigned long N20_timer;
    int lastDebugTime = 0;
    //variabili aggiuntive
    //man mano le si legge saranno spegate..
    int sumSX=0;     
    #define REFRESH  {qtrrc.readLine(sensorValues);}                                                       // aggiorna sensori
#define BLACK(x) (sensorValues[(x)]>soglia)                                                            // sensore 'X' nero
#define WHITE(x) !BLACK(x)

#define ENGINES_APPLICATION {set_motor(left_motor, left_speed); set_motor(right_motor, right_speed);}  //trasmissione
#define BORNOUT { motor_brake(1); motor_brake(0); delay(500); }  
    #define NOX 10
    /*
    SPIEGAZIONE:
    allora, il robot per stare meglio su una linea l'abbiamo fatto ragionare in analogico...
    siccome i sensori a infrarossi, dopo la calibrazione, mandano un segnale che nel software viene compreso come un numero tra 0 e 1000
    e siccome noi possiamo inviare tramite software ai motori un valore tra 0 e 100 possiamo mettere in relazione il valore di ogni sensore  con la velocita dei motori
    
    ragioniamo cosi:
    IL ROBOT DEVE STARE IN CENTRO LINEA = HA LA PRIORITA DI RIMANERE IL PIU POSSIBILE SU UN VALORE PIU ALTO POSSIBILE DEI DUE SENSORI IR CENTRALI
    
    
    innanzitutto creiamo due var int che conterranno il valore SOMMA DESTRA E SOMMA SINISTRA che verranno messi in relazione con la velocita sul MOTORE DESTRO e sul MOTORE SINISTRO 
    
    ANALIZZIAMO ORA UN PEZZO DI CODICE CHE TROVEREMO PIU AVANTI...
    ORA , LA RELAZIONE CHE INTERCORRE FRA IL VALORE DI OUTPUT DEI SENSORI E L'INPUT AI MOTORI è UNA SEMPLICE FRAZIONE CHE PRENDE IN CONSIDERAZIONE LA FULL_SPEED E LE DUE SOMME
    
    
  sumSX=sensorValues[leftNear];                                                       è LA SOMMA  CHE AQUISISCE IL VALORE CHE RIMANDA IL SENSORE IR CENTRALE SINISTRO    1    2   |3|   4   5   6
  
  sumDX=sensorValues[rightNear];                                                      è LA SOMMA  CHE AQUISISCE IL VALORE CHE RIMANDA IL SENSORE IR CENTRALE DESTRO      1    2    3   |4|  5   6
  
                                                                                                                                                                                                               1     2    |3|   |4|    5    6
                                                                                                                                                                                                                        <-         ->
  if(sumSX<100 || sumDX<100){                                                         SE IL VALORE DI UNO DI QUESTI DUE SENSORI SCEDNDE SOTTO UNA CERTA SOGLIA (100) VADO A VEDERE QUELLI PIU ESTERNI          1    |2|    3     4    |5|   6
  
  
  if (  sumSX<100 )               
    sumSX=sensorValues[leftCenter]*2;                                                 QUI CONTROLLO QUALE DEI DUE SENSORI MI RITORNA IL VALORE PIU BASSO DELLA SOGLIA 100
    
    
    if (  sumDX<100 )
    sumDX=sensorValues[rightCenter]*2;
     
   (Limportante  DIFFERENZA CHE INTERCORRE FRA     sumSX=sensorValues[leftNear];    sumSX=sensorValues[leftCenter]*2;  non è solamente il fatto che andiamoa  vedere un sensore piu distante dal centro rispetto al precedente MA è CHE NE RADDOPPIAMO IL VALORE)
   ( POICHE IL ROBOT ,ABBIAMO DETTO, DEVE AVERE LA PRIORITA A RIMANERE SULLA LINEA CENTRALE)  
    
    }else if(sumSX<210 || sumDX<210){
      if (  sumSX<210 )
             sumSX=sensorValues[leftFar]*4;
             if (  sumDX<210 )
             sumDX=sensorValues[rightFar]*4;
             }
             
             
             
    
      
    */
    
    
    
    int sumDX=0;   
 
 
 //ABBIAMO DETTO CHE SUM sx E SUM dx  sono gli output che provengono dai sensori ,bene , adesso bisogna definire gli input VARIABILI da inviare ai motori   
    int speedSX=0;    
    int speedDX=0;    
    
    
 //abbiamo prima accennato a una outline speed : la velocita dei motori quando il robot non è perfettamente sulla linea. il robot pero come fa a capire quando è o non è sperfettamente sulla linea?
 //esso deve semplicemente vedere se uno dei valori SUMsx O SUMdx (DERIVATI DAI SENSORI) SONO COMPRESI NEL MARGINE DI +-  UN TOT PER CENTO DI ESSI STESSI (X ESEMPIO SE SUMdx è COPRESO FRA SUMSX + O -  IL SUO 5 PER CENTO ) 
    int playrange=0;  // IL "GIOCO " DEL ROBOT è 0 PER CENTO PER PROVA 
     
      /*
      LA PROSSIMA VARIABILE NON SERVE ASSOLUTAMENTE A NIENTE è SOLO CHE SUL NOSTRO ROBOT IL MOTORE DESTRO RENDEVA L' 8% DI MENO RISPETTO AL SINISTRO ,L'HARDWARE DECELERATION è UNA COSTANTE DA AGGIUNGERE AI PARAMETRI PER DEFINIRE LA speed sx , SAREBBE COME DIRE:
      SE IL MOTORE DESTRIO VA L'8% DI MENO RIUSPETTO AL SINISTRO PRENDIAMO LA VELOCITA CHE VOGLIAMO CHE FACCIA IL MOTORE SINISTRO E DECREMENTIAMOLA DELL' 8 PER CENTO
      */
      
    int hardware_deceleration=0;
    
    
    // ALTRI PEZZI DI CODICE STANDARD ROBOVAL....

QTRSensorsRC qtrrc((unsigned char[]) {A0, A1, A2, A3, A4, A5},NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];
//ANCHE QUI NON CAMBIERà NIENTE....
void setup(){
 
  pinMode(out_STBY,OUTPUT);
  pinMode(out_A_PWM,OUTPUT);
  pinMode(out_A_IN1,OUTPUT);
  pinMode(out_A_IN2,OUTPUT);
  pinMode(out_B_PWM,OUTPUT);
  pinMode(out_B_IN1,OUTPUT);
  pinMode(out_B_IN2,OUTPUT);
  for (int i = 0; i < 100; i++){
    qtrrc.calibrate();
  }
  motor_standby(false);  
}



//ECCOCI NEL LOOP ...SI DIVIDE IN TRE PARTI...
void loop(){
  
  
  // PRIMA PARET: LETTURA DEGLI INPUT
  readSensors();
      if( WHITE(leftNear) || WHITE(rightNear) ) N20_timer = millis();  // se il robot non è perfettamente in linea resetta il timer del NOS

  
 //ALGORITMO DI RISOLUZIONE (REGOLA DELLA MANO SINISTRA)     IN OGNI FUNZIONE POI CI SARA L'ANALISI DEGLI INPUT E IL CALCOLO DEGLI OUTPUT 
     if(leftFarReading){
        turnLeft();
     }else{
         if(rightFarReading){
             turnRight();
             }
           }
           
  if(!leftFarReading && !rightFarReading && !leftCenterReading && !rightCenterReading && !leftNearReading && !rightNearReading){
        turnAround();
        }

  straight(); 
 
 
 
   //OUTPUT CALCOLATI DALLE FUNZIONI
   set_motor(left_motor, speedSX-speedSX* hardware_deceleration/100); //ECCO CHE SI VEDE L'UTILITA DELL'HARDWARE DECELERATION 
   set_motor(right_motor, speedDX);
   
} 




//ECCO STRAIGHT UNA FUNZIONE DI TOTALE INVENZIONE NOSTRA FATTA IN MODO CHE IL ROBO RESTI IL PIU POSSIBILE AL CENTRO DELLA LINEA NERA
void straight(){
  
  //ECCO IL PEZZO DI CODICE SPIEGATO SU, ALLA DEFINIZIONE DEELE VARIABILI GLOBALI
  sumSX=sensorValues[leftNear];
  sumDX=sensorValues[rightNear];
  
  if(sumSX<100 || sumDX<100){
  if (  sumSX<100 )
    sumSX=sensorValues[leftCenter]*2;
    if (  sumDX<100 )
    sumDX=sensorValues[rightCenter]*2;
    }else if(sumSX<210 || sumDX<210){
      if (  sumSX<210 )
             sumSX=sensorValues[leftFar]*4;
             if (  sumDX<210 )
             sumDX=sensorValues[rightFar]*4;
             }
             
             
             
             
   //ED ECCO L'ANALISI DEGLI INPUT E IL CALCOLO DEGLI OUTPUT          
  if(sumSX>sumDX){
    /*
    ED ECCO PRONTI LA SPIEGAZIONE E LA FORMULA CON LA QUALE SI METTE IN RELAZIONE GLI INPUT(SENSORI)  E GLI OUTPUT(MOTORI)
   
   ipotizziamo di avere due numeri: A  e   B
   ipotizziamo ora che A>B
   
   se dividiamo B per A il risultato sara un numero sempre piu piccolo al diminuire di A  o all'aumentare di B
   
    ALLORA, in questo caso somma destra < somma sinistra...
    
    VUOL DIRE CHE:
   TESI:
   
    IL ROBOT è PIU A DESTRA DELLA LINEA...
   
   SOLUZIONE :
    
    PASSO1-   AUMENTIAMO LA VELOCITA DEI MOTORI DI DESTRA (SI MA DI QUANTO?    SEMPLICE : FULL_SPEED)
    
    PASSO2-   DIMINUIAMO LA VELOCITA DEI  MOTORI DI SINISTRA (SI MA DI QUANTO?)
    
    
    RISPOSTA: BELLA DOMANDA!
    
    
    LA VELOCITA DEl MOTORE SI SINISTRA DOVRA ESSERE IN PROPORZIONE A QUANTO IL ROBOT è FUORI LINEA
    SICCOME STABILIAMO LA VELOCITA  DEI MOTORI DI DESTRA A FULL_SPEED NON POTRA FARE ALTRO CHE TORNARE PIAN PIANO SULLA LINEA
    AUMENTERA ANCHE LA VELOCITA DEL  MOTORE DI SINISTRA
    
    LA RISPOSTA è 
     speedSX=(sumDX*full_speed)/sumSX;  
    */
    
    
    
    
      speedDX=full_speed;
      speedSX=(sumDX*full_speed)/sumSX;          
      }
  
  if(sumSX<sumDX){
     //STESSO DISCORSO DI PRIMA SOLTANTO AL CONTRARIO
     speedSX=full_speed;
     speedDX=(sumSX*full_speed)/sumDX;           
     }
  
  
  //ECCO IN QUESTO IF SI CONTROLLA SE LE DUE SOMME  SONO SIMILI DEL PLAYRANGE %         ... SE SI ALLORA VUOL DIRE CHE NEL ROBOT  è NELLO SPAZIO DI GIOCO CONSENTITO E POTRA SBATTERSENE DELLE VELOCITA IN PROPORZIONE E ANDARE A MANETTA
  if (( (sumDX>(sumSX-(sumSX*playrange/100))) && (sumDX<(sumSX+(sumSX*playrange/100))) ) || ( (sumSX>(sumDX-(sumDX*playrange/100))) && (sumSX<(sumDX+(sumDX*playrange/100))) ) ){
    
      speedSX=outline_speed;
      speedDX=outline_speed; 
      }
      
  if(sumSX==sumDX){
     //ECCO QUI SI STIPULA ANCHE IL CASO IN CUI IL ROBOT SIA PERFETTAMENTE SULLA LINEA (IL CHE NON ACCADE QUASI MAI =)        
     speedSX=full_speed;
     speedDX=full_speed;           
     }
 
       speedSX += ( ( millis() - N20_timer ) / 1000 ) * NOX; //accellerazione proporzionale al tempo sulla linea
      speedDX += ( ( millis() - N20_timer ) / 1000 ) * NOX; //accellerazione proporzionale al tempo sulla linea
    
      if ( speedSX > 100 ) speedSX = 100; //controllo overflow
      if ( speedDX > 100 ) speedDX = 100; //controllo overflow
          
        
 } 

      
 void turnLeft(){
   
   //APPENA VEDO UNA CURVA A SINISTRA DEVO GIRARE A PRIORI POICHE STO SCEGLIENDO DI USARE LA REGOLA DELLA MANO SINISTRA COMUNQUE VADO AVANTI UN PO PER ESSERE DOPO ALLIEATO
   set_motor(left_motor, full_speed-full_speed* hardware_deceleration/100);
   set_motor(right_motor, full_speed);
   //delay(10);
   //BORNOUT;
   //ECCO CHE FORZO I MOTORI A GIRARE A PRIORI
    set_motor(left_motor,-turn_speed+turn_speed*hardware_deceleration/100);
    set_motor(right_motor,turn_speed+15);
    
        
         readSensors();
         //LEGGO I SENSORI E CONTINUO A LEGGERLI FINCHE  IL SENSORE DESTRO CENTRALE  O IL SINISTRO CENTRALE NON VEDE BIANCO (VUOL DIRE CHE SARO USCITO)
  if(rightNearReading || leftNearReading ){
  while(rightNearReading){   
    
    readSensors();
    
  }
  }
       //LEGGO I SENSORI E CONTINUO A LEGGERLI FINCHE  IL SENSORE DESTRO CENTRALE O IL SINISTRO CENTRALE NON VEDE NERO (VUOL DIRE CHE SARO RIENTRATO NELLA CURVA)
   while(!(rightNearReading || leftNearReading)){   
    set_motor(left_motor,-turn_speed+turn_speed*hardware_deceleration/100);
    set_motor(right_motor,turn_speed+5);
    readSensors();
  }
   
}





 void turnRight(){
   //APPENA VEDO UNA CURVA A DESTRA VADO AVANTI PER ASSICURARMI CHE NON CI SIA STRADA  :: SE CE STRADA DEVO PROSEGUIRE POICHE STO SCEGLIENDO DI USARE LA REGOLA DELLA MANO SINIOSTRA
   set_motor(left_motor, full_speed-full_speed* hardware_deceleration/100);
   set_motor(right_motor, full_speed);
  delay(10);
  readSensors();
             //SE DAVANTI NON CE NIENTE ALLORA GIRO 
  if(!(rightNearReading || leftNearReading)){
    //BORNOUT;
    set_motor(left_motor,turn_speed+25-turn_speed*hardware_deceleration/100);
         set_motor(right_motor,-turn_speed);
         //CONTINUO A GIRARE FINCHE NON RIENTRO IN PISTA 
      while(!(leftNearReading || leftNearReading)){
         
         readSensors();
         }
      }

}      




//ASI UNGUAL ALLA VOSTRA SOLO CON QUALCHE PICCOLA MODIFICA : L'HARDWARE DEC....
    void turnAround(){
     
  while(!leftNearReading && !rightNearReading){
    
    set_motor(left_motor,+blind_speed+15-blind_speed*hardware_deceleration/100);
    set_motor(right_motor,-blind_speed-15);
    readSensors();
    
  }

} 
//TUTTO IL RESTO SONO FUNZIONI ROBOVAL PREESISTENTI 
void set_motor(boolean motor, char speed) { 
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
void motor_speed(boolean motor, boolean direction, byte speed) { 
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
void motor_standby(boolean state) { 
  if (state == true)
    digitalWrite(out_STBY,LOW);
  else
    digitalWrite(out_STBY,HIGH);
}

void motor_coast(boolean motor) { 
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

void motor_brake(boolean motor) { 
  if (motor == left_motor) {
    digitalWrite(out_A_IN1,HIGH);
    digitalWrite(out_A_IN2,HIGH);
  } 
  else {
    digitalWrite(out_B_IN1,HIGH);
    digitalWrite(out_B_IN2,HIGH);
  }
}



void readSensors(){
  
  
  unsigned int position = qtrrc.readLine(sensorValues);
  leftFarReading     = sensorValues[leftFar]>soglia;   
  rightFarReading    = sensorValues[rightFar]>soglia;
  leftCenterReading  = sensorValues[leftCenter]>soglia;
  leftNearReading    = sensorValues[leftNear]>soglia;
  rightNearReading   = sensorValues[rightNear]>soglia;
  rightCenterReading = sensorValues[rightCenter]>soglia;

}



