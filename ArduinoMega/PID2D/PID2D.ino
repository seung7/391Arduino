/*this code makes the motor to rotates 45 back and forth. PID is used to minimize the settle time.*/
/*info: commercial motor must be more than 3V*/
/*Right is postivie, Left is negative (0->127, -127,-0)*/

#include <digitalWriteFast.h>  // https://github.com/NicksonYap/digitalWriteFast
#include "Arduino.h"

/* PWM */
#define M0_EN 2 //Enabling pin for the motor0 buttom motor
#define M1_EN 3 //Enabling pin for the motor1

/* Decoders */
#define RESET_Q0 52
#define RESET_Q1 53
#define Decoder_ClockOut 8 //Decoder1 & 2 share same Clockout
#define Timer_ClockOut 11
int8_t CurrentPointM0=PINF;
int8_t CurrentPointM1=PINK;

byte result0 = 0;
byte result1 = 0;
byte prev_result0 = 0;
byte prev_result1 = 0;
long ActualCount= 0;

/* Timer */
long the_time_now = 0;
long the_count_now = 0;
int toggle0 = 0;

/*Timer  PWM Clock */
#define PWM_clockout 2
#define PID_clockout 44

/* Motors */
#define M0_DIR1 23
#define M0_DIR2 22

#define M1_DIR1 25
#define M1_DIR2 24

int M0_DIR = 1;
int M1_DIR = 1;

/* PID */
double error, errSum, dErr;
double lastErr= 0.0;
unsigned long lastTime =0;
double kp=0.1465, ki=0.4813, kd=0.008183;
//double kp=1.1, ki=0, kd=0.03225;
int PWM_pidM0;
int PWM_pidM1;
int DesireCountM0 = 50; //half cycle 50/400 *360 = 45degree
const int DesireCountM0_1 = 90;
const int DesireCountM0_2 = 40;

int DesireCountM1 = 50;
const int DesireCountM1_1 = 90;
const int DesireCountM1_2 = 40;
int i = 0;

//Define ActualCount globally
/****************** SUBROUTINES ********************/
void read_and_store_decoder_data() {
  
 result0 = PINF & B11111111;
  if ( prev_result0 > result0 ){
  ActualCount++;
  }
  
ActualCount = ActualCount % 400;
prev_result0 = result0;
//Serial.println(ActualCount);

}

/*M0_start(), M1_start() make motor to rotate clockwise when it starts*/
void M0_start() {
 digitalWrite(M0_EN, HIGH);
 digitalWrite(M0_DIR1, (M0_DIR) ? HIGH : LOW); //BEGIN WITH MO_DIR1=HIGH, CLOCKWISE DIRECTION
 digitalWrite(M0_DIR2, (!M0_DIR) ? HIGH : LOW);
}

void M1_start() {
 digitalWrite(M1_EN, HIGH);
 digitalWrite(M1_DIR1, (M1_DIR) ? HIGH : LOW); //BEGIN WITH MO_DIR1=HIGH, CLOCKWISE DIRECTION
 digitalWrite(M1_DIR2, (!M1_DIR) ? HIGH : LOW);
}

/*This functino is no longer used*/
void M0_change_dir() {
 digitalWrite(M0_DIR1, (M0_DIR) ? HIGH : LOW);
 digitalWrite(M0_DIR2, (!M0_DIR) ? HIGH : LOW);
}
/***************** Functions **********************/


int pidController(int outputval, int desire, int actual){
int x; 
  /*How long since we last calculated*/
   unsigned long now = millis();
   double timeChange = (double)(now - lastTime);
   int output; 
   /*Compute all the working error variables*/
  
   error = desire - actual;
   if(abs(error) > 10) {
   //Serial.print("error:");
   //Serial.println(error);
   errSum += (error * timeChange);
   dErr = (error - lastErr) / timeChange;
   //Serial.print("errSum:");
   //Serial.println(errSum);
   /*Compute PID Output*/
   output = kp * error + kd * dErr;
   //Serial.println(output);
   output = abs(output*20);
   //Mutliply 50 for top motor
   //multiply small number for buttom motor
   //output= abs(output*23); //14 is pid constant value -> update the value in the future.
   //Serial.println(output);
   /*Remember some variables for next time*/
   lastErr = error;
   lastTime = now;
   
 
   x = constrain(output, 0, 255);
    if (x < 125){ // this prevent the value become less than 3v
    x=125;}
   }
   
   else{ 
   x = 0;}
   

   
   return x;
  
}
  



/****************** SETUP ********************/
void setup_motors() {
 pinModeFast (M0_EN,OUTPUT); // set PWM enable pinmode as output
 pinModeFast (M1_EN,OUTPUT);
 pinModeFast (M0_DIR1,OUTPUT);
 pinModeFast (M0_DIR1,OUTPUT);

 pinModeFast (M1_DIR1,OUTPUT);
 pinModeFast (M1_DIR2,OUTPUT);

 
}


void setup_decoder_8bit_input() {
 pinModeFast (RESET_Q0, OUTPUT);
 pinModeFast (RESET_Q1, OUTPUT);
}

void reset_decoders() {
 digitalWriteFast(RESET_Q0, LOW);
 delay(5);
 digitalWriteFast(RESET_Q0, HIGH);

 digitalWriteFast(RESET_Q1, LOW);
 delay(5);
 digitalWriteFast(RESET_Q1, HIGH);
 
}


/* Output 8 MHz on OC4C (Port H, Pin 5 aka PWM/Digital 8) */
void setup_decoder_clock() {
  pinModeFast (Decoder_ClockOut, OUTPUT);
  TCCR4A = 0;
  TCCR4A |= (1 << COM4C0);
  TCCR4B = 0;
  TCCR4B |= bit(WGM42) |  bit(CS40);
  OCR4A = 0;  
}

/* 50kHz interrupt for time-keeping
 * This timer will shows the time everytime, when the slot detectors detects the change 
 * When 5V applies, The motor provides 2000RPM which is converts to 33.333Hz.
 * 1 full rotation has 400 slots. so it gives 33.333*400 = 13,333Hz
 * Thus the timer should be bigger than 13,333Hz. 50KHz is set for that reason.
 */
/*void setup_50KHz_timer() {
//set timer1 interrupt at 50KHz
 TCCR1A = 0;// set entire TCCR1A register to 0
 TCCR1A |= (1 << COM1A0);  // toggle OC1A on Compare Matc
 pinModeFast (Timer_ClockOut, OUTPUT); // pin#11 is used for the Timer_ClockOut
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 50Khz increments. 16MHz/ (Prescalr*desire interrupted frequency)-1
  OCR1A = 159;// = (16*^6) / (50,000*1) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS bits for 24 prescaler 
  TCCR1B |= (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A); 
}

//Everytime the time passes 1/50,000s, it increase the variable 'the_time_now'
ISR(TIMER1_COMPA_vect){
 the_time_now++;
 }
*/

/*timer 1 is being used to change the desire point*/
void setup_1Hz_timer(){
TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  
  //OCR1A = 15624;
  OCR1A = 15624;
  //OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
}

/*define variables globally
This will draw a circle
Slot # being used is [100, 150]
*/

/*
//X represents X coordinates and bottom motor(m1),
//Y represents Y coordinates and top motor(m0)
float CircleArrayX[] = 25* {2, 1.866, 1.5, 1, 0.5, 0.144, 0, 0.144, 0.5, 1, 1.5, 1.866, 2} +100;
float CircleArrayY[] = 25* {1, 1.5, 1,866, 2, 1.866, 0.5, 1, 0.5, 0.144, 0, 0.144, 0.5, 1} +100;

//CircleArrayX[] = 50/2 * CircleArrayX[] +100;
//CircleArrayY[] = 50/2 * CircleArrayY[] +100;

//Change the direction of the motor every 1hz
*/
int ArrayX[] = {60, 60, 60, 60};
int ArrayY[] = {60, 10, 60, 10};
ISR(TIMER1_COMPA_vect){



 
if (i < sizeof(ArrayX)/sizeof(ArrayX[0])){
// nothing
}
else{
  i = 0;
}

//Serial.print("i: ");
//Serial.println(i);
//Serial.print("DesireCountM0: ");
//Serial.println(DesireCountM0);

DesireCountM0 = ArrayX[i];
DesireCountM1 = ArrayY[i];
Serial.print(DesireCountM1);
i++;





//DesireCountM1 = 45;
//DesireCountM0 = 45;
//DesireCountM0 = (DesireCountM0 != DesireCountM0_1 ? DesireCountM0_1 : DesireCountM0_2); 
//DesireCountM1 = (DesireCountM1 != DesireCountM1_1 ? DesireCountM1_1 : DesireCountM1_2);
}

/*3KHz timer is for the PWM Clock. Timer 3*/

/* ------------------------------------------------------------------
 * setup_PWM_timer()
 * ------------------------------------------------------------------
 * Uses TIMER3
 * PURPOSE: This code block sets the frequency of the PWM signals 
 *          whose duty cycle is set by the controller to determine 
 *          power sent to the motors.
 * DETAILS:
 *          Set timer 3 divisor to 8 for PWM frequency of 3921.16 Hz
 *          Keep all other defaults.
 * ------------------------------------------------------------------ */
/*3KHz timer is for the PWM Clock. Timer 3*/
void setup_3KHz_timer() {
 /*
 TCCR3A = 0;// set entire TCCR1A register to 0
 TCCR3A |= (1 << COM3B0);  // toggle OC1A on Compare Matc
 pinModeFast (PWM_clockout, OUTPUT);
  TCCR3B = 0;// same for TCCR1B
  TCNT3  = 0;//initialize counter value to 0
  // set compare match register for 50Khz increments. 16MHz/ (Prescalr*desire interrupted frequency)-1
  OCR3A = 26665;// = (16*10^6) / (6,000*1) - 1 (must be <65536)
  // turn on CTC mode
  TCCR3B |= (1 << WGM32);
  // Set CS12 and CS10 bits for 1024 prescaler 
  TCCR3B |= (1 << CS30);  
  // enable timer compare interrupt
  TIMSK3 |= (1 << OCIE3A); 
  */
  TCCR3B = TCCR3B & B11111000 | B00000010;
  TIMSK3 |= (1 << OCIE3A);       // Enable timer compare interrupt
}

ISR(TIMER3_COMPA_vect){
 analogWrite(M0_EN, 255);
 analogWrite(M1_EN, 255);
 //analogWrite(M0_EN, PWM_pidM0);
 //analogWrite(M1_EN, PWM_pidM1);
 //analogWrite(M0_EN, PWM_pidM0);
 //analogWrite(M1_EN, PWM_pidM1);
 
 
 }


/**0.3KHz timer is for the PID Controller_ Clock. Timer5.
 *Timer for the controller is slower than PWM Clock. 
 *So that controller will effectively control over several PWM clocks. 
 */
void setup_300Hz_timer() {

 TCCR5A = 0;// set entire TCCR1A register to 0
 TCCR5A |= (1 << COM5C0);  // toggle OC1A on Compare Matc
 pinModeFast (PID_clockout, OUTPUT);
  TCCR5B = 0;// same for TCCR1B
  TCNT5  = 0;//initialize counter value to 0
  // set compare match register for 50Khz increments. 16MHz/ (Prescalr*desire interrupted frequency)-1
  OCR5A = 53332;// = (16*10^6) / (3,00*1) - 1 (must be <65536)
  // turn on CTC mode
  TCCR5B |= (1 << WGM52);
  // Set CS12 and CS10 bits for 1024 prescaler 
  TCCR5B |= (1 << CS50);  
  // enable timer compare interrupt
  TIMSK5 |= (1 << OCIE5A); 

}

/** PID controller try to go to the Desire slot position. 
 *  It also execute pidController() function that calculate the position's error and update the PWM value.
 *  However, this new PWM value is not updated to the next tick of PWM Clock.
 */
ISR(TIMER5_COMPA_vect){
CurrentPointM0=PINF;
CurrentPointM1=PINK;

 Serial.print("PINK: ");
 Serial.println(CurrentPointM1);
 Serial.print("PINF: ");
 Serial.println(CurrentPointM0);

 //Logic for M0 (Buttom motor)
 if( CurrentPointM0 >  DesireCountM0 ) {
 //analogWrite(M0_EN, 0);
 digitalWrite (M0_DIR1, LOW);
 digitalWrite (M0_DIR2, HIGH);
 }
 else{
 digitalWrite (M0_DIR1, HIGH);
 digitalWrite (M0_DIR2, LOW);
 }


 //Logic for M1(Top motor)
 if( CurrentPointM1 > DesireCountM1) {
  digitalWrite (M1_DIR1, LOW);
  digitalWrite (M1_DIR2, HIGH);
  }
  else{
  digitalWrite (M1_DIR1, HIGH);
  digitalWrite (M1_DIR2, LOW);
  }
 
 PWM_pidM0 = pidController(PWM_pidM0, DesireCountM0, CurrentPointM0); // rotation = 400 counts
 PWM_pidM1 = pidController(PWM_pidM1, DesireCountM1, CurrentPointM1); // 1rotation = 400 counts
  
}


 /**************** MAIN ****************/
void setup() {
 cli(); //stop interrupts
 Serial.begin (250000);
 Serial.println("hi");
 
//This will be clock frequency (Sampling Frequecy) for decoder. The expected frequency for the encoder is ~13kHz. 1MHz >> 13KHz, and the decoder should collect the data correctly.
 setup_decoder_clock(); 
 //setup_50KHz_timer();
 setup_decoder_8bit_input();
 reset_decoders();
 setup_300Hz_timer();
 setup_3KHz_timer();
 setup_1Hz_timer();
 setup_motors();
 M0_start();
 M1_start();
 sei(); //allow interrupts
}

void loop() {
 read_and_store_decoder_data();

}

