/*this code makes the motor to rotates 45 back and forth. PID is used to minimize the settle time.*/

#include <digitalWriteFast.h>  // https://github.com/NicksonYap/digitalWriteFast
#include "Arduino.h"

/* PWM */
#define pwmEna 10 //Enabling pin for the motor

/* Decoders */
#define RESET_Q0 52
#define Decoder_ClockOut 8
#define Timer_ClockOut 11

byte result0 = 0;
byte result1 = 0;
byte prev_result0 = 0;
byte prev_result1 = 0;
long ActualCount= 0;

/* Timer */
long the_time_now = 0;
long the_count_now = 0;
int toggle0 = 0;

/*Timer for PWM Clock */
#define PWM_clockout 2
#define PID_clockout 44

/* Motors */
#define M0_DIR1 23
#define M0_DIR2 22
int M0_DIR = 1;
#define M1_EN   7
#define M1_DIR1 5
#define M1_DIR2 6
int M1_DIR = 0;

/* PID */
double error, errSum, dErr;
double lastErr= 0.0;
unsigned long lastTime =0;
double kp=1.1, ki=0, kd=0.03225;
int PWM_pid;
int DesireCount = 0; //half cycle 50/400 *360 = 45degree
const int DesireCount1 = 50;
const int DesireCount2 = 0;
int flag =1;
 
/****************** SUBROUTINES ********************/
void read_and_store_decoder_data() {
  
 result0 = PINF & B11111111;
 if ( prev_result0 != result0 ){
  ActualCount++;
  ActualCount = ActualCount % 400;
  prev_result0 = result0;
 }

}


void M0_start() {
 digitalWrite(pwmEna, HIGH);
 digitalWrite(M0_DIR1, (M0_DIR) ? HIGH : LOW); //BEGIN WITH MO_DIR1=HIGH, CLOCKWISE DIRECTION
 digitalWrite(M0_DIR2, (!M0_DIR) ? HIGH : LOW);
}


void M0_change_dir() {
 digitalWrite(M0_DIR1, (M0_DIR) ? HIGH : LOW);
 digitalWrite(M0_DIR2, (!M0_DIR) ? HIGH : LOW);
}
/***************** Functions **********************/
int pidController(int outputval, int desire, int actual){
 
  /*How long since we last calculated*/
   unsigned long now = millis();
   double timeChange = (double)(now - lastTime);
   int output; 
   /*Compute all the working error variables*/
   error = desire - actual;
   //Serial.print("error:");
   //Serial.println(error);
   errSum += (error * timeChange);
   dErr = (error - lastErr) / timeChange;
   //Serial.print("errSum:");
   //Serial.println(errSum);
   /*Compute PID Output*/
   output = kp * error + kd * dErr;
   //Serial.println(output);
   output= abs(output*23); //14 is pid constant value -> update the value in the future.
   //Serial.println(output);
   /*Remember some variables for next time*/
   lastErr = error;
   lastTime = now;

   int x = constrain(output, 0, 255);
   return x;
  
}
  



/****************** SETUP ********************/
void setup_motors() {
 //pinModeFast (M0_EN,OUTPUT);
 pinModeFast (M0_DIR1,OUTPUT);
 pinModeFast (M0_DIR1,OUTPUT);
 pinModeFast (M1_EN,OUTPUT);
 pinModeFast (M1_DIR1,OUTPUT);
 pinModeFast (M1_DIR1,OUTPUT);
 pinModeFast (pwmEna, OUTPUT); //set the PWM enable pinmode as output
}


void setup_decoder_8bit_input() {
 pinModeFast (RESET_Q0, OUTPUT);
}

void reset_decoders() {
 digitalWriteFast(RESET_Q0, LOW);
 delay(5);
 digitalWriteFast(RESET_Q0, HIGH);
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
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
}


//Change the direction of the motor every 1hz
ISR(TIMER1_COMPA_vect){

  if(flag=1) {
    DesireCount +=45;
    if(DesireCount >270) 
    flag=0;
  }
  else {
    DesireCount -=45;
    if(DesireCount<10)
    flag=1;
  }

 }

/*3KHz timer is for the PWM Clock. Timer 3*/
void setup_3KHz_timer() {
 
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
}

ISR(TIMER3_COMPA_vect){
 
 analogWrite(pwmEna, PWM_pid);
 
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
 
 Serial.println(PINF);
 
 if( PINF >  DesireCount ) {
 
 //analogWrite(pwmEna, 0);
  //digitalWriteFast (M0_EN, LOW);
  digitalWrite (M0_DIR1, LOW);
  digitalWrite (M0_DIR2, HIGH);
 }
 else{
 digitalWrite (M0_DIR1, HIGH);
 digitalWrite (M0_DIR2, LOW);
 }
 
 PWM_pid = pidController(PWM_pid, DesireCount, PINF); // 1rotation = 400 counts
  
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
 
 sei(); //allow interrupts
}

void loop() {
 read_and_store_decoder_data();
 
}


  
