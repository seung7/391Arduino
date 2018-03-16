#include <digitalWriteFast.h>  // https://github.com/NicksonYap/digitalWriteFast
#include "Arduino.h"

/* Decoders */

#define RESET_Q0 52

#define Decoder_ClockOut 8
#define Timer_ClockOut 11

byte result0 = 0;
byte result1 = 0;
byte prev_result0 = 0;
byte prev_result1 = 0;

/* Timer */
long the_time_now = 0;
long the_count_now = 0;
int toggle0 = 0;

/*Timer for PWM Clock */
#define PWM_clockout 2;
#define PID_clockout 44;

/* Motors */

#define M0_EN   4
#define M0_DIR1 22
#define M0_DIR2 23
int M0_DIR = 1;
#define M1_EN   7
#define M1_DIR1 5
#define M1_DIR2 6
int M1_DIR = 0;


/****************** SUBROUTINES ********************/

void read_and_store_decoder_data() {

 result0 = PINF & B11111111;
 //byte slotnumber = result0;
 //Serial.print (PINF);
 
 //Serial.print ( " The slotnumber: ");
 //Serial.print (slotnumber);
 if ( prev_result0 != result0 ){
  the_count_now++;
  prev_result0 = result0;
 }
  Serial.println(the_count_now);
  
 if( the_count_now > 127) { 
  the_count_now = 0;

   
   
   if(M0_DIR) {
    M0_DIR = 0;}
    else {
    M0_DIR = 1;
    }

  
  Serial.print(M0_DIR);
  M0_change_dir();
 }
 
  
 
  //Serial.print( result0 , BIN);          // Motor 0 Angular Position
  //Serial.print( " " );
  /*
  Serial.print( the_time_now );
  Serial.print( " " );
  Serial.print( the_count_now );
  Serial.print( " " );
  Serial.print( result0 );          // Motor 0 Angular Position
  Serial.println();
  */
 
}
 




/****************** SETUP ********************/
void setup_decoder_8bit_input() {
 pinModeFast (RESET_Q0, OUTPUT);
}

void reset_decoders() {
 digitalWriteFast(RESET_Q0, LOW);
 delay(5);
 digitalWriteFast(RESET_Q0, HIGH);
}


/* Output 8 MHz on OC4C (Port H, Pin 5 aka PWM/Digital 8). Using timer4 */
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
void setup_50KHz_timer() {
//set timer1 interrupt at 50KHz
 TCCR1A = 0;// set entire TCCR1A register to 0
 TCCR1A |= (1 << COM1A0);  // toggle OC1A on Compare Matc
 pinModeFast (Timer_ClockOut, OUTPUT); // pin#11 is used for the Timer_ClockOut
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 50Khz increments. 16MHz/ (Prescalr*desire interrupted frequency)-1
  OCR1A = 159;// = (16*10^6) / (50,000*1) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler 
  TCCR1B |= (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A); 
}

//Everytime the time passes 1/50,000s, it increase the variable 'the_time_now'
ISR(TIMER1_COMPA_vect){
 the_time_now++;
 }



void setup_motors() {
 pinModeFast (M0_EN,OUTPUT);
 pinModeFast (M0_DIR1,OUTPUT);
 pinModeFast (M0_DIR1,OUTPUT);
 pinModeFast (M1_EN,OUTPUT);
 pinModeFast (M1_DIR1,OUTPUT);
 pinModeFast (M1_DIR1,OUTPUT);
 
}

void M0_start() {
 digitalWrite(M0_EN, HIGH);
 digitalWrite(M0_DIR1, (M0_DIR) ? HIGH : LOW);
 digitalWrite(M0_DIR2, (!M0_DIR) ? HIGH : LOW);
}

void M0_change_dir() {
 digitalWrite(M0_DIR1, (M0_DIR) ? HIGH : LOW);
 digitalWrite(M0_DIR2, (!M0_DIR) ? HIGH : LOW);
}


 /**************** MAIN ****************/
void setup() {
 cli(); //stop interrupts
 Serial.begin (250000);
 Serial.println("hi");
 
//This will be clock frequency (Sampling Frequecy) for decoder. The expected frequency for the encoder is ~13kHz. 1MHz >> 13KHz, and the decoder should collect the data correctly.
 setup_decoder_clock(); 
 setup_50KHz_timer();
 setup_decoder_8bit_input();
 reset_decoders();

 setup_motors();
 M0_start();
 
 sei(); //allow interrupts
}

void loop() {
 read_and_store_decoder_data();
}
