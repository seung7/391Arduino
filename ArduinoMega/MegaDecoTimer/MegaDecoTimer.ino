#include <digitalWriteFast.h>  // https://github.com/NicksonYap/digitalWriteFast
#include "Arduino.h"

/* Decoders */

#define D0 A0
#define D1 A1
#define D2 A2
#define D3 A3
#define D4 A4
#define D5 A5
#define D6 A6
#define D7 A7
//#define READING_DX 11

#define Decoder_ClockOut 8
#define Timer_ClockOut 11
#define RESET 52

byte d0_store = 1;
byte d1_store = 0;
byte d2_store = 1;
byte d3_store = 0;
byte d4_store = 1;
byte d5_store = 0;
byte d6_store = 1;
byte d7_store = 0;

byte result0 = 0;
byte result1 = 0;
byte prev_result0 = 0;
byte prev_result1 = 0;

/* Timer */
long the_time_now = 0;
long the_count_now = 0;
int toggle0 = 0;

/* Motors */

#define M0_EN   4
#define M0_DIR1 3
#define M0_DIR2 2
int M0_DIR = 1;
#define M1_EN   7
#define M1_DIR1 5
#define M1_DIR2 6
int M1_DIR = 0;


/****************** SUBROUTINES ********************/

void read_and_store_decoder_data() {
 result0 = PINF & B11111111;
 
 //if ( prev_result0 != result0 ){
 // the_count_now++;
  prev_result0 = result0;
  //Serial.print( result0 , BIN);          // Motor 0 Angular Position
  //Serial.print( " " );
  Serial.print( the_time_now );
  Serial.print( " " );
  //Serial.print( the_count_now );
  Serial.print( " " );
  Serial.print( result0 );          // Motor 0 Angular Position
  Serial.println();
// }
}
 




/****************** SETUP ********************/
void setup_decoder_8bit_input() {
 //pinModeFast (READING_DX, OUTPUT); // Select for which decoder to read
 pinModeFast (RESET, OUTPUT);

 pinModeFast (D0,INPUT);
 pinModeFast (D1,INPUT);
 pinModeFast (D2,INPUT);
 pinModeFast (D3,INPUT);
 pinModeFast (D4,INPUT);
 pinModeFast (D5,INPUT);
 pinModeFast (D6,INPUT);
 pinModeFast (D7,INPUT);
 
}

void reset_decoders() {
 digitalWriteFast(RESET, LOW);
 delay(5);
 digitalWriteFast(RESET, HIGH);
}


/** This timer actually creates 2MHz freuqncy, but it works as decoder clock with 1MHz.(Thus, this routine is called 1Mhz_decoder_clock).
 *  First 2MHz, the signal will goes up, and next frequency, the signal will goes down. As a result it creates 1Mhz Clock output for the decoder.    
 *  Look at the routine ISR(TIMER0_COMPA_vect) to check the details..
 */
void setup_1MHz_decoder_clock() {
  // Decoder_ClockOut(Pin#8 ) as Output
  pinModeFast (Decoder_ClockOut, OUTPUT); 
  //set timer0 interrupt at 1MHz
  TCCR0A = 0;// set entire TCCR0A register to 0
  TCCR0B = 0;// same for TCCR0B
  TCNT0  = 0;//initialize counter value to 0
  OCR0A = 7;// = (16*10^6) / (2*10^6*1) - 1 (must be <256);//set compare match register for 2MHz increments
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS10 bits for 1 prescaler (= no prescalar)
  TCCR0B |= (1 << CS10);  
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
}

/** This timer will shows the time everytime, when the slot detectors detects the change 
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
  OCR1A = 1000;// = (16*10^6) / (50,000*1) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler 
  TCCR1B |= (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A); 
}

//Setting decoder clock to be 1Mhz
ISR(TIMER0_COMPA_vect){
  if (toggle0){
    digitalWrite(Decoder_ClockOut,HIGH);
    toggle0 = 0;
  }
  else{
    digitalWrite(Decoder_ClockOut,LOW);
    toggle0 = 1;
  }
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
 //stop interrupts
 cli();

//This will be clock frequency (Sampling Frequecy) for decoder. The expected frequency for the encoder is ~13kHz. 1MHz >> 13KHz, and the decoder should collect the data correctly.
 setup_1MHz_decoder_clock(); 
 setup_50KHz_timer();
 setup_decoder_8bit_input();
 reset_decoders();

 setup_motors();
 M0_start();
 
 //allow interrupts
 sei();
 Serial.begin (250000);
 Serial.println("hi");
}

void loop() {
 //digitalWriteFast(READING_DX, 0);
 read_and_store_decoder_data();
 //result0 = d0_store + (d1_store << 1) + (d2_store << 2) + (d3_store << 3) + (d4_store << 4) + (d5_store << 5) + (d6_store << 6) + (d7_store << 7);
 /*
 digitalWriteFast(READING_DX, 1);
 read_and_store_decoder_data();
 result1 = D0.d0 + (D0.d1 << 1) + (D0.d2 << 2) + (D0.d3 << 3) + (D0.d4 << 4) + (D0.d5 << 5) + (D0.d6 << 6) + (D0.d7 << 7);
 */
 //Serial.print("test");
}
