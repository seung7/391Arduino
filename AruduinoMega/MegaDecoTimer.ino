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

#define CLOCKOUT 9
#define RESET 53

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
 
 if ( prev_result0 != result0 ){
  the_count_now++;
  prev_result0 = result0;
  //Serial.print( result0 , BIN);          // Motor 0 Angular Position
  //Serial.print( " " );
  Serial.print( the_time_now );
  Serial.print( " " );
  Serial.print( the_count_now );
  Serial.print( " " );
  Serial.print( result0 );          // Motor 0 Angular Position
  Serial.println();
 }
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

void setup_8_MHz_clock() {
 pinModeFast (CLOCKOUT, OUTPUT); 
 // set up Timer 1
 TCCR1A = bit (COM1A0);  // toggle OC1A on Compare Match
 TCCR1B = bit (WGM12) | bit (CS10);   // CTC, no prescaling
 OCR1A =  0;       // output every cycle
}

void setup_cool_timer() {
//set timer1 interrupt at 1z
  cli();//stop interrupts

  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments. 16MHz/ (Prescalr*desire interrupted frequency)-1
  OCR1A = 0.5625;// = (16*10^6) / (10000*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler 
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts
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

ISR(TIMER1_COMPA_vect){
 the_time_now++;
 }




 /**************** MAIN ****************/

void setup() {
 // setup_8_MHz_clock();         // for clocking decoders; too lazy to get external one for them
 setup_cool_timer();
 setup_decoder_8bit_input();
 reset_decoders();

 setup_motors();
 M0_start();
 
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
}
