
//timer1 will interrupt at 1Hz

void setup(){

//set timer1 interrupt at 1z
  cli();//stop interrupts

  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments. 16MHz/ (Prescalr*desire interrupted frequency)-1
  OCR1A = 14.624;// = (16*10^6) / (1000*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler 
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts
  Serial.begin (250000);
  //Serial.println("hi");
}

ISR(TIMER1_COMPA_vect){
Serial.println("hi");

}

void loop(){
 


  }
