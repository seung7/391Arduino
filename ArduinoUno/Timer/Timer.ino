
//timer1 will interrupt at 50Hz
boolean toggle = 0 ; 
void setup(){
  pinMode(11, OUTPUT);

//set timer1 interrupt at 1z
  cli();//stop interrupts

  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments. 16MHz/ (Prescalr*desire interrupted frequency)-1
  OCR1A = 319;// = (16*10^6) / (50000*1) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1 prescaler (=No prescalar)
  TCCR1B |= (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts
  Serial.begin (250000);
  //Serial.println("hi");
}

ISR(TIMER1_COMPA_vect){//timer1 interrupt 50kHz toggles pin 13
//generates pulse wave of frequency 50kHz/2 = 25kHz (takes two cycles for full wave- toggle high then toggle low)
  if (toggle){
    digitalWrite(11,HIGH);
    toggle = 0;
  }
  else{
    digitalWrite(11,LOW);
    toggle = 1;
  }
}

void loop(){
 


  }
