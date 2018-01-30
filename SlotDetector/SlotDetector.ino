#define outputSlot 2
#define buttonSlot 3

#define Direc1 8
#define Direc2 9
#define Enable 10
//#define Button 7

int counter = 0;
int State;
int LastSlotDigital;  
float angle;
int first_sample_rxd = 0;
int micros_now;

boolean turning_state = 0;


void setup() {
 Serial.begin (250000);
 pinMode (outputSlot,INPUT);
 pinMode (Direc1,OUTPUT);
 pinMode (Direc2,OUTPUT);
 pinMode (Enable,OUTPUT);
 digitalWrite(Direc1, HIGH);
 digitalWrite(Direc2, LOW);
 digitalWrite(Enable, LOW);

 // count changes if motor is running
 attachInterrupt(digitalPinToInterrupt(outputSlot), count_ISR, CHANGE);
 // the motor starts when you set pin 3 from low to high (do not use button, just plug it into Vcc)
 attachInterrupt(digitalPinToInterrupt(buttonSlot), start_ISR, RISING);

 Serial.println("Angle, Time");
}

void loop() {

 int NewSlotDigital = digitalRead(outputSlot); // Reads the "current" state of the outputA
 
 if(digitalRead(Enable) == HIGH){
   angle = (counter % 200) * 1.8;
   micros_now = micros();

   Serial.print(angle);
   Serial.print(",");
   Serial.println(micros_now);
   
   //This will stop the motor at the certain angle
   if ( angle > 180 ){
     //digitalWrite(Enable,LOW);
     counter = 0;
     
     if ( !turning_state ) {
     digitalWrite(Direc1, LOW);
     digitalWrite(Direc2, HIGH);
     turning_state = 1;
     } else {
     digitalWrite(Direc1, HIGH);
     digitalWrite(Direc2, LOW);
     turning_state = 0;
     }
   }
   
 }
}

void count_ISR() {
  if(digitalRead(Enable) == HIGH){
    counter++;
  }
}

void start_ISR() {
  digitalWrite(Enable,HIGH);
}
