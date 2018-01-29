#define outputSlot 13
#define Direc1 8
#define Direc2 9
#define Enable 10
#define Button 7

int counter = 0;
int State;
int LastSlotDigital;  
float Angle;
int milli_past;
int first_sample_rxd = 0;


void setup() {
 Serial.begin (250000);
 pinMode (outputSlot,INPUT);
 pinMode (Button,INPUT);
 pinMode (Direc1,OUTPUT);
 pinMode (Direc2,OUTPUT);
 pinMode (Enable,OUTPUT);
 digitalWrite(Direc1, HIGH);
 digitalWrite(Direc2, LOW);
 digitalWrite(Enable, LOW);
 digitalWrite(Button, LOW);

 LastSlotDigital = digitalRead(outputSlot);
 Serial.println("Angle,Time");
}

void loop() {

 int NewSlotDigital = digitalRead(outputSlot); // Reads the "current" state of the outputA
 
 //Once the button pressed, it will enable the motor
 if(digitalRead(Button) == HIGH){
   digitalWrite(Enable,HIGH);
   }

   //Serial.print(digitalRead(Button));
     
 
 // If the previous and the current state of the outputA are different, that means a Pulse has occured
 if (NewSlotDigital != LastSlotDigital){  
     LastSlotDigital = NewSlotDigital;
     counter ++;
     Angle = (counter % 200) *1.8;
 
   Serial.print(Angle);
   Serial.print(",");
   
   int milli_now = millis();
   Serial.print(milli_now);
   Serial.println("ms");

   //This will stop the motor at the certain angle
   if ( 28 < Angle < 32 ){
     //digitalWrite(Enable,LOW);
     //Serial.end();
     }
 }
}


