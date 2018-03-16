//This code make a motor to rotate back and forth once it reaches -180°and 180°
//The motor will accelerate and decelerate as it reach -180°and 180°

#define outputSlot 13
#define Direc1 3
#define Direc2 4
#define Enable 10
#define Button 7

int counter = 0;
int State;
int LastSlotDigital;  
float Angle;
int milli_past;
int first_sample_rxd = 0;
int DesireAngle1 = 180;
int DesireAngle2 = -180;
int pwmAccelPercent;
int countermoduler;
float pwmValue=0;

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
 Serial.println("Angle, Time");
}

void loop() {
  
 int NewSlotDigital = digitalRead(outputSlot); // Reads the "current" state of the outputA
 int countermoduler = counter % 1600;
 
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
   
   //int milli_now = millis();
   //Serial.print(milli_now);
   //Serial.println("ms");
 }

 
   //rotate 8 cycles and change the direction.
   //count = 8* 200 = 1600
    pwmAccelPercent = (1600 - countermoduler) * (100/1600); //This will accelerate and decelerate
    pwmValue= pwmAccelPercent * (255/100);
    analogWrite(Enable,pwmValue);
   if ( countermoduler  == 0) {
      if(digitalRead(Direc1) == HIGH) {
      digitalWrite(Direc1,LOW);
      digitalWrite(Direc2,HIGH);
      }

      else if(digitalRead(Direc2) == HIGH) {
      digitalWrite(Direc1,HIGH);
      digitalWrite(Direc2,LOW);
      }
   } 
   //pwmAcceleration = (1600 - counter) * (100/1600);
   //analogWrite(Enable,pwmAcceleration);
   
   /*if ( (DesireAngle1 - 2) < Angle < (DesireAngle1 +2) ){
     if( digitalRead(Direc1) == HIGH) {
     
     digitalWrite(Direc1,LOW);
     digitalWrite(Direc2,HIGH);
     
     delay(4000);  
     //counter = 0;
     }
     else if( digitalRead(Direc2) == HIGH){
     digitalWrite(Direc1,HIGH);
     digitalWrite(Direc2,LOW);
     delay(4000);
     //counter = 0;
     }
     
     }*/
 
}


