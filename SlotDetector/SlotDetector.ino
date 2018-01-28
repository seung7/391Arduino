
 #define outputSlot 13

 int counter = 0; 
 int State;
 int LastSlotDigital; 
int milli_past;
 float Angle;
 
 void setup() { 
   pinMode (outputSlot,INPUT);
   Serial.begin (250000);
   // Reads the initial state of the outputA
    LastSlotDigital = digitalRead(outputSlot);   
    Serial.print("First digital input is: ");
    Serial.println(LastSlotDigital);
    //Convert slot to Angle
    //360/100 = 3.6 DEGREE
    //count = 3
 } 
 void loop() { 
   int NewSlotDigital = digitalRead(outputSlot); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (NewSlotDigital != LastSlotDigital){     
       LastSlotDigital = NewSlotDigital;
       counter ++;
       Angle = (counter % 200) *1.8;
 
     Serial.print(" ");
     Serial.println(Angle);

     

    int milli_now = millis();
    
    if (abs(milli_now - milli_past == 1000)){
    Serial.end();
    }
    
    milli_past = milli_now;
     //Serial.println("°");
   } 
 }

