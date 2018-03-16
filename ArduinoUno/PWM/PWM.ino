/*  Arduino DC Motor Control - PWM | H-Bridge | L298N  -  Example 01
    by Dejan Nedelkovski, www.HowToMechatronics.com
*/
const int enA=4; //Enabling pin for the motor
const int in1=22; //Digital output1
const int in2=23; //Digital output2
const int button=3; //This button will change the direction of the motor
int rotDirection = 0;
int pressed = 0;
int pwmPercentage = 100;
float pwmValue = 0;
void setup() {

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  //pinMode(button, INPUT);
  Serial.begin(9600);
  
  // Set initial rotation direction: Clockwise
  //digitalWrite(button, LOW);
 
  digitalWrite(enA, HIGH); // Enable is always on
  pwmPercentage=10; // Set the power input in percentage for the motor
  pwmValue= pwmPercentage * (255/100); // Convert percentage to Analaoge input which ranges from 0 to 255
  analogWrite(enA,pwmValue); // Send PWM to L298N Enable pin
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}
void loop() {
  
  /*
  int in1state = digitalRead(in1);
  int in2state = digitalRead(in2);
  int buttonstate = digitalRead(button);
  bool pressed=false;
  //Serial.print("input1:");
  //Serial.print(in1state);
  //Serial.print("   input2:");
  //Serial.println(in2state);
  Serial.println(buttonstate);

  // If button is pressed - change rotation direction
  
  if (buttonstate == HIGH) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  // If button is pressed - change rotation direction
  if (buttonstate == LOW) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
/*  //int potValue = analogRead(A0);// Read potentiometer value
  //int pwmOutput = map(potValue, 0, 1023, 0 , 255); // Map the potentiometer value from 0 to 255
  //analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
  // Read button - Debounce

  
 
  
  */
}
