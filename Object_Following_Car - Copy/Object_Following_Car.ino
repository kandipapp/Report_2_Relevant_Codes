#include <Servo.h> 

#define toneC 261.63 

float distanceThresold = 9.00; 
float distanceFRONT; 
float distanceLEFT;
float distanceRIGHT; 

Servo myservo; 



//FRONT sensor
int triggerPinF = 6;
int echoPinF = 7;

//LEFT sensor
int triggerPinL = 8;
int echoPinL = 9;

//RIGHT sensor 
int triggerPinR = 4;
int echoPinR = 2;

//LED
int frontLED;
int leftLED;
int rightLED; 

// Motor A connections - Right
int enA = 11;
int in1 = A0;
int in2 = A1;

// Motor B connections - Left
int enB = 3;
int in3 = A2;
int in4 = A3;

//Buzzer
int buzzerPin = 13; 




unsigned long readUSTimeF(int triggerPinF, int echoPinF){
  pinMode(triggerPinF, OUTPUT);
  pinMode(echoPinF, INPUT); 

  digitalWrite(triggerPinF, LOW);
  delayMicroseconds(2);

  digitalWrite(triggerPinF, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPinF, LOW);
  
  return pulseIn(echoPinF, HIGH); 

}

unsigned long readUSTimeL(int triggerPinL, int echoPinL){
  pinMode(triggerPinL, OUTPUT);
  pinMode(echoPinL, INPUT); 

  digitalWrite(triggerPinL, LOW);
  delayMicroseconds(2);

  digitalWrite(triggerPinL, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPinL, LOW);
  
  return pulseIn(echoPinL, HIGH); 

}

unsigned long readUSTimeR(int triggerPinR, int echoPinR){
  pinMode(triggerPinR, OUTPUT);
  pinMode(echoPinR, INPUT); 

  digitalWrite(triggerPinR, LOW);
  delayMicroseconds(2);

  digitalWrite(triggerPinR, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPinR, LOW);
  
  return pulseIn(echoPinR, HIGH); 

}

void forward() {
  

  analogWrite(enA, 100);
  analogWrite(enB, 100);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void left(){ 
  analogWrite(enA, 255);
  analogWrite(enB, 255);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

 
void right(){
  analogWrite(enA, 255);
  analogWrite(enB, 255);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void stop(){
  analogWrite(enA, 0);
  analogWrite(enB, 0);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  myservo.attach(5); 
  pinMode(frontLED, OUTPUT); 
  pinMode(leftLED, OUTPUT); 
  pinMode(rightLED, OUTPUT); 
  pinMode(buzzerPin, OUTPUT); 

  

  
  

}

void loop() {

  digitalWrite(frontLED, HIGH); 
  digitalWrite(leftLED, HIGH); 
  digitalWrite(rightLED, HIGH); 

  distanceFRONT = (343*0.0001*readUSTimeF(triggerPinF, echoPinF))/2 ;
  Serial.print("    DistanceFRONT = ");
  Serial.print(distanceFRONT); 

  distanceLEFT = (343*0.0001*readUSTimeL(triggerPinL, echoPinL))/2 ;
  Serial.print("    DistanceLEFT = ");
  Serial.print(distanceLEFT); 

  distanceRIGHT = (343*0.0001*readUSTimeR(triggerPinR, echoPinR))/2 ;
  Serial.print("    DistanceRIGHT = ");
  Serial.print(distanceRIGHT); 
  Serial.println();




  if (distanceFRONT >= 5 && distanceFRONT <= 50) {
    forward();
    if (distanceFRONT <= 10) {
      stop(); 
      tone(buzzerPin, 261.63); 
      myservo.write(0);
      delay(250); 
      myservo.write(60);
    }
  } else if (distanceLEFT 5 < 25) {
    left();
  } else if (distanceRIGHT 5 < 25) {
    right();
  } else {
    stop();
    noTone(buzzerPin); 
  }
}