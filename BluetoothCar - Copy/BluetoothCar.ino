#include <LiquidCrystal.h>
#include <math.h>


#define FORWARD 'F'
#define BACKWARD 'B'
#define LEFT 'L'
#define RIGHT 'R'
#define CIRCLE 'C'
#define CROSS 'X'
#define TRIANGLE 'T'
#define SQUARE 'S'
#define START 'A'
#define PAUSE 'P'

// Motor A connections - Right
int enA = 11;
int in1 = A0;
int in2 = A1;

// Motor B connections - Left
int enB = 3;
int in3 = A2;
int in4 = A3;



// LCD
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);




void setup() {
  Serial.begin(9600);  // Set the baud rate for serial communication
  // Initialize any other necessary setup code here

  // Motor pins
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);



  // LCD initialization
  lcd.begin(16, 2);
  lcd.print(" Bluetooth Car"); 



}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    executeCommand(command);
  }
}

void executeCommand(char command) {
  switch (command) {
    case FORWARD:
      forward(); 
      break;
    case BACKWARD:
      backward(); 
      break;
    case LEFT:
      left();
      break;
    case RIGHT:
      right();  
      break;
    case CIRCLE:
      slowbackward(); 
      break;
    case CROSS:
      fastbackward(); 
      break;
    case TRIANGLE:
      fastForward(); 
      break;
    case SQUARE:
      slowForward(); 
      break;
    case START:
      break;
    case PAUSE:
      stopMotor(); 
      break;
    default:
      stopMotor(); 
  }
}


void forward() {
  
  analogWrite(enA, 80);
  analogWrite(enB, 80);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  lcd.clear(); 
  lcd.setCursor(0,0);
  lcd.print("Moving forward"); 
}

void fastForward() {
  
  analogWrite(enA, 210);
  analogWrite(enB, 210);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  lcd.clear(); 
  lcd.setCursor(0,0);
  lcd.print("Moving forward");
  lcd.setCursor(0,1); 
  lcd.print("Fast forward mode");  
}

void slowForward() {
  
  analogWrite(enA, 90);
  analogWrite(enB, 90);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  lcd.clear(); 
  lcd.setCursor(0,0);
  lcd.print("Moving forward");
  lcd.setCursor(0,1); 
  lcd.print("Slowing mode");  
}

void backward(){
  
  analogWrite(enA, 80);
  analogWrite(enB, 80);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  lcd.clear(); 
  lcd.setCursor(0,0);
  lcd.print("Moving backward");
}

void fastbackward(){
 
  analogWrite(enA, 210);
  analogWrite(enB, 210);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  lcd.clear(); 
  lcd.setCursor(0,0);
  lcd.print("Moving backward");
  lcd.setCursor(0,1); 
  lcd.print("Fast backward mode");  
}

void slowbackward(){
  
  analogWrite(enA, 90);
  analogWrite(enB, 90);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  lcd.clear(); 
  lcd.setCursor(0,0);
  lcd.print("Moving backward");
  lcd.setCursor(0,1); 
  lcd.print("Slow backward mode");  
}

void left() {
  
  analogWrite(enA, 255);
  analogWrite(enB, 255);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  
  lcd.clear(); 
  lcd.setCursor(0,0);
  lcd.print("Turning left");
  
}

void right() {
 
  analogWrite(enA, 255);
  analogWrite(enB, 255);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  lcd.clear(); 
  lcd.setCursor(0,0);
  lcd.print("Turning right");
}

void stopMotor() {
  

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  lcd.clear(); 
  lcd.setCursor(0,0);
  lcd.print("Stationary state");
}



