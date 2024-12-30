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


int forward = 7; 
int backward = 8; 
int left = 9; 
int right = 10;


void setup() {
  Serial.begin(9600);  // Set the baud rate for serial communication
  // Initialize any other necessary setup code here

  pinMode(forward, OUTPUT); 
  pinMode(backward, OUTPUT);
  pinMode(left, OUTPUT); 
  pinMode(right, OUTPUT);  

}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    executeCommand(command);
  }
  // Continue with other tasks in your main loop
}

void executeCommand(char command) {
  switch (command) {
    case FORWARD:
      digitalWrite(forward, HIGH); 
      delay(50); 
      break;
    case BACKWARD:
      // Perform action for moving backward
      digitalWrite(backward, HIGH); 
      delay(50); 
      break;
    case LEFT:
      // Perform action for turning left
      digitalWrite(left, HIGH); 
      delay(50); 
      break;
    case RIGHT:
      // Perform action for turning right
      digitalWrite(right, HIGH); 
      delay(50); 
      break;
    case CIRCLE:
      // Perform action for circle
      break;
    case CROSS:
      // Perform action for immediate stop or crossing
      break;
    case TRIANGLE:
      // Perform action for toggling a state (e.g., LED on/off)
      break;
    case SQUARE:
      // Perform action for retrieving and sending status information
      break;
    case START:
      // Perform action for starting a process or operation
      break;
    case PAUSE:
      // Perform action for pausing a process or operation
      break;
    default:
      digitalWrite(forward, LOW); 
      digitalWrite(backward, LOW); 
      digitalWrite(left, LOW); 
      digitalWrite(right, LOW); 
      break;
  }
  


}