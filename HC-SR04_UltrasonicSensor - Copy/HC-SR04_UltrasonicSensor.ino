float distanceThresold = 9.00; 


int triggerPin = 7 ;
int echoPin = 6;

unsigned long readUSTime(int triggerPin, int echoPin){
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT); 

  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);

  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  
  return pulseIn(echoPin, HIGH); 

}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Time to receive echo = ");
  Serial.print(readUSTime(triggerPin, echoPin)); 
  Serial.print(" microseconds "); 

  
  float distance = (343*0.0001*readUSTime(triggerPin, echoPin))/2 ;
  Serial.print("    Distance = ");
  Serial.print(distance); 
   

  if (distance < distanceThresold){
    Serial.print("  STOP"); 
    Serial.println();
  }
  else{
    Serial.print("  CONTINUE"); 
    Serial.println();
  }



}
