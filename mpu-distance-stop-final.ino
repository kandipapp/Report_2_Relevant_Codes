#include <LiquidCrystal.h>
#include <math.h>
#include <Wire.h>

// Motor A connections - Right
int enA = 11;
int in1 = A0;
int in2 = A1;

// Motor B connections - Left
int enB = 3;
int in3 = A2;
int in4 = A3;

// IR Sensors
int R_S = 13;  // Right sensor
int L_S = 12;  // Left sensor

// Rotary Encoder
int encPinA = 2;
const unsigned long wheelCircumference = 2 * 3.25 * 3;  // Circumference (radius = 3 cm)
volatile int pulseCount = 0;
float eq1 = 0.0;  // Distance traveled in cm
int offCount = 0;
bool startDistanceCalc = false;  // Flag to start distance calculation

// LCD
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// Timer Variables
unsigned long startMillis1 = 0;
unsigned long startMillis2 = 0;
unsigned long elapsedMillis1 = 0;
unsigned long elapsedMillis2 = 0;
unsigned long afterRotationMillis = 0;
unsigned long stopMotorDelayMillis = 0;
unsigned long stopStartTime = 0;
unsigned long rotatingMillis = 0;
bool downRampCompleted = false;
bool running = false;

// Button Setup
#define btnRight 0
#define btnUp 1
#define btnDown 2
#define btnLeft 3
#define btnSelect 4
#define btnNone 5
int lastButtonPress = btnNone;

// MPU-6050 setup
int MPU_addr = 0x68;
int AcX, AcY, AcZ;
float angle;

int minVal = 265;
int maxVal = 402;

// State Definitions
enum State { NORMAL, ON_RAMP, ROTATING, AFTER_ROTATION, AFTER210, CONTINUEAFTER, RUSHRAMP };
State vehicleState = NORMAL;

unsigned long stateStartTime = 0;  // Track state durations

void setup() {
  // Motor pins
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // IR sensors
  pinMode(R_S, INPUT);
  pinMode(L_S, INPUT);

  // Rotary encoder
  pinMode(encPinA, INPUT);
  attachInterrupt(digitalPinToInterrupt(encPinA), countPulse, CHANGE);

  // LCD initialization
  A

  // MPU-6050 setup
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);
}

void loop() {
  // Read button inputs
  int button = read_LCD_buttons();
  if (button != lastButtonPress && button != btnNone) {
    handleButtonPress(button);
    lastButtonPress = button;
    delay(200);  // Debounce delay
  } else if (button == btnNone) {
    lastButtonPress = btnNone;
  }

  // Calculate the angle
  mpu_6050();

  // Main function for motor control based on state
  handleVehicleState();

  // Update LCD with real-time data
  lcdMonitor();
}

void handleVehicleState() {
  static unsigned long rampStartTime = 0;

  switch (vehicleState) {
    case NORMAL:
      if (downRampCompleted) {  // Distance and timer start only after ramp descent
        startDistanceCalc = true;
        downRampCompleted = false;  // Reset flag for future use

      } else if (angle > 10) {
        if (rampStartTime == 0) rampStartTime = millis();
        if (millis() - rampStartTime > 500) {  // Stability check
          vehicleState = ON_RAMP;
          stateStartTime = millis();
          rampStartTime = 0;
          Serial.println("Switching to ON_RAMP state.");
        }
      } else {
        rampStartTime = 0;  // Reset if angle drops
        startMotor();
        Serial.println("Normal State");
      }
      break;

    case ON_RAMP:
      upMotor();
      if (angle < 2 && millis() - stateStartTime > 1700) {  // Ramp completion check
        vehicleState = ROTATING;
        stopMotor();
        delay(4000);
        stateStartTime = millis();
        Serial.println("Switching to ROTATING state.");
      }
      break;

    case ROTATING:
      if (millis() - stateStartTime < 1550) {
        left();  // Rotate for 2 seconds
      } else {
        stopMotor();
        delay(1000);
        vehicleState = AFTER_ROTATION;
        eq1 = 0.0;                  // Reset distance after rotation
        startDistanceCalc = false;  // Ensure distance calculation is paused
        rotatingMillis = millis();
        Serial.println("Switching to AFTER_ROTATION state.");
      }
      break;

    case AFTER_ROTATION:
      downMotor();
      if (angle < -10 && millis() - rotatingMillis > 2500) {  // Detect going down the ramp
        stopMotor();
        delay(2000);
        vehicleState = RUSHRAMP;
        afterRotationMillis = millis();
        Serial.println("Down Hill detected. Waiting 2 seconds...");
      }
      break;

    case RUSHRAMP:
    upMotor();
    if (angle>-2 && millis()-afterRotationMillis > 580){
      stopMotor();
      delay(2000);
      eq1 =0.0;
      pulseCount = 0;
      startDistanceCalc = true;
      downRampCompleted = true;  // Ensure flag is reset for state handling
      vehicleState = AFTER210;
    }
    case AFTER210:
      startMotor();
      if (eq1 >= 210.0) {  // Stop motor when distance reaches 210 cm
      stopMotor();
      stopStartTime = millis();  // Record the stop time
      vehicleState = CONTINUEAFTER;
      Serial.println("Motor stopped at 210cm.");
      }
      break;
    case CONTINUEAFTER:
      if (millis() - stopStartTime >= 2000) {
        startMotor();  
        vehicleState = NORMAL;                        // Resume movement
        Serial.println("Resuming movement after 2 seconds.");
      }
      break;
  }
}


void startMotor() {
  if (vehicleState == ROTATING) {
    return;  // Do nothing during rotation, as rotation logic is handled separately
  }

  // Normal motor control logic based on IR sensors
  if (digitalRead(R_S) == HIGH && digitalRead(L_S) == HIGH) {
    forward();
  } else if (digitalRead(R_S) == HIGH && digitalRead(L_S) == LOW) {
    right();
  } else if (digitalRead(R_S) == LOW && digitalRead(L_S) == HIGH) {
    left();
  } else if (digitalRead(R_S) == LOW && digitalRead(L_S) == LOW) {
    stopMotor();
  }
}

void upMotor() {
  if (vehicleState == ROTATING) {
    return;  // Do nothing during rotation, as rotation logic is handled separately
  }

  // Normal motor control logic based on IR sensors
  if (digitalRead(R_S) == HIGH && digitalRead(L_S) == HIGH) {
    fastForward();
  } else if (digitalRead(R_S) == HIGH && digitalRead(L_S) == LOW) {
    right();
  } else if (digitalRead(R_S) == LOW && digitalRead(L_S) == HIGH) {
    left();
  } else if (digitalRead(R_S) == LOW && digitalRead(L_S) == LOW) {
    stopMotor();
  }
}

void downMotor() {
  if (vehicleState == ROTATING) {
    return;  // Do nothing during rotation, as rotation logic is handled separately
  }

  // Normal motor control logic based on IR sensors
  if (digitalRead(R_S) == HIGH && digitalRead(L_S) == HIGH) {
    slowForward();
  } else if (digitalRead(R_S) == HIGH && digitalRead(L_S) == LOW) {
    right();
  } else if (digitalRead(R_S) == LOW && digitalRead(L_S) == HIGH) {
    left();
  } else if (digitalRead(R_S) == LOW && digitalRead(L_S) == LOW) {
    stopMotor();
  }
}

void forward() {
  running = true;
  analogWrite(enA, 80);
  analogWrite(enB, 80);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void fastForward() {
  running = true;
  analogWrite(enA, 220);
  analogWrite(enB, 235);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void slowForward() {
  running = true;
  analogWrite(enA, 65);
  analogWrite(enB, 65);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void left() {
  running = true;
  analogWrite(enA, 255);
  analogWrite(enB, 255);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void right() {
  running = true;
  analogWrite(enA, 255);
  analogWrite(enB, 255);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void stopMotor() {
  running = false;

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void countPulse() {
  int onCount = digitalRead(encPinA);
  if (offCount != onCount) {
    pulseCount++;
    if (startDistanceCalc) {
      eq1 = (float(pulseCount) * wheelCircumference) / 40.0;  // Update distance only when allowed
    }
    offCount = onCount;
  }
}

void lcdMonitor() {
  static unsigned long lastUpdate = 0;  // Tracks the last LCD update time
  unsigned long currentMillis = millis();

  if (currentMillis - lastUpdate >= 500) {  // Update every 500 ms
    lastUpdate = currentMillis;

    lcd.clear();  // Clear the LCD before updating

    if (startDistanceCalc == false) {
      // Before ramp descent completion: Show initial timer and angle
      unsigned long elapsedMillis1 = millis() - startMillis1;

      lcd.setCursor(0, 0);
      lcd.print("Time: ");
      lcd.print(elapsedMillis1 / 1000);  // Convert to seconds
      lcd.setCursor(10, 0);
      lcd.print("s");

      lcd.setCursor(0, 1);
      lcd.print("Angle: ");
      lcd.print(angle, 2);  // Show angle with 2 decimal places
      lcd.setCursor(12, 1);
      lcd.print(" deg");
    } else if (startDistanceCalc == true) {
      // After ramp descent completion: Show distance and new timer
      unsigned long elapsedMillis2 = millis() - startMillis2;

      lcd.setCursor(0, 0);
      lcd.print("Time: ");
      lcd.print(elapsedMillis2 / 1000);  // Convert to seconds
      lcd.setCursor(10, 0);
      lcd.print("s");

      lcd.setCursor(0, 1);
      lcd.print("Dist: ");
      lcd.print(eq1, 2);  // Show distance with 2 decimal places
      lcd.setCursor(14, 1);
      lcd.print("cm");
    }
  }
}

int read_LCD_buttons() {
  int adc_key_in = analogRead(A0);

  if (adc_key_in > 1000) return btnNone;
  if (adc_key_in < 50) return btnRight;
  if (adc_key_in < 195) return btnUp;
  if (adc_key_in < 380) return btnDown;
  if (adc_key_in < 555) return btnLeft;
  if (adc_key_in < 790) return btnSelect;

  return btnNone;
}

void handleButtonPress(int button) {
  if (button == btnLeft) {  // Reset timer and distance
    stopMotor();
    pulseCount = 0;
    eq1 = 0.0;

    startMillis1 = millis();
    lcd.clear();
    lcd.print("Reset!");
    delay(1000);
    lcd.clear();
  } else if (button == btnSelect) {  // Start/Stop motor
    if (running) {
      stopMotor();
    } else {
      startMotor();
    }
  }
}

void mpu_6050() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  angle = atan2((float)AcY, (float)AcZ) * 180 / PI;
}
