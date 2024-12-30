#include <SoftwareSerial.h>

SoftwareSerial BTSerial(10, 11); // RX | TX

void setup() {
    Serial.begin(9600);          // Monitor serial communication
    BTSerial.begin(38400);       // HC-05 default baud rate in AT mode
    Serial.println("Enter AT commands:");
}

void loop() {
    // Forward data from Serial Monitor to HC-05
    if (Serial.available()) {
        BTSerial.write(Serial.read());
    }
    // Forward data from HC-05 to Serial Monitor
    if (BTSerial.available()) {
        Serial.write(BTSerial.read());
    }
}

