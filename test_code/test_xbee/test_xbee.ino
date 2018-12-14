//Arduino UNO XBee Test Code
#include <SoftwareSerial.h>

#define rxPin 3
#define txPin 2
int led = 13;
int temp = 0;
SoftwareSerial xbee =  SoftwareSerial(rxPin, txPin);

void setup() {
  Serial.begin(9600); 
  pinMode(led, OUTPUT);
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  xbee.begin(9600);
}
 
void loop() {
  if (xbee.available() > 0) {
    temp = xbee.read();
    Serial.print(temp);
    xbee.println("Test");
    digitalWrite(led, HIGH);
    delay(1000);
    digitalWrite(led, LOW);
    delay(1000);
  }
}
