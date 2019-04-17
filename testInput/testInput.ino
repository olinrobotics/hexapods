int pin = 46;

void setup() {
  Serial.begin(9600);
  pinMode(pin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(digitalRead(pin));
}
