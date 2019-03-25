int ledPin = 13;
int sensorPin = 8;  // input signal from Pi
int enable = 3;  // motor enable pin
int pulse = 4;  // motor rotation pin

int prev = 0;  

void setup() {
  pinMode(sensorPin, INPUT_PULLUP);
  pinMode(pulse, OUTPUT);
  pinMode(enable, OUTPUT);
  pinMode(ledPin, OUTPUT);
  
  digitalWrite(enable, HIGH);
}

void moveStepper(int steps){
  digitalWrite(enable, LOW);
  delay(10);

  for (int i=0; i<steps; i++) {
    digitalWrite(pulse, HIGH);
    delay(1);
    digitalWrite(pulse, LOW);
    delay(1);
  }

  digitalWrite(enable, HIGH);
}

void loop() {
  int piOutput = digitalRead(sensorPin);
  digitalWrite(ledPin, piOutput);

  if (piOutput && !prev) moveStepper(200);

  prev = piOutput;
}
