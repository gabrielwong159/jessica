int ledPin = 13;
int sensorPin = 8;
int pulse = 4;
int enable = 3;

void setup() {
  // put your setup code here, to run once:
  pinMode(sensorPin, INPUT_PULLUP);
  pinMode(pulse, OUTPUT);
  pinMode(enable, OUTPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(enable, HIGH);
  Serial.begin(9600);
}

int piOutput = 0;
void loop() {
  piOutput = digitalRead(sensorPin);
  if(piOutput){
    digitalWrite(enable, LOW);
    delay(10);
    for(int i=0;i<200;i++){
      piOutput = digitalRead(sensorPin);
      Serial.println(piOutput);
      digitalWrite(ledPin, piOutput);
      digitalWrite(pulse, HIGH);
      delay(1);
      digitalWrite(pulse, LOW);
      delay(1);
    }
    digitalWrite(enable, HIGH);
  }
}
