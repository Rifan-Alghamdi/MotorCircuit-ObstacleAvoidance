const int pirPin = 2;
const int ledPin = 13;
const int buttonPin = 7;

unsigned long ledStartTime = 0;
bool ledByButton = false;
bool ledByMotion = false;

void setup() {
  pinMode(pirPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
}

void loop() {
  int buttonState = digitalRead(buttonPin);
  int motion = digitalRead(pirPin);

  // الزر
  if (buttonState == HIGH && !ledByButton && !ledByMotion)
  {
    ledByButton = true;
    ledStartTime = millis();
    digitalWrite(ledPin, HIGH);
  }

  if (ledByButton && millis() - ledStartTime >= 7000) {
    if (motion == LOW) {
      for (int i = 0; i < 3; i++) {
        digitalWrite(ledPin, HIGH);
        delay(300);
        digitalWrite(ledPin, LOW);
        delay(300);
      }
    }
    ledByButton = false;
    digitalWrite(ledPin, LOW);
  }

  // الحساس
  if (motion == HIGH && !ledByMotion && !ledByButton) {
    ledByMotion = true;
    digitalWrite(ledPin, HIGH);
    ledStartTime = millis();
  }

  if (ledByMotion && motion == LOW && millis() - ledStartTime >= 7000)
  {
    for (int i = 0; i < 3; i++) {
      digitalWrite(ledPin, HIGH);
      delay(300);
      digitalWrite(ledPin, LOW);
      delay(300);
    }
    ledByMotion = false;
    digitalWrite(ledPin, LOW);
  }
}