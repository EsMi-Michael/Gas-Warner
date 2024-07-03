///
// EsMi Gaswarner 03.07.2024
// Grüne LED nur Statusanzeige in langem Intervall kurz
// Rote LED und Buzzer als Alarm
// "debug" für Serielle Ausgabe
///

int redLed = 9;
int greenLed = 3;
int buzzer = 5;
int smokeA0 = A1;
int digin = 14;

// Ihr Schwellenwert
int sensorThres = 150;

unsigned long previousMillis = 0; // Variable zum Speichern des letzten Blinkzeitpunkts
const long interval = 30000; // 30 Sekunden Intervall
bool debugMode = false; // Variable zur Speicherung des Debug-Modus

void setup() {
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(smokeA0, INPUT);
  pinMode(digin, INPUT);
  Serial.begin(9600);
}

void loop() {
  checkSerialInput();

  int analogSensor = analogRead(smokeA0);
  int digitalSensor = digitalRead(digin);

  // Ausgabe der Werte in einer Zeile nur, wenn der Debug-Modus aktiviert ist
  if (debugMode) {
    Serial.print("Pin A0: ");
    Serial.print(analogSensor);
    Serial.print(", Pin 14: ");
    Serial.print(digitalSensor);
    Serial.print(", Schwellenwert: ");
    Serial.println(sensorThres);
  }

  // Überprüft, ob der Schwellenwert erreicht wurde oder ob der digitale Pin 14 LOW ist
  if (analogSensor > sensorThres || digitalSensor == LOW) {
    digitalWrite(redLed, HIGH);
    digitalWrite(greenLed, LOW);
    tone(buzzer, 2200, 1500);
    delay(2000);
  } else {
    digitalWrite(redLed, LOW);
    noTone(buzzer);

    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;

      // Grüne LED kurz blinken lassen
      digitalWrite(greenLed, HIGH);
      delay(100); // LED für 100 ms einschalten
      digitalWrite(greenLed, LOW);
    }
  }
  delay(100);
}

void checkSerialInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Entfernt führende und nachfolgende Leerzeichen

    if (input.equalsIgnoreCase("debug")) {
      debugMode = !debugMode; // Umschalten des Debug-Modus
      Serial.print("Debug-Modus ");
      if (debugMode) {
        Serial.println("aktiviert");
      } else {
        Serial.println("deaktiviert");
      }
    }
  }
}
