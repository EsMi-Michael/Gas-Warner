///
// EsMi Gaswarner
// V2 / 26 Dec 2024
// Grüne LED nur Statusanzeige in langem Intervall kurz
// Rote LED und Buzzer als Alarm
// "debug" für serielle Ausgabe
///
// Erste Stunde: Schwellwert = 100
// Danach: Schwellwert = 50 (oder Wert, der per "thres=XYZ" gesetzt wird)
// Alarm nur beim Überschreiten des analogen Schwellwerts
///

int redLed = 9;
int greenLed = 3;
int buzzer = 5;
int smokeA0 = A1;
int digin = 14;

// Standard-Schwellwert (gilt nach der ersten Stunde,
// es sei denn, er wird per "thres=XYZ" geändert)
int sensorThres = 50;

// Merkt sich den Programmstart (Millisekunden seit Reset)
unsigned long startTime = 0; 

unsigned long previousMillis = 0;  // Blink-Timer
const long interval = 30000;       // 30 Sekunden für die grüne LED
bool debugMode = false;            // Debug-Ausgabe an/aus
bool alarmEnabled = true;          // Alarm an/aus

void setup() {
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(buzzer, OUTPUT);
  
  // Falls du den digitalen Pin noch anderweitig benötigst
  pinMode(digin, INPUT);
  
  Serial.begin(9600);
  Serial.println("Gaswarner gestartet. Folgende Befehle stehen zur Verfügung:");
  Serial.println("  debug          -> Schaltet Debug-Mode an/aus");
  Serial.println("  thres=<Zahl>   -> Setzt den Schwellwert auf <Zahl>");
  Serial.println("  alarm on/off   -> Schaltet den Alarm an oder aus");
  Serial.println();

  // Startzeit merken
  startTime = millis();
}

void loop() {
  checkSerialInput();

  // Erste Stunde = 3.600.000 ms
  // => Während der ersten Stunde Schwellwert fest auf 100
  if (millis() - startTime < 3600000UL) {
    sensorThres = 120;
  }
  // Danach entweder 50 (Default) oder den Wert, den du per "thres=XYZ" eingestellt hast

  int analogSensor = analogRead(smokeA0);
  int digitalSensor = digitalRead(digin);  // nur noch zur Info, nicht relevant für Alarm

  // Wenn Debugmodus an ist, bei jedem Durchlauf die aktuellen Werte ausgeben
  if (debugMode) {
    Serial.print("Pin A0: ");
    Serial.print(analogSensor);
    Serial.print(", Pin 14: ");
    Serial.print(digitalSensor);
    Serial.print(", Schwellenwert: ");
    Serial.print(sensorThres);
    Serial.print(", Alarm ");
    Serial.println(alarmEnabled ? "EIN" : "AUS");
  }

  // Alarm nur auslösen, wenn Alarm aktiviert UND analogSensor > sensorThres
  if (alarmEnabled && (analogSensor > sensorThres)) {
    digitalWrite(redLed, HIGH);
    digitalWrite(greenLed, LOW);

    // Buzzer ertönt kurz
    tone(buzzer, 2350, 1200);
    delay(2000);
  } else {
    // Alarm aus (rote LED aus, Buzzer aus)
    digitalWrite(redLed, LOW);
    noTone(buzzer);

    // Grüne LED alle 30 Sekunden kurz blinken lassen
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      digitalWrite(greenLed, HIGH);
      delay(500);
      digitalWrite(greenLed, LOW);
    }
  }

  delay(500);
}

/**
 * Liest Befehle vom seriellen Monitor (9600 Baud) und reagiert darauf:
 *  - "debug"          -> Debug-Mode an/aus
 *  - "thres=<Zahl>"   -> neuen Schwellwert setzen (erst nach der ersten Stunde wirksam)
 *  - "alarm on/off"   -> Alarm an/aus
 */
void checkSerialInput() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Entfernt führende/nachfolgende Leerzeichen

    // Debugmodus schalten
    if (input.equalsIgnoreCase("debug")) {
      debugMode = !debugMode;
      Serial.print("Debug-Modus ");
      Serial.println(debugMode ? "aktiviert" : "deaktiviert");
      printCurrentValues();
      return;
    }

    // Alarm an/aus
    if (input.equalsIgnoreCase("alarm on")) {
      alarmEnabled = true;
      Serial.println("Alarm eingeschaltet");
      printCurrentValues();
      return;
    }
    if (input.equalsIgnoreCase("alarm off")) {
      alarmEnabled = false;
      Serial.println("Alarm ausgeschaltet");
      printCurrentValues();
      return;
    }

    // Schwellwert setzen, z.B. "thres=120"
    // Während der ersten Stunde ignorieren wir das
    if (input.startsWith("thres=")) {
      if (millis() - startTime < 3600000UL) {
        // Erste Stunde -> Wert ignorieren
        Serial.println("Schwellwert kann in der ersten Stunde nicht geaendert werden!");
        printCurrentValues();
        return;
      } else {
        // Nach der ersten Stunde
        String valueString = input.substring(6);
        int newThreshold = valueString.toInt();
        if (newThreshold > 0) {
          sensorThres = newThreshold;
          Serial.print("Neuer Schwellenwert: ");
          Serial.println(sensorThres);
        } else {
          Serial.println("Ungueltiger Wert fuer den Schwellenwert!");
        }
        printCurrentValues();
      }
      return;
    }

    // Unbekannter Befehl
    Serial.println("Unbekannter Befehl. Verfuegbare Befehle:");
    Serial.println("  debug");
    Serial.println("  thres=<Zahl>");
    Serial.println("  alarm on/off");
    printCurrentValues();
  }
}

/**
 * Gibt die aktuellen Werte (Sensor- und Statusinformationen) aus.
 */
void printCurrentValues() {
  int analogSensor = analogRead(smokeA0);
  int digitalSensor = digitalRead(digin);

  Serial.println("\n--- AKTUELLE WERTE ---");
  Serial.print("Pin A0 (analog): ");
  Serial.println(analogSensor);
  Serial.print("Pin 14 (digital): ");
  Serial.println(digitalSensor);

  Serial.print("Aktueller Schwellwert: ");
  Serial.println(sensorThres);

  Serial.print("Alarm ist ");
  Serial.println(alarmEnabled ? "EIN" : "AUS");

  Serial.print("Debug-Modus ist ");
  Serial.println(debugMode ? "aktiv" : "inaktiv");

  Serial.println("----------------------\n");
}
