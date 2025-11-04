// Arduino: D7 starts ON, then listens to serial for on/off

const int GATE_PIN = 13;

void setup() {
  pinMode(GATE_PIN, OUTPUT);
  digitalWrite(GATE_PIN, HIGH);   // start ON

  Serial.begin(115200);
}

void loop() {
  // Process all incoming bytes as soon as they arrive
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '0') {
      digitalWrite(GATE_PIN, LOW);   // OFF
      // Serial.println("D13 OFF");   // optional debug
    } 
    else if (c == '1') {
      digitalWrite(GATE_PIN, HIGH);  // ON
      // Serial.println("D13 ON");    // optional debug
    }
    // ignore any other characters
  }
}


//void loop() {
//  if (Serial.available() > 0) {
//    String cmd = Serial.readStringUntil('\n');
//    cmd.trim();  // remove \r\n and spaces
//
//    if (cmd == "off" || cmd == "0") {
//      digitalWrite(GATE_PIN, LOW);
////      Serial.println("D13 OFF");
//    } else if (cmd == "on" || cmd == "1") {
//      digitalWrite(GATE_PIN, HIGH);
////      Serial.println("D12 ON");
//    }
//  }
//}
