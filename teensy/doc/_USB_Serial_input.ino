void setup() {
  // Forces Teensy 4.x hardware down to 12 Mbit/sec compatibility mode
  // USB1_PORTSC1 |= USB_PORTSC1_PFSC; 
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    char incomingChar = Serial.read(); // Read the incoming byte
    // Ignore Carriage Return (\r) and New Line (\n)

    // type abc and enter then it prints
    // Echo: a
    // Echo: b
    // Echo: c
    if (incomingChar != '\n' && incomingChar != '\r') {
      Serial.print("Echo: ");
      Serial.println(incomingChar);
    }
  }
}
