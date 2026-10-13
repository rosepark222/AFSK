// Pin Definitions based on your configuration
const int pinENA = 14; // PWM speed control for Motor A
const int pinENB = 15; // PWM speed control for Motor B

// const int pinIN1 = 16; // Direction control 1
// const int pinIN2 = 17; // Direction control 2
// const int pinIN3 = 34; // Direction control 3
// const int pinIN4 = 35; // Direction control 4

const int pinIN1 = 34; // Direction control 1
const int pinIN2 = 35; // Direction control 2
const int pinIN3 = 16; // Direction control 3
const int pinIN4 = 17; // Direction control 4

const int internalLED = 13; // Built-in LED on Teensy 4.1

// 1/4 speed (64 out of 255 max PWM)
const int motorSpeed = 200; // 64; 

void setup() {
  // Configure control pins and internal LED as outputs
  pinMode(pinENA, OUTPUT);
  pinMode(pinENB, OUTPUT);
  pinMode(pinIN1, OUTPUT);
  pinMode(pinIN2, OUTPUT);
  pinMode(pinIN3, OUTPUT);
  pinMode(pinIN4, OUTPUT);
  pinMode(internalLED, OUTPUT);
}

void loop() {
  // Turn LED ON
  digitalWrite(internalLED, HIGH);

  // 1. Move Forward at 1/4 speed
  analogWrite(pinENA, motorSpeed);
  analogWrite(pinENB, motorSpeed);
  digitalWrite(pinIN1, HIGH);
  digitalWrite(pinIN2, LOW);
  digitalWrite(pinIN3, HIGH);
  digitalWrite(pinIN4, LOW);
  delay(2000);

  // Stop for 1 second
  stopMotors();
  delay(1000);

  // Turn LED OFF
  digitalWrite(internalLED, LOW);

  // 2. Turn Left at 1/4 speed (Left motor reverse, Right motor forward)
  analogWrite(pinENA, motorSpeed);
  analogWrite(pinENB, motorSpeed);
  digitalWrite(pinIN1, LOW);
  digitalWrite(pinIN2, HIGH);
  digitalWrite(pinIN3, HIGH);
  digitalWrite(pinIN4, LOW);
  delay(1000);

  // Stop for 1 second
  stopMotors();
  delay(1000);

  // Turn LED ON
  digitalWrite(internalLED, HIGH);

  // 3. Turn Right at 1/4 speed (Left motor forward, Right motor reverse)
  analogWrite(pinENA, motorSpeed);
  analogWrite(pinENB, motorSpeed);
  digitalWrite(pinIN1, HIGH);
  digitalWrite(pinIN2, LOW);
  digitalWrite(pinIN3, LOW);
  digitalWrite(pinIN4, HIGH);
  delay(1000);

  // Stop for 2 seconds before repeating loop
  stopMotors();
  delay(2000);
}

// Helper function to stop both motors
void stopMotors() {
  analogWrite(pinENA, 0);
  analogWrite(pinENB, 0);
  digitalWrite(pinIN1, LOW);
  digitalWrite(pinIN2, LOW);
  digitalWrite(pinIN3, LOW);
  digitalWrite(pinIN4, LOW);
}
