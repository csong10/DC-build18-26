/*
 * Arduino Motor Controller for Raspberry Pi Rover
 * Listens for commands via USB Serial ('F', 'B', 'L', 'R', 'S')
 */

// --- WIRING CONFIGURATION ---
// L298N Motor A (Left)
const int ENA = 6;  // PWM Speed Control
const int IN1 = 7;
const int IN2 = 8;

// L298N Motor B (Right)
const int ENB = 11; // PWM Speed Control
const int IN3 = 9;
const int IN4 = 10;

// Speed Settings (0-255)
const int SPEED_FAST = 200;
const int SPEED_TURN = 150;

void setup() {
  Serial.begin(9600); // Must match Python script baudrate
  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  stopMotors(); // Start safe
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    switch(cmd) {
      case 'F': moveForward(); break;
      case 'B': moveBackward(); break;
      case 'L': turnLeft(); break;
      case 'R': turnRight(); break;
      case 'S': stopMotors(); break;
    }
  }
}

// --- MOVEMENT FUNCTIONS ---

void moveForward() {
  // Motor A Forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, SPEED_FAST);
  
  // Motor B Forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, SPEED_FAST);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, SPEED_FAST);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, SPEED_FAST);
}

void turnLeft() {
  // Left Motor Back, Right Motor Forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, SPEED_TURN);
  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, SPEED_TURN);
}

void turnRight() {
  // Left Motor Forward, Right Motor Back
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, SPEED_TURN);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, SPEED_TURN);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}