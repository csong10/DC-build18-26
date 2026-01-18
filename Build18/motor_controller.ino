/*
 * 4WD Arduino Motor Controller
 * Controls 4 Independent Motors using two L298N Drivers
 *
 * Driver 1: Left Side (Front & Back)
 * Driver 2: Right Side (Front & Back)
 */

// --- PIN CONFIGURATION ---

// LEFT BACK MOTOR (L298N #1 - Motor A)
const int ENA = 3;  // PWM (Speed)
const int IN1 = 2;
const int IN2 = 4;

// LEFT FRONT MOTOR (L298N #1 - Motor B)
const int ENB = 5;  // PWM (Speed)
const int IN3 = 7;
const int IN4 = 8;

// RIGHT BACK MOTOR (L298N #2 - Motor A)
const int ENC = 6;  // PWM (Speed)
const int IN5 = 10;
const int IN6 = 11;

// RIGHT FRONT MOTOR (L298N #2 - Motor B)
const int END = 9;  // PWM (Speed)
const int IN7 = 12;
const int IN8 = 13;

// --- SPEED SETTINGS (0-255) ---
int speedFast = 200;
int speedTurn = 160;

void setup() {
  Serial.begin(9600); // Communication with Raspberry Pi

  // Initialize all pins as outputs
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENC, OUTPUT); pinMode(IN5, OUTPUT); pinMode(IN6, OUTPUT);
  pinMode(END, OUTPUT); pinMode(IN7, OUTPUT); pinMode(IN8, OUTPUT);

  stopMotors(); // Safety first
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    switch (cmd) {
      case 'F': moveForward(); break;
      case 'B': moveBackward(); break;
      case 'L': turnLeft(); break;
      case 'R': turnRight(); break;
      case 'S': stopMotors(); break;
    }
  }
}

// --- MOVEMENT LOGIC ---

void moveForward() {
  // All 4 wheels spin forward
  setMotor(ENA, IN1, IN2, speedFast, true);  // Left Back
  setMotor(ENB, IN3, IN4, speedFast, true);  // Left Front
  setMotor(ENC, IN5, IN6, speedFast, true);  // Right Back
  setMotor(END, IN7, IN8, speedFast, true);  // Right Front
}

void moveBackward() {
  // All 4 wheels spin backward
  setMotor(ENA, IN1, IN2, speedFast, false);
  setMotor(ENB, IN3, IN4, speedFast, false);
  setMotor(ENC, IN5, IN6, speedFast, false);
  setMotor(END, IN7, IN8, speedFast, false);
}

void turnLeft() {
  // Skid Steer: Left wheels Back, Right wheels Forward
  setMotor(ENA, IN1, IN2, speedTurn, false); // Left Back Back
  setMotor(ENB, IN3, IN4, speedTurn, false); // Left Front Back
  setMotor(ENC, IN5, IN6, speedTurn, true);  // Right Back Fwd
  setMotor(END, IN7, IN8, speedTurn, true);  // Right Front Fwd
}

void turnRight() {
  // Skid Steer: Left wheels Forward, Right wheels Back
  setMotor(ENA, IN1, IN2, speedTurn, true);  // Left Back Fwd
  setMotor(ENB, IN3, IN4, speedTurn, true);  // Left Front Fwd
  setMotor(ENC, IN5, IN6, speedTurn, false); // Right Back Back
  setMotor(END, IN7, IN8, speedTurn, false); // Right Front Back
}

void stopMotors() {
  digitalWrite(ENA, LOW); digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(ENB, LOW); digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  digitalWrite(ENC, LOW); digitalWrite(IN5, LOW); digitalWrite(IN6, LOW);
  digitalWrite(END, LOW); digitalWrite(IN7, LOW); digitalWrite(IN8, LOW);
}

// --- HELPER FUNCTION ---
// This saves us from writing the same digitalWrite code 4 times
void setMotor(int enPin, int inA, int inB, int speed, boolean forward) {
  analogWrite(enPin, speed);
  if (forward) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
  } else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
  }
}