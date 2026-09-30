// Two-motor IBT_2 + Arduino test (with PWM speed control)
// Commands: 1 = forward, 0 = backward, 2 = left arc, 3 = right arc, 4 = pivot left, 5 = pivot right, else = stop

const int L_RPWM = 5;   // Left motor
const int L_LPWM = 6;
const int R_RPWM = 9;   // Right motor
const int R_LPWM = 10;

const int FULL_SPEED = 255;  // full speed, matches old digitalWrite behavior
const int TURN_SPEED = 130;  // slower wheel speed for arc turns — tune this for tighter/gentler arcs

void setMotor(int rpwmPin, int lpwmPin, int speed, int dir) {
  // dir: 1 = forward, 0 = backward, else = stop
  speed = constrain(speed, 0, 255);
  if (dir == 1) {
    analogWrite(rpwmPin, speed);
    analogWrite(lpwmPin, 0);
  } else if (dir == 0) {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, speed);
  } else {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
  }
}

void driveMotor(int cmd) {
  switch (cmd) {
    case 1: // forward
      setMotor(L_RPWM, L_LPWM, FULL_SPEED, 1);
      setMotor(R_RPWM, R_LPWM, FULL_SPEED, 1);
      Serial.println("Forward");
      break;

    case 0: // backward
      setMotor(L_RPWM, L_LPWM, FULL_SPEED, 0);
      setMotor(R_RPWM, R_LPWM, FULL_SPEED, 0);
      Serial.println("Backward");
      break;

    case 2: // gentle left arc: left motor slower, right motor full, both forward
      setMotor(L_RPWM, L_LPWM, TURN_SPEED, 1);
      setMotor(R_RPWM, R_LPWM, FULL_SPEED, 1);
      Serial.println("Left arc");
      break;

    case 3: // gentle right arc: right motor slower, left motor full, both forward
      setMotor(L_RPWM, L_LPWM, FULL_SPEED, 1);
      setMotor(R_RPWM, R_LPWM, TURN_SPEED, 1);
      Serial.println("Right arc");
      break;

    case 4: // pivot left (sharp): opposite directions, same speed
      setMotor(L_RPWM, L_LPWM, FULL_SPEED, 0);
      setMotor(R_RPWM, R_LPWM, FULL_SPEED, 1);
      Serial.println("Pivot left");
      break;

    case 5: // pivot right (sharp): opposite directions, same speed
      setMotor(L_RPWM, L_LPWM, FULL_SPEED, 1);
      setMotor(R_RPWM, R_LPWM, FULL_SPEED, 0);
      Serial.println("Pivot right");
      break;

    default: // stop
      setMotor(L_RPWM, L_LPWM, 0, 2);
      setMotor(R_RPWM, R_LPWM, 0, 2);
      Serial.println("Stopped");
      break;
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);
  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);
  Serial.println("Ready. Send 1 (fwd), 0 (back), 2 (left arc), 3 (right arc), 4 (pivot left), 5 (pivot right), else (stop)");
}

void loop() {
  if (Serial.available()) {
    int cmd = Serial.parseInt();
    driveMotor(cmd);
  }
}