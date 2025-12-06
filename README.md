# LFR-and-Obstacle-Avoiding-
// ==================== Mode Selection Pin ====================
#define MODE_PIN 0   // D0 → LOW = LFR, HIGH = Obstacle Avoiding

// ==================== Motor Driver Pins ====================
#define IN1 4
#define IN2 5
#define IN3 2
#define IN4 7
#define EN1 6
#define EN2 11

// ==================== Libraries ====================
#include <Servo.h>
#include <NewPing.h>

// ==================== Obstacle Avoiding Setup ====================
#define SERVO_PIN 3
#define TRIG_PIN 8
#define ECHO_PIN 9
#define MAX_DISTANCE 200
#define SAFE_DISTANCE 15
#define MOTOR_SPEED 150       
Servo myServo;
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

// ==================== LFR Setup ====================
#define IR1 A0
#define IR2 A1
#define IR3 A2
#define IR4 A3
#define IR5 A4

const int baseSpeed = 130;
const int sharpTurnSpeed = 120;
const int curveSpeed = 110;

enum TrackState { NORMAL, L_TURN_LEFT, L_TURN_RIGHT, T_SECTION, CURVE, LOST, FINISH };
TrackState currentState = NORMAL;

unsigned long lastIntersectionTime = 0;
unsigned long finishLineStartTime = 0;
bool finishLineDetected = false;
bool isTurning = false;

// ==================== Setup ====================
void setup() {
  pinMode(MODE_PIN, INPUT_PULLUP);   // Only one mode pin

  // Motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(EN1, OUTPUT); pinMode(EN2, OUTPUT);

  // Servo for obstacle avoider
  myServo.attach(SERVO_PIN);
  myServo.write(90);
  delay(500);
}

// ==================== Main Loop ====================
void loop() {
  int mode = digitalRead(MODE_PIN);

  if (mode == LOW) {
    runLFR();                  // Default mode
  } else {
    runObstacleAvoiding();     // Mode 2
  }
}

//////////////////////////////////////////////////////////////////////
// ===================== MODE 1 : LFR ==============================
//////////////////////////////////////////////////////////////////////
void runLFR() {
  int s[5] = {
    !digitalRead(IR1),
    !digitalRead(IR2),
    !digitalRead(IR3),
    !digitalRead(IR4),
    !digitalRead(IR5)
  };

  // Finish Line Detection
  if (s[0] && s[1] && s[2] && s[3] && s[4]) {
    if (finishLineStartTime == 0) {
      finishLineStartTime = millis();
    } else if (millis() - finishLineStartTime >= 2000 && !finishLineDetected) {
      finishLineDetected = true;
      currentState = FINISH;
    }
  } else {
    finishLineStartTime = 0;
  }

  if (!finishLineDetected) {
    if (s[0] && s[4] && millis() - lastIntersectionTime > 500) {
      currentState = T_SECTION;
      lastIntersectionTime = millis();
    } 
    else if ((s[0] || s[1]) && !s[2] && !s[3] && !s[4]) {
      currentState = L_TURN_LEFT;
    } 
    else if ((s[3] || s[4]) && !s[0] && !s[1] && !s[2]) {
      currentState = L_TURN_RIGHT;
    } 
    else if (s[1] || s[3]) {
      currentState = CURVE;
    } 
    else if (s[2]) {
      currentState = NORMAL;
    } 
    else {
      currentState = LOST;
    }
  }

  switch (currentState) {
    case T_SECTION:    handleTSection(s); break;
    case L_TURN_LEFT:  followLeftL(s); break;
    case L_TURN_RIGHT: followRightL(s); break;
    case CURVE:        followCurve(s[1], s[3]); break;
    case NORMAL:       forward(); break;
    case LOST:         searchLine(); break;
    case FINISH:       stopAtFinish(); break;
  }
}

void forward() {
  analogWrite(EN1, baseSpeed);
  analogWrite(EN2, baseSpeed);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void followLeftL(int s[5]) {
  if (!isTurning) {
    isTurning = true;

    analogWrite(EN1, sharpTurnSpeed);
    analogWrite(EN2, sharpTurnSpeed);
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);

    unsigned long start = millis();
    while (millis() - start < 1000) {
      if (!digitalRead(IR3)) break;
    }

    forward();
    delay(100);
    isTurning = false;
  }
}

void followRightL(int s[5]) {
  if (!isTurning) {
    isTurning = true;

    analogWrite(EN1, sharpTurnSpeed);
    analogWrite(EN2, sharpTurnSpeed);
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);

    unsigned long start = millis();
    while (millis() - start < 1000) {
      if (!digitalRead(IR3)) break;
    }

    forward();
    delay(100);
    isTurning = false;
  }
}

void handleTSection(int s[5]) {
  followRightL(s);
  delay(300);
}

void followCurve(bool leftActive, bool rightActive) {
  int speedLeft = curveSpeed;
  int speedRight = curveSpeed;

  if (leftActive)  speedRight -= 30;
  if (rightActive) speedLeft  -= 30;

  analogWrite(EN1, speedLeft);
  analogWrite(EN2, speedRight);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void searchLine() {
  static unsigned long spinStart = 0;
  static bool spinLeft = true;

  int s[5] = {
    !digitalRead(IR1),
    !digitalRead(IR2),
    !digitalRead(IR3),
    !digitalRead(IR4),
    !digitalRead(IR5)
  };

  if (s[0] || s[1] || s[2] || s[3] || s[4]) {
    forward();
    spinStart = 0;
    return;
  }

  if (spinStart == 0) spinStart = millis();

  if (spinLeft) {
    analogWrite(EN1, 100);
    analogWrite(EN2, 80);
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    analogWrite(EN1, 100);
    analogWrite(EN2, 80);
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  }

  if (millis() - spinStart >= 1500) {
    spinLeft = !spinLeft;
    spinStart = millis();
  }
}

void stopAtFinish() {
  analogWrite(EN1, 0);
  analogWrite(EN2, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);

  while (true);  // Stop permanently
}

//////////////////////////////////////////////////////////////////////
// ============== MODE 2 : OBSTACLE AVOIDING ========================
//////////////////////////////////////////////////////////////////////
void runObstacleAvoiding() {
  int distance = getDistance();

  if (distance > SAFE_DISTANCE) {
    moveForward(MOTOR_SPEED);
  } else {
    stopMotors();
    delay(100);
    moveBackward(MOTOR_SPEED);
    delay(300);
    stopMotors();
    delay(100);

    myServo.write(150);
    delay(400);
    int leftDist = getDistance();

    myServo.write(30);
    delay(400);
    int rightDist = getDistance();

    myServo.write(90);
    delay(200);

    if (leftDist >= rightDist) turnLeft();
    else turnRight();
  }
}

int getDistance() {
  delay(50);
  int cm = sonar.ping_cm();
  if (cm == 0) cm = MAX_DISTANCE;
  return cm;
}

void moveForward(int speedVal) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(EN1, speedVal);
  analogWrite(EN2, speedVal);
}

void moveBackward(int speedVal) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(EN1, speedVal);
  analogWrite(EN2, speedVal);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(EN1, MOTOR_SPEED);
  analogWrite(EN2, MOTOR_SPEED);
  delay(400);
  stopMotors();
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(EN1, MOTOR_SPEED);
  analogWrite(EN2, MOTOR_SPEED);
  delay(400);
  stopMotors();
}

void stopMotors() {
  analogWrite(EN1, 0);
  analogWrite(EN2, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
