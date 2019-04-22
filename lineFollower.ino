#include <TimerOne.h>

#define MOTOR_SPEED 120
#define Kp 25
#define Kd 3

#define NUM_LINE_SENSORS 5
#define SENSOR_LINE_VALUE 1 // 1 = black, 0 = white

#define FPS 50

struct Motor {
  int enable; // speed
  int input1; // direction
  int input2; // direction
};

struct Ultrasonic {
  int trigger;
  int echo;
};

Motor leftMotor = { 13, 12, 11 };
Motor rightMotor = { 8, 10, 9 };

Ultrasonic frontUltrasonic = { 22, 2 };

int lineSensors[NUM_LINE_SENSORS] = { 7, 6, 5, 4, 3 }; // left to right
int lastActiveSensor;

int lastError = 0;
long lastMillis = 0;

// long duration;
volatile long distance;

/**
 * Motor control
 * @param {Motor} motor
 * @param {int} speed
 * @param {int} direction
 */
void motorControl(Motor motor, int speed, int direction=1) {
  analogWrite(motor.enable, speed);
  digitalWrite(motor.input1, (direction == 1 ? HIGH : LOW));
  digitalWrite(motor.input2, (direction == 1 ? LOW : HIGH));
}

/**
 * Setup
 */
void setup() {
  Timer1.initialize(10 * 1000);
  Timer1.attachInterrupt(getFrontDistance);

  pinMode(leftMotor.enable, OUTPUT);
  pinMode(leftMotor.input1, OUTPUT);
  pinMode(leftMotor.input2, OUTPUT);

  pinMode(rightMotor.enable, OUTPUT);
  pinMode(rightMotor.input1, OUTPUT);
  pinMode(rightMotor.input2, OUTPUT);

  pinMode(frontUltrasonic.trigger, OUTPUT);
  pinMode(frontUltrasonic.echo, INPUT);

  for (int i = 0; i < NUM_LINE_SENSORS; i++) {
    pinMode(lineSensors[i], INPUT);
  }

  Serial.begin(9600);
}

/**
 * Loop
 */
void loop() {
  unsigned long currentMillis = millis();

  if (distance >= 2 && distance <= 400) {
    if (distance <= 15) {
      motorControl(leftMotor, 0, 1);
      motorControl(rightMotor, 0, 1);
      return;
    }
  }

  if (currentMillis - lastMillis >= 1000 / FPS) {
    int lineSensorData[NUM_LINE_SENSORS] = {};

    for (int i = 0; i < NUM_LINE_SENSORS; i++) {
      lineSensorData[i] = digitalRead(lineSensors[i]);
    }

    int goal = 3;
    int activeSensor = getActiveLineDetectionSensor(lineSensorData);

    if (activeSensor == 0) {
      activeSensor = lastActiveSensor;
    }

    int error = goal - activeSensor;
    int loopTime = millis() - lastMillis;

    float p = Kp * error;
    float d = Kd * ((error - lastError) / loopTime);

    int leftMotorSpeed = MOTOR_SPEED - (p + d);
    int rightMotorSpeed = MOTOR_SPEED + (p + d);

    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

    motorControl(leftMotor, leftMotorSpeed, 1);
    motorControl(rightMotor, rightMotorSpeed, 1);

    lastError = error;
    lastMillis = millis();
    lastActiveSensor = activeSensor;
  }
}

/**
 * Returns the index of the sensor that detected the line
 * @param {array} sensorData
 * @return {int}
 */
int getActiveLineDetectionSensor(int sensorData[]) {
  for (int i = 0; i < NUM_LINE_SENSORS; i++) {
    if (sensorData[i] == SENSOR_LINE_VALUE) {
      return i + 1;
    }
  }

  return 0;
}

/**
 * Returns the front distance in cm's
 * @return
 */
long getFrontDistance() {
  digitalWrite(frontUltrasonic.trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(frontUltrasonic.trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(frontUltrasonic.trigger, LOW);

  long duration = pulseIn(frontUltrasonic.echo, HIGH);
  distance = duration * 0.034 / 2;
}
