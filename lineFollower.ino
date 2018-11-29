// shiftIn register
#define LOAD_PIN 2
#define CLOCK_ENABLE_PIN 3
#define CLOCK_PIN 4
#define DATA_PIN 5

#define MOTOR_SPEED 100
#define Kp 30
#define Kd 3

#define SENSOR_LINE_VALUE 1 // 1 = black, 0 = white

struct Motor {
  int enable; // speed
  int input1; // direction
  int input2; // direction
};

Motor leftMotor = { 10, 6, 7 };
Motor rightMotor = { 11, 8, 9 };

int lastError = 0;
long lastMillis = 0;

/**
 * Motor control
 * @param {Motor} motor
 * @param {int} speed
 * @param {int} direction
 */
void motorControl(Motor motor, int speed, int direction=1) {
  analogWrite(motor.enable, speed);
  digitalWrite(motor.input1, (direction == 1 ? LOW : HIGH));
  digitalWrite(motor.input2, (direction == 1 ? HIGH : LOW));
}

/**
 * Setup
 */
void setup() {
  pinMode(LOAD_PIN, OUTPUT);
  pinMode(CLOCK_ENABLE_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, INPUT);

  pinMode(rightMotor.enable, OUTPUT);
  pinMode(rightMotor.input1, OUTPUT);
  pinMode(rightMotor.input2, OUTPUT);

  pinMode(leftMotor.enable, OUTPUT);
  pinMode(leftMotor.input1, OUTPUT);
  pinMode(leftMotor.input2, OUTPUT);

  Serial.begin(9600);
}

/**
 * Loop
 */
void loop() {
  byte sensorData = getSensorData();

  int goal = 4;
  int activeSensor = getLineDetectionSensor(sensorData);
  int error = goal - activeSensor;
  int loopTime = millis() - lastMillis;

  float p = Kp * error;
  float d = Kd * ((error - lastError) / loopTime);

  int leftMotorSpeed = MOTOR_SPEED + (p + d);
  int rightMotorSpeed = MOTOR_SPEED - (p + d);

  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  motorControl(leftMotor, leftMotorSpeed, 1);
  motorControl(rightMotor, rightMotorSpeed, 1);

  lastError = error;
  lastMillis = millis();

  delay(20);
}

/**
 * Returns the number of the sensor that detected the line
 * @param {byte} sensorData
 * @return {int}
 */
int getLineDetectionSensor(byte sensorData) {
  for (int i = 1; i < 8; i++) {
    if ((bitRead(sensorData, i) == 0) == SENSOR_LINE_VALUE) {
      return i;
    }
  }

  return 0;
}

/**
 * Returns the sensor data
 * @return {byte} 1 == far right, 7 == far left
 */
byte getSensorData() {
  digitalWrite(LOAD_PIN, LOW);
  delayMicroseconds(5);

  digitalWrite(LOAD_PIN, HIGH);
  delayMicroseconds(5);

  digitalWrite(CLOCK_PIN, HIGH);
  digitalWrite(CLOCK_ENABLE_PIN, LOW);

  byte data = shiftIn(DATA_PIN, CLOCK_PIN, LSBFIRST);
  digitalWrite(CLOCK_ENABLE_PIN, HIGH);

  return data;
}
