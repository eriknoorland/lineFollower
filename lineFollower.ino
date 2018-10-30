// shiftIn register
#define LOAD_PIN 2
#define CLOCK_ENABLE_PIN 3
#define CLOCK_PIN 4
#define DATA_PIN 5

#define START_BUTTON_PIN 12

// motor board
#define MTR_R_ENABLE_PIN 11 // speed
#define MTR_R_INPUT_1_PIN 8 // direction
#define MTR_R_INPUT_2_PIN 9 // direction

#define MTR_L_ENABLE_PIN 10 // speed
#define MTR_L_INPUT_1_PIN 6 // direction
#define MTR_L_INPUT_2_PIN 7 // direction

#define INIT_MOTOR_SPEED 255
#define MEDIUM_MOTOR_SPEED (INIT_MOTOR_SPEED / 3) * 2
#define SLOW_MOTOR_SPEED INIT_MOTOR_SPEED / 3
#define SUPER_SLOW_MOTOR_SPEED INIT_MOTOR_SPEED / 4

#define SENSOR_LINE_VALUE 1 // 1 == black, 0 == white

int buttonReading;
int prevButtonReading = 0;
long time = 0;
long debounce = 200;
bool hasStarted = false;
int lastLeftMotorSpeedCorrection = 0;
int lastRightMotorSpeedCorrection = 0;

/**
 * Setup
 */
void setup() {
  pinMode(LOAD_PIN, OUTPUT);
  pinMode(CLOCK_ENABLE_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, INPUT);
  pinMode(START_BUTTON_PIN, INPUT);
  pinMode(MTR_R_ENABLE_PIN, OUTPUT);
  pinMode(MTR_R_INPUT_1_PIN, OUTPUT);
  pinMode(MTR_R_INPUT_2_PIN, OUTPUT);
  pinMode(MTR_L_ENABLE_PIN, OUTPUT);
  pinMode(MTR_L_INPUT_1_PIN, OUTPUT);
  pinMode(MTR_L_INPUT_2_PIN, OUTPUT);

  Serial.begin(9600);
}
 
/**
 * Loop
 */
void loop() {
  buttonReading = digitalRead(START_BUTTON_PIN);

  if (buttonReading == HIGH && prevButtonReading == LOW && millis() - time > debounce) {
    hasStarted = !hasStarted;
    time = millis();    
  }

  prevButtonReading = buttonReading;

  if (hasStarted) {
    byte sensorData = getSensorData();
    
    int leftMotorSpeedCorrection = 0;
    int rightMotorSpeedCorrection = 0;

    if (sensorData == 255 && (lastLeftMotorSpeedCorrection != 0 || lastRightMotorSpeedCorrection != 0)) {
      leftMotorSpeedCorrection = lastLeftMotorSpeedCorrection;
      rightMotorSpeedCorrection = lastRightMotorSpeedCorrection;
    } else {
      for (int i = 1; i < 8; i++) {
        int bitValue = bitRead(sensorData, i) == 0;

        // 1 == far right, 7 == far left
        if (bitValue == SENSOR_LINE_VALUE) {
          Serial.println(i);

          if (i == 1) { // hard right!
            leftMotorSpeedCorrection = 0;
            rightMotorSpeedCorrection = INIT_MOTOR_SPEED;
          }
          else if (i == 7) { // hard left!
            leftMotorSpeedCorrection = INIT_MOTOR_SPEED;
            rightMotorSpeedCorrection = 0;
          }
          else if (i == 2) { // medium right!
            leftMotorSpeedCorrection = SLOW_MOTOR_SPEED;
            rightMotorSpeedCorrection = MEDIUM_MOTOR_SPEED;
          }
          else if (i == 6) { // medium left!
            leftMotorSpeedCorrection = MEDIUM_MOTOR_SPEED;
            rightMotorSpeedCorrection = SLOW_MOTOR_SPEED;
          }
          else if (i == 3) { // soft right!
            leftMotorSpeedCorrection = SUPER_SLOW_MOTOR_SPEED;
            rightMotorSpeedCorrection = SLOW_MOTOR_SPEED;
          }
          else if (i == 5) { // soft left!
            leftMotorSpeedCorrection = SLOW_MOTOR_SPEED;
            rightMotorSpeedCorrection = SUPER_SLOW_MOTOR_SPEED;
          }
          else if (i == 4) { // immer gerade aus!
            leftMotorSpeedCorrection = 0;
            rightMotorSpeedCorrection = 0;
          }
        }
      }
    }

    int leftMotorSpeed = INIT_MOTOR_SPEED - leftMotorSpeedCorrection;
    int rightMotorSpeed = INIT_MOTOR_SPEED - rightMotorSpeedCorrection;

    // The motor speed should not exceed the max PWM value
    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

    analogWrite(MTR_L_ENABLE_PIN, leftMotorSpeed);
    digitalWrite(MTR_L_INPUT_1_PIN, LOW);
    digitalWrite(MTR_L_INPUT_2_PIN, HIGH);

    analogWrite(MTR_R_ENABLE_PIN, rightMotorSpeed);
    digitalWrite(MTR_R_INPUT_1_PIN, LOW);
    digitalWrite(MTR_R_INPUT_2_PIN, HIGH);

    lastLeftMotorSpeedCorrection = leftMotorSpeedCorrection;
    lastRightMotorSpeedCorrection = rightMotorSpeedCorrection; 
  }
  else {
    analogWrite(MTR_L_ENABLE_PIN, 0);
    analogWrite(MTR_R_ENABLE_PIN, 0);
  }
}

/**
 * Returns the sensor data
 * @return byte
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
