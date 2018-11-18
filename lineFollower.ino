// shiftIn register
#define LOAD_PIN 2
#define CLOCK_ENABLE_PIN 3
#define CLOCK_PIN 4
#define DATA_PIN 5

// ultrasonic sensor
// #define TRIGGER_PIN 12
// #define ECHO_PIN 13

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

#define WHITE 0
#define BLACK 1
#define SENSOR_LINE_VALUE BLACK

int lastLeftMotorSpeedCorrection = 0;
int lastRightMotorSpeedCorrection = 0;

// unsigned long previousMillis = 0;

/**
 * Setup
 */
void setup() {
  pinMode(LOAD_PIN, OUTPUT);
  pinMode(CLOCK_ENABLE_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, INPUT);

  // pinMode(TRIGGER_PIN, OUTPUT);
  // pinMode(ECHO_PIN, INPUT);

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
  // unsigned long currentMillis = millis();

  // if (currentMillis - previousMillis >= 1000) {
  //   Serial.println("1 second has past");
    
  //   if (getDistance() <= 10) {
  //     Serial.println("Stop!");
  //     leftMotorSpeed = 0;
  //     rightMotorSpeed = 0;
  //   }

  //   previousMillis = currentMillis;
  // }

  byte sensorData = getSensorData();
  
  int leftMotorSpeedCorrection = 0;
  int rightMotorSpeedCorrection = 0;
  int noLineDetected = SENSOR_LINE_VALUE == 1 ? 255 : 1;

  // we lost the line, keep going in the direction you were going before
  if (sensorData == noLineDetected && (lastLeftMotorSpeedCorrection != 0 || lastRightMotorSpeedCorrection != 0)) {
    leftMotorSpeedCorrection = lastLeftMotorSpeedCorrection;
    rightMotorSpeedCorrection = lastRightMotorSpeedCorrection;
  } else {
    int numLineDetections = getNumLineDetections(sensorData);

    // if more than one sensor of the sensor array triggers
    // we might have hit a crossing and should go straight forward
    if (numLineDetections > 1) {
      leftMotorSpeedCorrection = 0;
      rightMotorSpeedCorrection = 0;
    } else {
      for (int i = 1; i < 8; i++) {
        int bitValue = bitRead(sensorData, i) == 0;

        // 1 == far right, 7 == far left
        if (bitValue == SENSOR_LINE_VALUE) {
          Serial.println(i);

          if (i == 4) { // immer gerade aus!
            leftMotorSpeedCorrection = 0;
            rightMotorSpeedCorrection = 0;
          }
          else if (i == 1) { // hard right!
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

/**
 * Returns the distance to on object in front
 * @return {long}
 */
// byte getDistance() {
//   long duration;

//   digitalWrite(TRIGGER_PIN, LOW);
//   delayMicroseconds(2);
//   digitalWrite(TRIGGER_PIN, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(TRIGGER_PIN, LOW);

//   duration = pulseIn(ECHO_PIN, HIGH);

//   return duration * 0.034 / 2;
// }

/**
 * Returns the number of simultaneous line detections
 * @param {byte} sensorData
 * @return {bool}
 */
bool getNumLineDetections(byte sensorData) {
  int numLineDetections = 0;

  for (int i = 1; i < 8; i++) {
    int bitValue = bitRead(sensorData, i) == 0;

    if (bitValue == SENSOR_LINE_VALUE) {
      numLineDetections++;
    }
  }

  return numLineDetections;
}

/**
 * Returns the sensor data
 * @return {byte}
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
