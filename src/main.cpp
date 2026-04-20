#include <Arduino.h>
#include <Lights.h>
#include <Lidar.h>
#include <Engine.h>
#include <Compass.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Lidar.h>

int BUTTON_PIN = 13;
int BUTTON_STATE = 1;

/*
Motor PINS
*/
const int ENABLE_MOTOR = 32;
const int MOTOR_1 = 26;
const int MOTOR_2 = 25;
const int ROBOT_WIDTH = 82;

auto engine = Engine(MOTOR_1, MOTOR_2, ENABLE_MOTOR);
Compass robotCompass;
Servo myservo;

Distance_Sensor leftSensor;
Distance_Sensor rightSensor;
Distance_Sensor frontSensor;

void setup() {
  Serial.begin(115200);

  Wire.begin();

  set_light_state(2, 0);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Motor
  pinMode(MOTOR_1, OUTPUT);
  pinMode(MOTOR_2, OUTPUT);
  pinMode(ENABLE_MOTOR, OUTPUT);

  // Reset
  pinMode(5, OUTPUT);
  pinMode(18, OUTPUT);
  pinMode(15, OUTPUT);
  digitalWrite(5, LOW);
  digitalWrite(18, LOW);
  digitalWrite(15, LOW);

  delay(50);

  if (!frontSensor.begin(15, 0x30)) {
    Serial.println("Front ToF failed!");
    while(1);
  }

  delay(50);
  
  if (!leftSensor.begin(5, 0x31)) {
    Serial.println("Left ToF failed!");
    while(1);
  }

  delay(50);

  if (!rightSensor.begin(18, 0x32)) {
    Serial.println("Right ToF failed!");
    while(1);
  }

  if (!robotCompass.begin()) {
    Serial.println("Compass failed!");
    while(1);
  }

  ledcSetup(0, 30000, 8);
  ledcAttachPin(ENABLE_MOTOR, 0);

  engine.begin();

  myservo.attach(33, 500, 2400); 
  set_light_state(2, 3);
};

bool started = false;
float last_error = 0;
float targetAngle = NAN;
bool isClockwise = false;

const int MIN_ANGLE = 45;
const int MAX_ANGLE = 135;
const int STRAIGHT_ANGLE = 88;

const float Kp = 0;
const float Kg = 2.0;

void loop() {
    BUTTON_STATE = digitalRead(BUTTON_PIN);
    blink_lights();

    float newHeading = robotCompass.getYaw();
    if (BUTTON_STATE == 0) {
      started = !started;
      targetAngle = newHeading;
      delay(100);
    } 

    if (!started) {
      set_light_state(2, 3);
      engine.stop();
      return;
    }

    set_light_state(2, 1);
    //engine.drive(255);

    float heading = targetAngle - newHeading;
    if (heading > 180)  heading -= 360;
    if (heading < -180) heading += 360;

    const Distance_Result frontDistance = frontSensor.measureDistance();
    delay(20);

    const Distance_Result leftDistance = leftSensor.measureDistance();
    delay(20);

    const Distance_Result rightDistance = rightSensor.measureDistance();

    int angle = Kg * heading;

    Serial.println(String("L: ") + leftDistance.distance + "mm (" + leftDistance.status + ") R: " 
    + rightDistance.distance + "mm (" + rightDistance.status + ") Gyro: " + heading + " Front: " + frontDistance.distance + "mm");

    // 4 - out of range
    if (rightDistance.status == 0 && leftDistance.status == 0) {
      angle += Kp * (leftDistance.distance - rightDistance.distance);
    }

    if (abs(angle) < 3) {
      angle = 0;
    }

    const int angle_constrained = constrain(STRAIGHT_ANGLE + angle, MIN_ANGLE, MAX_ANGLE);
    myservo.write(angle_constrained);
};

