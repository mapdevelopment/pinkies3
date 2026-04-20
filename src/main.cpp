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
const int MOTOR_1 = 25;
const int MOTOR_2 = 26;
const int ROBOT_WIDTH = 82;

auto engine = Engine(MOTOR_1, MOTOR_2, ENABLE_MOTOR);
Compass robotCompass;
Servo myservo;

Distance_Sensor leftSensor;
Distance_Sensor rightSensor;

void setup() {
  Serial.begin(9600);

  Wire.begin();

  set_light_state(2, 0);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Motor
  pinMode(MOTOR_1, OUTPUT);
  pinMode(MOTOR_2, OUTPUT);
  pinMode(ENABLE_MOTOR, OUTPUT);

  ledcSetup(0, 30000, 8);
  ledcAttachPin(ENABLE_MOTOR, 0);

  engine.begin();

  if (!robotCompass.begin()) {
    Serial.println("BNO085 Fail!");
    while(1) {};
  }

  if (!leftSensor.begin(4, 0x30) || !rightSensor.begin(5, 0x31)) {
    Serial.println("ToF Sensors failed to start!");
    while(1) {};
  }
  
  myservo.attach(33); 
  set_light_state(2, 3);
};

float started = false;
float last_error = 0;
float targetAngle = NULL;
bool isClockwise = false;

const int MIN_ANGLE = 55;
const int MAX_ANGLE = 130;
const int STRAIGHT_ANGLE = 88;

const float Kp = 0;
const float Kg = 0;

void loop() {

    // Read and set sensor states
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
    engine.drive(255);

    float heading = targetAngle - newHeading;
    if (heading > 180)  heading -= 360;
    if (heading < -180) heading += 360;

    const int leftDistance = leftSensor.measureDistance();
    delay(50);
    const int rightDistance = rightSensor.measureDistance();
    int angle = Kg * angle + Kp * (leftDistance - rightDistance);
    if (abs(angle) < 5) {
      angle = 0;
    }

    const int angle_constrained = constrain(STRAIGHT_ANGLE + angle, MIN_ANGLE, MAX_ANGLE);
    myservo.write(angle_constrained);
};

