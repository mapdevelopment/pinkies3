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
float last_time = millis();
int targetAngle = 0;
int edge = 0;
bool isClockwise = false;

const int MIN_ANGLE = 60;
const int MAX_ANGLE = 120;
const int STRAIGHT_ANGLE = 88;
const int TARGET_DISTANCE = 300;
const int TURN_DISTANCE = 400;

const float Kp = 0.09; // 0.1
const float Kg = 0.95;
const float Kd = 0.05; // 0.1

void loop() {
    BUTTON_STATE = digitalRead(BUTTON_PIN);
    blink_lights();

    float newHeading = robotCompass.getYaw();
    if (BUTTON_STATE == 0) {
      started = !started;
      targetAngle = newHeading;
      delay(400);
    } 

    if (!started) {
      set_light_state(2, 3);
      engine.stop();
      return;
    }

    set_light_state(2, 1);
    engine.drive(255);

    int heading = targetAngle - newHeading;
    if (heading > 180)  heading -= 360;
    if (heading < -180) heading += 360;

    Distance_Result frontDistance = frontSensor.measureDistance();
    Distance_Result leftDistance = leftSensor.measureDistance();
    const Distance_Result rightDistance = rightSensor.measureDistance();

    // 2 or 4 when object is too far
    
    if (frontDistance.distance <= TURN_DISTANCE) {
      isClockwise = leftDistance.distance <= 800 && leftDistance.status == 0;
      while (frontDistance.distance <= 1000 || frontDistance.distance >= 2700) {
        frontDistance = frontSensor.measureDistance();
        //Serial.println(String("left:") + leftDistance.distance + " " + frontDistance.distance + " status: " + leftDistance.status);

        if (isClockwise) {
          myservo.write(MIN_ANGLE);
        } else {
          myservo.write(MAX_ANGLE);
        }

        delay(20);
      }

      if (isClockwise) {
        targetAngle -= 90;
      } else {
        targetAngle += 90;
      }

      targetAngle %= 360;
      edge++;
      set_light_state(2, 3, edge);
    }

    float angle = Kg * heading;

    //Serial.println(String("L: ") + leftDistance.distance + "mm (" + leftDistance.status + ") T: " 
    //+ targetAngle + ") Gyro: " + heading + " Front: " + frontDistance.distance + "mm");

    const Distance_Result outerDistance = isClockwise ? leftDistance : leftDistance;

    // if the robot is going counterclockwise,  we should use the right sensor
    const int dist_err = outerDistance.distance - TARGET_DISTANCE;
    float err = heading - last_error;

    if (outerDistance.distance <= 850 && outerDistance.distance >= 100) {
      angle += Kp * dist_err;
    }

    if (last_error == 0) {
      last_time = millis();
      last_error = err;
    } else {
      err /= (millis() - last_time);
      last_time = millis();
      angle -= Kd * err;
    }

    if (abs(angle) < 3) {
      angle = 0;

      // Stop after the robot is centered in the sector
      if (edge >= 12) {
        engine.stop();
        ESP.restart();
      }
    }

    const int angle_constrained = constrain(STRAIGHT_ANGLE + round(angle), MIN_ANGLE, MAX_ANGLE);
    myservo.write(angle_constrained);
};

