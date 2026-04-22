#include <Arduino.h>
#include <Lights.h>
#include <Lidar.h>
#include <Engine.h>
#include <Compass.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Lidar.h>

#define RXD2 3
#define TXD2 1

int BUTTON_PIN = 13;
int BUTTON_STATE = 1;

/*
Motor PINS
*/
const bool OBSTICLE_ROUND = false;
const int ENABLE_MOTOR = 32;
const int MOTOR_1 = 26;
const int MOTOR_2 = 25;
const int ROBOT_WIDTH = 82;

auto engine = Engine(MOTOR_1, MOTOR_2, ENABLE_MOTOR);
Compass robotCompass;
Servo myservo;
HardwareSerial camera_serial(2);

Distance_Sensor leftSensor;
Distance_Sensor rightSensor;
Distance_Sensor frontSensor;

void setup() {
  Serial.begin(115200);

  if (OBSTICLE_ROUND) {
    camera_serial.begin(9600, SERIAL_8N1, RXD2, TXD2);
  }

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
float targetAngle = 0;
int edge = 0;
bool isClockwise = true;
bool sideLock = false;

const int MIN_ANGLE = 60;
const int MAX_ANGLE = 120;
const int STRAIGHT_ANGLE = 88;
const int TARGET_DISTANCE = 300;
const int TURN_DISTANCE = 500;

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

    float heading = targetAngle - newHeading;
    if (heading > 180)  heading -= 360;
    if (heading < -180) heading += 360;

    Distance_Result frontDistance = frontSensor.measureDistance();
    Distance_Result leftDistance = leftSensor.measureDistance();
    const Distance_Result rightDistance = rightSensor.measureDistance();
    const int width = leftDistance.distance + rightDistance.distance;

    Serial.println(String("L: ") + leftDistance.distance + "mm (" + leftDistance.status + ") R: " 
    + rightDistance.distance + "(" + rightDistance.status + ") Gyro: " + heading + " Front: " + frontDistance.distance + "mm (" + frontDistance.status + ")");


    if (frontDistance.distance <= TURN_DISTANCE && frontDistance.status == 0 && width >= 900) {
      if (!sideLock) {
        isClockwise = (leftDistance.distance <= 700 && (leftDistance.status == 0 || leftDistance.status == 2));
        sideLock = true;
      }

      while (frontDistance.distance <= 1500 || frontDistance.distance >= 2700 || frontDistance.status == 4) {
        frontDistance = frontSensor.measureDistance();
        Serial.println(String("left:") + leftDistance.distance + " " + frontDistance.distance + " status: " + leftDistance.status);

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

      targetAngle = fmod(targetAngle, 360.0);
      edge++;
      set_light_state(23, 3, edge - 1);
    }

    float angle = Kg * heading;

    const Distance_Result outerDistance = isClockwise ? leftDistance : rightDistance;

    // if the robot is going counterclockwise,  we should use the right sensor
    float dist_err = outerDistance.distance - width / 2;

    if (OBSTICLE_ROUND && camera_serial.available() > 0) {
      char data = camera_serial.read();
      Serial.println(data);
    }

    float err = heading - last_error;

    if (outerDistance.distance <= 850 && width <= 1100) {
      angle += Kp * dist_err * (isClockwise ? 1 : -1);
      if (abs(dist_err) < 50 && edge == 12 && heading <= 5) {
        delay(2000);
        engine.stop();
        ESP.restart();
      }
    }

    if (last_error == 0) {
      last_time = millis();
      last_error = err;
    } else {
      err /= (millis() - last_time);
      last_time = millis();
      angle -= Kd * err;
    }

    if (abs(angle) < 2) {
      angle = 0;

      // Stop after the robot is centered in the sector
    }

    const int angle_constrained = constrain(STRAIGHT_ANGLE + round(angle), MIN_ANGLE, MAX_ANGLE);
    myservo.write(angle_constrained);
};

