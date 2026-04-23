#include <Arduino.h>
#include <Lights.h>
#include <Lidar.h>
#include <Engine.h>
#include <Compass.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Lidar.h>

#define RXD2 16
#define TXD2 17

// Constants
const int BUTTON_PIN = 14;
const bool OBSTACLE_ROUND = false;
const int ENABLE_MOTOR = 32;
const int MOTOR_1 = 26;
const int MOTOR_2 = 25;
const int ROBOT_WIDTH = 150;
const int MIN_ANGLE = 60;
const int MAX_ANGLE = 120;
const int STRAIGHT_ANGLE = 88;
const int TARGET_DISTANCE = 300;
const int WIDTH_THRESHOLD = 150;

// PPD Constants
const float Kp = 0.09; // 0.1
const float Kg = 0.95;
const float Kd = 0.05; // 0.1

// Global variables
bool started = false;
float last_error = 0;
float last_time = 0;
float targetAngle = 0;
int edge = 0;
bool isClockwise = true;
bool sideLock = false;
int sectorWidth[4] = { 0 };
int cumulativeWidth = 0;
int measurementCount = 0;
int clockStop = 0;
bool button_state = 1;
auto engine = Engine(MOTOR_1, MOTOR_2, ENABLE_MOTOR);

// Class variables
Compass robotCompass;
Servo myservo;
HardwareSerial camera_serial(2);
Distance_Sensor leftSensor;
Distance_Sensor rightSensor;
Distance_Sensor frontSensor;

void setup() {
  Serial.begin(115200);

  if (OBSTACLE_ROUND) {
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

void loop() {
    button_state = digitalRead(BUTTON_PIN);
    blink_lights();

    if (button_state == 0) {
      started = !started;
      delay(400);
      targetAngle = robotCompass.getYaw();
    } 

    if (!started) {
      set_light_state(2, 3);
      engine.stop();
      return;
    }

    set_light_state(2, 1);
    engine.drive(255);

    float heading = targetAngle - robotCompass.getYaw();
    if (heading > 180)  heading -= 360;
    if (heading < -180) heading += 360;

    Distance_Result frontDistance = frontSensor.measureDistance();
    const Distance_Result leftDistance = leftSensor.measureDistance();
    const Distance_Result rightDistance = rightSensor.measureDistance();
    const int width = leftDistance.distance + rightDistance.distance;
    const int currentSector = abs(targetAngle) / 90;
    int nextSector = currentSector + 1;
    if (nextSector == 4) {
      nextSector = 0;
    }

    Serial.println(String("L: ") + leftDistance.distance + "mm (" + leftDistance.status + ") R: " 
    + rightDistance.distance + "(" + rightDistance.status + ") Gyro: " + heading + " Front: " + frontDistance.distance + "mm (" + frontDistance.status + ") Sector: " + currentSector + " Width: " + sectorWidth[currentSector] + " Next: " + nextSector);

    if (frontDistance.distance <= (ROBOT_WIDTH + (sectorWidth[nextSector] == 0 ? TARGET_DISTANCE : (sectorWidth[nextSector] / 2)))
      && frontDistance.status == 0 && width >= 900) {
      if (!sideLock) {
        isClockwise = (leftDistance.distance <= 700 && (leftDistance.status == 0 || leftDistance.status == 2));
        sideLock = true;
      }

      if (!OBSTACLE_ROUND && measurementCount > 0) {
        sectorWidth[currentSector] = constrain(cumulativeWidth / (measurementCount), 500, 900);
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
      cumulativeWidth = 0;
      measurementCount = 0;
    }

    float angle = Kg * heading;

    // if the robot is going counterclockwise,  we should follow the right sensor
    const Distance_Result outerDistance = isClockwise ? leftDistance : rightDistance;
    const int T = sectorWidth[currentSector] == 0 ? TARGET_DISTANCE : sectorWidth[currentSector] / 2;
    const float dist_err = outerDistance.distance - T;

    if (OBSTACLE_ROUND && camera_serial.available() > 0) {
      char data = camera_serial.read();
      Serial.println(data);
    }

    float err = last_error - heading;

    if (outerDistance.distance <= 850 && width <= 1100 && 
      (sectorWidth[currentSector] == 0 || abs(sectorWidth[currentSector] - width) <= WIDTH_THRESHOLD) // in case one of the sensors disconnects, prevents turning 
    ) {
      angle += Kp * dist_err * (isClockwise ? 1 : -1);
      
      // stopping
      if (abs(dist_err) < 50 && edge >= 12 && heading <= 5) {
        if (clockStop == 0) {
          clockStop = millis();
        } else if ((millis() - clockStop) >= 2000) {
          engine.stop();
          ESP.restart();
        }
      }

      // vehicle learns the width's of the sectors
      if (!OBSTACLE_ROUND && heading <= 3) {
        cumulativeWidth += width;
        measurementCount++;
      }
    }

    // Calculates derivative term
    if (last_error == 0) {
      last_time = millis();
      last_error = err;
    } else {
      err /= (millis() - last_time);
      last_time = millis();
      angle += Kd * err;
    }

    // prevents jittery
    if (abs(angle) < 1) 
      angle = 0;

    const int angle_constrained = constrain(STRAIGHT_ANGLE + round(angle), MIN_ANGLE, MAX_ANGLE);
    myservo.write(angle_constrained);
};