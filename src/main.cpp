#include <Arduino.h>
#include <Lights.h>
#include <Lidar.h>
#include <Engine.h>
#include <Compass.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Sorting.h>

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

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(9600);

  set_light_state(2, 0);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Motor
  pinMode(MOTOR_1, OUTPUT);
  pinMode(MOTOR_2, OUTPUT);
  pinMode(ENABLE_MOTOR, OUTPUT);

  ledcSetup(0, 30000, 8);
  ledcAttachPin(ENABLE_MOTOR, 0);

  setup_lidar_sensors();
  engine.begin();

  // Initialize Compass
  if (!robotCompass.begin()) {
      Serial.println("BNO085 Fail!");
  }

  myservo.attach(33); 
  set_light_state(2, 3);
};

float started = false;
float last_error = 0;
float last_time = millis();
const size_t buffer_size = 30;
float track_buffer[buffer_size] = { 0 };
float targetAngle = 0;
const float STRAIGHT_ANGLE = 80.0;
const float Kp = 0.02;
const float Kd = 0;
int track_tracker = 0;

void loop() {

    // Read and set sensor states
    BUTTON_STATE = digitalRead(BUTTON_PIN);
    blink_lights();
    read_lidar_data();

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

    float angle = targetAngle - newHeading;
    if (angle > 180)  angle -= 360;
    if (angle < -180) angle += 360;

    float rad_angle = radians(angle);
    float width = (SENSOR_DISTANCE[0] + SENSOR_DISTANCE[1]) * cos(rad_angle);
        if (track_tracker == (buffer_size - 1)) {
        track_tracker = 0;
    } else {
        track_tracker++;
    }

    track_buffer[track_tracker] = width;
    const float track = get_dominant_cluster_average(
        buffer_size, track_buffer, 20
    );
    float distance = SENSOR_DISTANCE[0] * cos(rad_angle);

    float error = (track) / 2 - distance; 
    if (!last_error) {
        last_error = error;
    }

    unsigned long now = millis();
    float delta_t = (now - last_time) / 1000.0f;
    last_time = now;


    const float derivative_delta = (error - last_error) / delta_t;
    last_error = error;

    int turning_angle = STRAIGHT_ANGLE 
        + Kp * error
        + Kd * derivative_delta;

    //Serial.printf("Left: %.2f | Right: %.2f | Width: %.2f\n", SENSOR_DISTANCE[1], SENSOR_DISTANCE[0], width);

    float final_servo_angle = constrain(turning_angle, STRAIGHT_ANGLE - 45, STRAIGHT_ANGLE + 45);
    Serial.printf("Proportional: %.2f | Error: %.2f | Derivative: %.2f | Angle: %.2f | Width: %.2f \n", Kp * error, error, Kd * derivative_delta, final_servo_angle, track);

    myservo.write(final_servo_angle);
};

