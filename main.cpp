#include <Arduino.h>
#include <Lights.h>
#include <Lidar.h>

int BUTTON_PIN = 13;
int BUTTON_STATE = 1;

/*
Motor PINS
*/
int ENABLE_MOTOR = 32;
int MOTOR_1 = 25;
int MOTOR_2 = 26;

void setup() {
    Serial.begin(9600);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // Motor
    pinMode(MOTOR_1, OUTPUT);
    pinMode(MOTOR_2, OUTPUT);
    pinMode(ENABLE_MOTOR, OUTPUT);

    ledcSetup(0, 30000, 8);
    ledcAttachPin(ENABLE_MOTOR, 0);

    //start_lidar_sensors();
};

void loop() {
    // Read and set sensor states
    BUTTON_STATE = digitalRead(BUTTON_PIN);
    blink_lights();
    //read_lidar_distance();
};

