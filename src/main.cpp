#include <Arduino.h>
#include <Lights.h>
#include <Lidar.h>
#include <Engine.h>
#include <Compass.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Lidar.h>
#include <vector>
#include <Config.h>
#include <Pixy2.h>

// PPD Constants
const float Kp = 0.09; // 0.1
const float Kg = 0.5;
const float Kd = 0.05; // 0.1

// Global variables
bool started = false;
float last_error = 0;
float last_time = 0;
float targetAngle = 0;
int edge = 0;
bool isClockwise = false;
bool sideLock = false;
int clockStop = 0;
bool button_state = 1;
int ticks = 0;
bool obstacleDetected = false;

// Class variables
auto engine = Engine(MOTOR_1, MOTOR_2, ENABLE_MOTOR);
Compass robotCompass;
Servo myservo;
Distance_Sensor leftSensor;
Distance_Sensor rightSensor;
Distance_Sensor frontSensor;
Pixy2 pixy;

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
  pinMode(2, OUTPUT);
  pinMode(15, OUTPUT);
  digitalWrite(5, LOW);
  digitalWrite(2, LOW);
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

  if (!rightSensor.begin(2, 0x32)) {
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

  myservo.attach(SERVO_PIN, 500, 2400); 
  myservo.write(STRAIGHT_ANGLE);
  set_light_state(16, 3);
  
  pixy.init();
  pixy.setLamp(1, 0);
  pixy.setLED(0, 0, 0);
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
      set_light_state(16, 3);
      engine.stop();
      return;
    }

    engine.drive(ENGINE_SPEED);
    pixy.ccc.getBlocks();

    float heading = targetAngle - robotCompass.getYaw();
    while (heading > 180.0)  heading -= 360.0;
    while (heading < -180.0) heading += 360.0;

    Distance_Result frontDistance = frontSensor.measureDistance();
    const Distance_Result leftDistance = leftSensor.measureDistance();
    const Distance_Result rightDistance = rightSensor.measureDistance();
    float width = leftDistance.distance + rightDistance.distance;
    width *= abs(cos(radians(heading)));

    // Sector detection
    const int currentSector = abs(targetAngle) / 90;
    int nextSector = currentSector + 1;
    if (nextSector == 4) {
      nextSector = 0;
    }
    Serial.println(String("L: ") + leftDistance.distance + "mm (" + leftDistance.status + ") R: " 
    + rightDistance.distance + "(" + rightDistance.status + ") Gyro: " + heading + " Front: " + frontDistance.distance + "mm (" + frontDistance.status + ") Sector: " + currentSector + " Width: " + width) ;
    
    // Turning && frontDistance.status == 0 && width > MAX_ARENA_WIDTH
    if (frontDistance.distance <= FRONT_DISTANCE) {
      if (!sideLock) {
        isClockwise = !(leftDistance.distance > rightDistance.distance || leftDistance.status == 4);
      }

      if (isClockwise) 
        targetAngle -= 90;
      else targetAngle += 90;
    
      if (isClockwise) {
        myservo.write(MIN_ANGLE);
      } else {
        myservo.write(MAX_ANGLE);
      }

      delay(750);

      edge++;
      sideLock = true;

      if (edge >= 12) {
        clockStop = millis();
      }

      myservo.write(STRAIGHT_ANGLE);
      delay(600);
    }

    float angle = Kg * heading;

    /*
      Camera detection
    */
    int max_y = 0; // Highest Y value means closest to the bottom of camera view
    Block block;
    int found = false;
    
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      auto _block = pixy.ccc.blocks[i];
      
      if (_block.m_y > max_y && _block.m_height < 70 && _block.m_height > 8) {
        max_y = _block.m_y;
        block = _block;
        found = true;
      }
    }

    if (OBSTACLE_ROUND && found && millis() > ticks) {
      int off = 0;

      if (block.m_signature == 1) {
        off = -105;
        set_light_state(17, 2);
        set_light_state(16, 0);
      } else {
        off = 55;
        set_light_state(16, 2);
        set_light_state(17, 0);
      }

      //Serial.println("Block: " + String(block.m_signature) + " X: " + block.m_x + " Y: " + block.m_y + " W: " + block.m_width + " H: " + block.m_height);

      int c_err = pixy.frameWidth / 2 + off - block.m_x;
      Serial.println("Error: " + String(c_err) + " Angle: " + angle);
      angle = 0.32 * c_err;
      obstacleDetected = true;
    } else {
      set_light_state(16, 2);
      set_light_state(17, 2);

      if (millis() > ticks && obstacleDetected) {
        ticks = millis() + 450;
        obstacleDetected = false;
      }
    }

    // if the robot is going counterclockwise,  we should follow the right sensor
    const Distance_Result outerDistance = isClockwise ? leftDistance : rightDistance;

    float err = last_error - heading;
    const float dist_err = outerDistance.distance - TARGET_DISTANCE;

    if ((millis() > ticks && pixy.ccc.numBlocks == 0 && OBSTACLE_ROUND) || (!OBSTACLE_ROUND && sideLock)) {
      angle += Kp * dist_err * (isClockwise ? 1 : -1);
    }

    if ((millis() - clockStop) >= 2500 && clockStop != 0) {
      engine.stop();
      ESP.restart();
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