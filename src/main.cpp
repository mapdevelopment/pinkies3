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
int ticks = 0;
int currently_tracking = -1;

// Class variables
auto engine = Engine(MOTOR_1, MOTOR_2, ENABLE_MOTOR);
Compass robotCompass;
Servo myservo;
Distance_Sensor leftSensor;
Distance_Sensor rightSensor;
Distance_Sensor frontSensor;
Pixy2 pixy;

int get_distance(int sector, bool turn = false) {
  if (!OBSTACLE_ROUND) {
    if (sectorWidth[sector] == 0)
      return TARGET_DISTANCE;
    else return sectorWidth[sector] * 2/3;
  }

  return TARGET_DISTANCE;
}

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

    engine.drive(150);
    pixy.ccc.getBlocks();

    float heading = targetAngle - robotCompass.getYaw();
    if (heading > 180)  heading -= 360;
    if (heading < -180) heading += 360;

    Distance_Result frontDistance = frontSensor.measureDistance();
    const Distance_Result leftDistance = leftSensor.measureDistance();
    const Distance_Result rightDistance = rightSensor.measureDistance();
    const int width = leftDistance.distance + rightDistance.distance;

    // Sector detection
    const int currentSector = abs(targetAngle) / 90;
    int nextSector = currentSector + 1;
    if (nextSector == 4) {
      nextSector = 0;
    }

    // Turning
    if (frontDistance.distance <= (get_distance(nextSector, true) + ROBOT_WIDTH)
      && frontDistance.status == 0 && width >= 1000 && !OBSTACLE_ROUND) {
      if (!sideLock) {

        // can be done using camera if orange line is detected first then we are going clockwise
        isClockwise = (leftDistance.distance <= 700 && (leftDistance.status == 0 || leftDistance.status == 2))
          || rightDistance.status == 4
          || (rightDistance.distance >= leftDistance.distance && rightDistance.status == 2 && leftDistance.status == 2);

        sideLock = true;
      }

      if (!OBSTACLE_ROUND && measurementCount > 0) {
        const int w = cumulativeWidth / measurementCount + ROBOT_WIDTH;
        // dynamic, but for competition it would be better to have fixed widths for the sectors (600 or 1000 mm)
        /*
        sectorWidth[currentSector] = (
          abs(MAX_ARENA_WIDTH - w) > abs(MIN_ARENA_WIDTH - w)) ? MIN_ARENA_WIDTH : MAX_ARENA_WIDTH
        ;
        */
        
        sectorWidth[currentSector] = constrain(
          w,
          MIN_ARENA_WIDTH, 
          MAX_ARENA_WIDTH) - ROBOT_WIDTH
        ;
      }

      if (isClockwise) 
        targetAngle -= 90;
      else targetAngle += 90;
      
      targetAngle = fmod(targetAngle, 360.0);
      while (abs(heading) >= 2) {
        heading = targetAngle - robotCompass.getYaw();

        if (isClockwise) {
          myservo.write(MAX_ANGLE);
        } else {
          myservo.write(MIN_ANGLE);
        }

        delay(20);
      }

      edge++;
      if (edge >= 12) {
        clockStop = millis();
      }

      cumulativeWidth = 0;
      measurementCount = 0;
    }

    float angle = Kg * heading;
    Serial.println(String("L: ") + leftDistance.distance + "mm (" + leftDistance.status + ") R: " 
    + rightDistance.distance + "(" + rightDistance.status + ") Gyro: " + heading + " Front: " + frontDistance.distance + "mm (" + frontDistance.status + ") Sector: " + currentSector + " Width: " + width) ;
    
    /*
      Camera detection
    */
    bool obstacleDetected = false;
    if (OBSTACLE_ROUND && pixy.ccc.numBlocks > 0 && millis() > ticks) {
      int max_area = 0;
      Block block;
      for (int i = 0; i < pixy.ccc.numBlocks; i++) {
        long area = (long)pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
        int index = pixy.ccc.blocks[i].m_index;

        if (area > max_area || index == currently_tracking) {
          max_area = area;
          block = pixy.ccc.blocks[i];
          if (index == currently_tracking) {
            break;
          }
        }
      }

      int off = 0;
      if (currently_tracking != block.m_index) {
        currently_tracking = block.m_index;
        ticks = millis() + 100;
        return;
      }

      if (block.m_signature == 1) {
        off = -90;
        set_light_state(17, 1);
      } else {
        off = 90;
        set_light_state(16, 1);
      }

      Serial.println("Block: " + String(block.m_signature) + " X: " + block.m_x + " Y: " + block.m_y + " W: " + block.m_width + " H: " + block.m_height);

      int c_err = pixy.frameWidth / 2 + off * (pixy.frameHeight / block.m_height) - block.m_x;
      Serial.println("Error: " + String(c_err) + " Angle: " + angle);
      if (block.m_height > 50 && block.m_height < 140)
        angle = 0.08 * c_err;
    } else {
      set_light_state(16, 0);
      set_light_state(17, 0);

      if (currently_tracking != -1 && pixy.ccc.numBlocks == 0) {
        currently_tracking = -1;
        ticks = millis() + 100;
      }

      if (ticks > millis())
        angle = 0;
    }

    // if the robot is going counterclockwise,  we should follow the right sensor
    const Distance_Result outerDistance = isClockwise ? leftDistance : rightDistance;

    float err = last_error - heading;
    const float dist_err = outerDistance.distance - get_distance(currentSector);

    /*if (outerDistance.distance <= 850 && width <= 1100 && 
      (sectorWidth[currentSector] == 0 || abs(sectorWidth[currentSector] - width) <= WIDTH_THRESHOLD) // in case one of the sensors disconnects, prevents turning 
    ) {
      angle += Kp * dist_err * (isClockwise ? 1 : -1);

      // vehicle learns the width's of the sectors
      if (!OBSTACLE_ROUND && heading <= 3) {
        cumulativeWidth += width;
        measurementCount++;
      }
    }*/

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