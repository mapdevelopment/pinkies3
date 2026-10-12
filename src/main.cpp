#include <Arduino.h>
#include <ESP32Servo.h>
#include <Compass.h>
#include <Engine.h>
#include <Lights.h>
#include <Config.h>
#include <Pixy2.h>
#include <Wire.h>
#include <Lidar.h>
#include <BluetoothSerial.h>

auto engine = Engine(MOTOR_1, MOTOR_2, ENABLE_MOTOR);
Compass robotCompass;
Servo myservo;
Distance_Sensor leftSensor;
Distance_Sensor rightSensor;
Distance_Sensor frontSensor;
BluetoothSerial SerialBT;
Pixy2 pixy;

void setup() {
  Serial.begin(115200);

  // Bluetooth Serial
  SerialBT.begin("ESP32_PlatformIO_Monitor"); 

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
    SerialBT.println("Front ToF failed!");
    while(1);
  }

  delay(50);
  
  if (!leftSensor.begin(5, 0x31)) {
    SerialBT.println("Left ToF failed!");
    while(1);
  }

  delay(50);

  if (!rightSensor.begin(2, 0x32)) {
    SerialBT.println("Right ToF failed!");
    while(1);
  }

  if (!robotCompass.begin()) {
    SerialBT.println("Compass failed!");
    while(1);
  }

  ledcSetup(0, 30000, 8);
  ledcAttachPin(ENABLE_MOTOR, 0);

  engine.begin();

  myservo.attach(SERVO_PIN, 500, 2400); 
  myservo.write(STRAIGHT_ANGLE);
  set_light_state(16, 3);
  
  pixy.init();
  pixy.setLamp(0, 0);
  pixy.setLED(255, 192, 203);
};

enum StateManager : uint8_t {
    AVOIDANCE, TURN, FOLLOWING, PARKING_START, STAND_BY
};

StateManager state = OBSTACLE_ROUND ? PARKING_START : FOLLOWING;

int targetAngle;
int started = false;
float last_error = 0.0;
unsigned long last_measurement = millis();
int firstPillar[4] = { 0 };
bool isClockwise = false;
unsigned long lastTurn = 0;
bool sideLock = false;
int edgeCount = 0;
bool afterTurn = false;
signed long trackingDelay = 0;

// PID
constexpr float Kg = 0.6;
constexpr float Kp = 0.09;
constexpr float Kd = 0.05;

Block camera_find_pillar() {
  Block last_block; 
  float high_y = 0.0; 

  int total_blocks = pixy.ccc.getBlocks();

  for (int i = 0; i < total_blocks; i++) {
    auto current_block = pixy.ccc.blocks[i];
    if (current_block.m_signature != 1 && current_block.m_signature != 2) 
      continue;

    if (current_block.m_y > high_y && current_block.m_height > 45 && current_block.m_height < 110
      && current_block.m_width > 35) {
      high_y = current_block.m_y;
      last_block = current_block;
    }
  }

  if (high_y != 0.0) {
    return last_block;
  }

  return Block();
}

void loop() {
    const int button_state = digitalRead(BUTTON_PIN);

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
    };

    engine.drive(ENGINE_SPEED);

    float angle = 0;
    float heading = targetAngle - robotCompass.getYaw();

    while (heading > 180.0)  heading -= 360.0;
    while (heading < -180.0) heading += 360.0;
  
    Distance_Result frontDistance = frontSensor.measureDistance();
    const Distance_Result leftDistance = leftSensor.measureDistance();
    const Distance_Result rightDistance = rightSensor.measureDistance();

    const int width = (leftDistance.distance + rightDistance.distance) * abs(cos(radians(heading)));
    const Distance_Result innerWall = isClockwise ? leftDistance : rightDistance;
    const Distance_Result outerWall = isClockwise ? rightDistance : leftDistance;
    const Block target = camera_find_pillar();
    const int timespan = millis() - lastTurn;
    const int trackDelay = millis() - trackingDelay;
    bool avoiding = false;
    bool canTurn = false; // (sideLock ? outerWall.distance > MAX_ARENA_WIDTH : width > MAX_ARENA_WIDTH);

    if (state == AVOIDANCE && target.m_width > 0) {
      const int camera_err = target.m_signature == 1 ? target.m_x : (target.m_x - pixy.frameWidth) ;

      if (!canTurn || afterTurn) {
        angle += camera_err * 0.17;
        avoiding = true;
        trackingDelay = millis();
        SerialBT.println(String("Err: ") + camera_err + " X: " + target.m_x + "H: " + target.m_height + " W: " + target.m_width + " Signature: " + target.m_signature + " Angle: " + angle + " avoiding obstacles ");
        
        if (target.m_signature == 1)
          set_light_state(16, 1);
        else set_light_state(17, 1);
      }
    } else if (state == PARKING_START) {
      SerialBT.println("parking");
    } else if (state == TURN) {
      if (isClockwise) 
        targetAngle -= 90;
      else targetAngle += 90;

      engine.drive(ENGINE_SPEED);

      if (isClockwise) {
        myservo.write(MIN_ANGLE);
      } else {
        myservo.write(MAX_ANGLE);
      }

      delay(TURN_DELAY);

      if (OBSTACLE_ROUND) {
        myservo.write(STRAIGHT_ANGLE);
        engine.drive(-ENGINE_SPEED);
        delay(1000);
      }
      
      state = STAND_BY;
      afterTurn = true;
      edgeCount++;
      lastTurn = millis();

      return;
    } else if (state == FOLLOWING) {
      const int error = innerWall.distance - TARGET_DISTANCE;
      if (abs(heading) < 25.0 && sideLock) {
        angle += Kp * error * (isClockwise ? 1 : -1);

        SerialBT.println(String("w: ") + width + " H:" + heading + " L: " + leftDistance.distance + " (" + 
          leftDistance.status + ") R: " + rightDistance.distance + " (" + rightDistance.status + ") T: " + frontDistance.distance + " (" + frontDistance.status + ") isClockwise: " + isClockwise + " Heading: " + heading);
      }

     set_light_state(17, 0);
     set_light_state(16, 0);
    }
  
    const int d_time = millis() - last_measurement;
    const float d = d_time > 0 ? (heading - last_error) / d_time : 0; 

    if (!avoiding)
      angle += Kg * heading + Kd * d;

    last_error = heading;
    last_measurement = millis();

    angle = constrain(STRAIGHT_ANGLE + round(angle), MIN_ANGLE, MAX_ANGLE);
    myservo.write(angle);

    // Enables target avoidance, it will be overwritten if the robot is required to do a turn
    if (target.m_height > 0 && OBSTACLE_ROUND)
      state = AVOIDANCE;
    else if (trackDelay > 800) {
      afterTurn = false;

      if(state != FOLLOWING)
        SerialBT.println("sWITCHING STATE TO FOLLOWING");

      state = FOLLOWING;
    } else {
        state = STAND_BY;
    }

    if (timespan < 3000) return;

    if (edgeCount == 12) {
      engine.stop();
      ESP.restart();
    }

    if (frontDistance.distance < FRONT_DISTANCE &&
      frontDistance.status != 7 &&
      canTurn &&
      (target.m_width == 0 || !OBSTACLE_ROUND) &&
      abs(heading) < 20
    ) {
      state = TURN;
      SerialBT.println("performs a turn");

      if (!sideLock) {
        isClockwise = (rightDistance.distance > leftDistance.distance || rightDistance.status == 7);
        sideLock = true;
      }
    }
}