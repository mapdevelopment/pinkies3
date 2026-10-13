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
  pixy.setLamp(1, 0);
  pixy.setLED(0, 0, 0);
};

enum StateManager : uint8_t {
    AVOIDANCE, TURN, FOLLOWING, PARKING_START, STAND_BY
};

StateManager state = FOLLOWING;

int targetAngle;
int started = false;
float last_error = 0.0;
unsigned long last_measurement = millis();
int firstPillar[4] = { 0 };
bool isClockwise = false;
unsigned long lastTurn = 0;
bool sideLock = false;
int edgeCount = 0;

// PID
constexpr float Kg = 0.6;
constexpr float Kp = 0.09;
constexpr float Kd = 0.05;

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
    const Distance_Result outerDistance = isClockwise ? leftDistance : rightDistance;
    const Distance_Result innerDistance = isClockwise ? rightDistance : leftDistance;
    const int timespan = millis() - lastTurn;

    if (state == TURN) {
      set_light_state(17, 1);

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

      set_light_state(17, 0);
      state = STAND_BY;
      edgeCount++;
      lastTurn = millis();

      return;
    } else if (state == FOLLOWING) {
      const int error = innerDistance.distance - TARGET_DISTANCE;
      if (abs(heading) < 25.0 && sideLock && width < MAX_ARENA_WIDTH) {
        angle += Kp * error * (isClockwise ? -1 : 1);
      }
    }

    SerialBT.println(String("w: ") + width + " H:" + heading + " L: " + leftDistance.distance + " (" + 
      leftDistance.status + ") R: " + rightDistance.distance + " (" + rightDistance.status + ") T: " + frontDistance.distance + " (" + frontDistance.status + ") isClockwise: " + isClockwise + " Heading: " + heading);

    const int d_time = millis() - last_measurement;
    const float d = d_time > 0 ? (heading - last_error) / d_time : 0; 

    angle += Kg * heading + Kd * d;

    last_error = heading;
    last_measurement = millis();

    angle = constrain(STRAIGHT_ANGLE + round(angle), MIN_ANGLE, MAX_ANGLE);
    myservo.write(angle);

    // Enables target avoidance, it will be overwritten if the robot is required to do a turn
    state = FOLLOWING;

    if (timespan < 2000) return;

    if (edgeCount == 12) {
      engine.stop();
      ESP.restart();
    }

    if (frontDistance.distance < FRONT_DISTANCE &&
      frontDistance.status != 7 &&
      (sideLock ? innerDistance.distance > MAX_ARENA_WIDTH : width > MAX_ARENA_WIDTH)
    ) {
      state = TURN;
      SerialBT.println("performs a turn");

      if (!sideLock) {
        isClockwise = (rightDistance.distance > leftDistance.distance || rightDistance.status == 7);
        sideLock = true;
      }
    }
}