#ifndef CONFIG_H
#define CONFIG_H

constexpr bool OBSTACLE_ROUND = true;
constexpr int MAX_ARENA_WIDTH = OBSTACLE_ROUND ? 1200 : 950;
constexpr int BUTTON_PIN = 12;
constexpr int ENABLE_MOTOR = 27;
constexpr int MOTOR_1 = 25;
constexpr int MOTOR_2 = 26;
constexpr int SERVO_PIN = 33;
constexpr int MIN_ANGLE = 32;
constexpr int MAX_ANGLE = 123;
constexpr int STRAIGHT_ANGLE = 78;
constexpr int TARGET_DISTANCE = OBSTACLE_ROUND ? 500 : 250;
constexpr int FRONT_DISTANCE = OBSTACLE_ROUND ? 800 : 1000;
constexpr int ENGINE_SPEED = OBSTACLE_ROUND ? 130 : 150;
constexpr int TURN_DELAY = 720;
#endif
