#ifndef CONFIG_H
#define CONFIG_H

constexpr int MIN_ARENA_WIDTH = 600;
constexpr int MAX_ARENA_WIDTH = 850;
constexpr int BUTTON_PIN = 12;
constexpr bool OBSTACLE_ROUND = false;
constexpr int ENABLE_MOTOR = 27;
constexpr int MOTOR_1 = 25;
constexpr int MOTOR_2 = 26;
constexpr int SERVO_PIN = 33;
constexpr int MIN_ANGLE = 45;
constexpr int MAX_ANGLE = 125;
constexpr int STRAIGHT_ANGLE = 85;
constexpr int TARGET_DISTANCE = OBSTACLE_ROUND ? 500 : 250;
constexpr int FRONT_DISTANCE = 600;
constexpr int ENGINE_SPEED = OBSTACLE_ROUND ? 140 : 140;

#endif
