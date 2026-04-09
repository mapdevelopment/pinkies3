#ifndef ENGINE_H
#define ENGINE_H

#include <Arduino.h>

class Engine {
  private:
    uint8_t pinIN1;
    uint8_t pinIN2;
    uint8_t pinPWM;
    int currentSpeed;

  public:
    // Constructor: Assigns pins to the motor instance
    Engine(uint8_t in1, uint8_t in2, uint8_t pwm) {
        pinIN1 = in1;
        pinIN2 = in2;
        pinPWM = pwm;
    }

    // Initialize the pins
    void begin() {
        pinMode(pinIN1, OUTPUT);
        pinMode(pinIN2, OUTPUT);
        pinMode(pinPWM, OUTPUT);
        stop();
    }

    /**
     * Set motor power
     * @param speed: -255 to 255 (Negative is reverse, Positive is forward)
     */
    void drive(int speed) {
        currentSpeed = constrain(speed, -255, 255);

        if (currentSpeed > 0) {
            // Forward
            digitalWrite(pinIN1, HIGH);
            digitalWrite(pinIN2, LOW);
            analogWrite(pinPWM, currentSpeed);
        } 
        else if (currentSpeed < 0) {
            // Reverse
            digitalWrite(pinIN1, LOW);
            digitalWrite(pinIN2, HIGH);
            analogWrite(pinPWM, abs(currentSpeed));
        } 
        else {
            stop();
        }
    }

    void stop() {
        digitalWrite(pinIN1, LOW);
        digitalWrite(pinIN2, LOW);
        analogWrite(pinPWM, 0);
        currentSpeed = 0;
    }
};

#endif