#ifndef COMPASS_H
#define COMPASS_H

#include <Adafruit_BNO08x.h>
#include <math.h>

class Compass {
private:
    Adafruit_BNO08x bno08x;
    sh2_SensorValue_t sensorValue;
    int8_t _resetPin;

public:
    // Pass -1 if you don't have a reset pin wired, but it's highly recommended!
    Compass(int8_t resetPin = 27) : _resetPin(resetPin) {}

    bool begin() {
        // 1. HARDWARE RESET (If pin is provided)
        if (_resetPin != -1) {
            pinMode(_resetPin, OUTPUT);
            digitalWrite(_resetPin, LOW);
            delay(50); 
            digitalWrite(_resetPin, HIGH);
            delay(200); // Important: BNO085 takes time to boot its internal ARM core
        }

        // 2. INITIALIZE I2C
        // Note: The Adafruit library handles Wire.begin() internally if not already called
        if (!bno08x.begin_I2C(0x4A) && !bno08x.begin_I2C(0x4B)) {
            return false;
        }

        // 3. ENABLE REPORTS
        if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 50000)) {
            return false;
        }

        return true;
    }

    float getYaw() {
        if (!bno08x.getSensorEvent(&sensorValue)) return -1.0;

        if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
            float i = sensorValue.un.gameRotationVector.i;
            float j = sensorValue.un.gameRotationVector.j;
            float k = sensorValue.un.gameRotationVector.k;
            float r = sensorValue.un.gameRotationVector.real;

            float siny_cosp = 2 * (r * k + i * j);
            float cosy_cosp = 1 - 2 * (j * j + k * k);
            float yaw = atan2(siny_cosp, cosy_cosp);

            float degrees = yaw * (180.0 / M_PI);
            if (degrees < 0) degrees += 360.0;
            return degrees;
        }
        return -1.0;
    }
};

#endif