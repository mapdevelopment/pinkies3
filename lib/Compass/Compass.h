#ifndef COMPASS_H
#define COMPASS_H

#include <Adafruit_BNO08x.h>
#include <math.h>

class Compass {
private:
    Adafruit_BNO08x bno08x;
    sh2_SensorValue_t sensorValue;
    int8_t _resetPin;
    float currentYaw; // <-- Added to cache the last good reading

public:
    Compass(int8_t resetPin = 32) : _resetPin(resetPin), currentYaw(0.0) {}

    bool begin() {
        if (_resetPin != -1) {
            pinMode(_resetPin, OUTPUT);
            digitalWrite(_resetPin, LOW);
            delay(50); 
            digitalWrite(_resetPin, HIGH);
            delay(200); 
        }

        if (!bno08x.begin_I2C(0x4A) && !bno08x.begin_I2C(0x4B)) {
            return false;
        }

        // 50000 microseconds = 50ms interval (20Hz)
        if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 50000)) {
            return false;
        }

        return true;
    }

    float getYaw() {
        // Pump the BNO08x queue to process all pending events
        while (bno08x.getSensorEvent(&sensorValue)) {
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
                
                currentYaw = degrees; // Save the newly computed orientation
            }
        }
        
        // Always return the last known good configuration state
        return currentYaw; 
    }
};

#endif
