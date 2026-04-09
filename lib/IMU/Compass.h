#ifndef COMPASS_H
#define COMPASS_H

#include <Adafruit_BNO08x.h>
#include <math.h>

class Compass {
private:
    Adafruit_BNO08x bno08x;
    sh2_SensorValue_t sensorValue;
    uint8_t _addr;

public:
    Compass() : _addr(0x4A) {}

    /**
     * Initializes the BNO085. 
     * Tries 0x4A first, then 0x4B.
     */
    bool begin() {
        bool status = false;
        
        // Try default address 0x4A
        if (bno08x.begin_I2C(0x4A)) {
            _addr = 0x4A;
            status = true;
        } 
        // Try alternative address 0x4B
        else if (bno08x.begin_I2C(0x4B)) {
            _addr = 0x4B;
            status = true;
        }

        if (!status) return false;

        // Enable the Game Rotation Vector report
        // 50000us = 50ms (20Hz). This is stable for I2C sharing.
        if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 50000)) {
            return false;
        }

        return true;
    }

    /**
     * Reads the IMU and returns Yaw (0-360).
     * Returns -1.0 if no new data is available.
     */
    float getYaw() {
        // If no event is pending, return -1 immediately (non-blocking)
        if (!bno08x.getSensorEvent(&sensorValue)) {
            return -1.0;
        }

        if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
            float i = sensorValue.un.gameRotationVector.i;
            float j = sensorValue.un.gameRotationVector.j;
            float k = sensorValue.un.gameRotationVector.k;
            float r = sensorValue.un.gameRotationVector.real;

            // Quaternion to Euler Yaw calculation
            float siny_cosp = 2 * (r * k + i * j);
            float cosy_cosp = 1 - 2 * (j * j + k * k);
            float yaw = atan2(siny_cosp, cosy_cosp);

            // Convert to degrees
            float degrees = yaw * (180.0 / M_PI);

            // Normalize to 0-360
            if (degrees < 0) degrees += 360.0;
            
            return degrees;
        }
        
        return -1.0;
    }
};

#endif