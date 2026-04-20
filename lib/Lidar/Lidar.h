#ifndef LIDAR_H
#define LIDAR_H

#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>

class Distance_Sensor {
    private:
        int8_t _xshut;
        VL53L4CD _sensor;

    public:
        Distance_Sensor() : _sensor(&Wire, -1) {}

        // Pass XSHUT pin AND a unique I2C address (e.g., 0x30, 0x31...)
        bool begin(int8_t xshut, uint8_t newAddress) {
            _xshut = xshut;
            
            pinMode(_xshut, OUTPUT);
            digitalWrite(_xshut, LOW); // Keep sensor in reset
            delay(10);
            digitalWrite(_xshut, HIGH); // Wake up this specific sensor
            delay(10);

            // Initialize at default 0x29
            if (_sensor.begin() != 0) return false;

            // Immediately change to the unique address
            if (_sensor.VL53L4CD_SetI2CAddress(newAddress) != 0) return false;

            _sensor.VL53L4CD_StartRanging();
            return true;
        }

        int measureDistance() {
            uint8_t dataReady;
            VL53L4CD_Result_t results;
            _sensor.VL53L4CD_CheckForDataReady(&dataReady);

            if (dataReady) {
                _sensor.VL53L4CD_GetResult(&results);
                _sensor.VL53L4CD_ClearInterrupt();
                return (results.range_status == 0) ? results.distance_mm : -1;
            }
            return -1;
        }
};
#endif
