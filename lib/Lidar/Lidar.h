#ifndef LIDAR_H
#define LIDAR_H

#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <VL53L1X.h>

struct Distance_Result {
    int distance;
    uint8_t status;
};

class Distance_Sensor {
    private:
        int8_t _xshut;
        VL53L4CD _sensor1;
        VL53L1X _sensor2;

    public:
        // -1 tells the driver we handle XSHUT manually
        Distance_Sensor() : _sensor1(&Wire, -1) {}

        bool begin(int8_t xshut, uint8_t newAddress) {
            _xshut = xshut;
            delay(50);

            digitalWrite(_xshut, HIGH);
            if (_xshut == 15) {
                if (!_sensor2.init()) {
                    return false;
                }

                _sensor2.setAddress(newAddress);
                _sensor2.startContinuous(50);
            } else {
                if (_sensor1.InitSensor(newAddress) != 0) {
                    return false;
                }

               _sensor1.VL53L4CD_SetRangeTiming(50, 0);
               _sensor1.VL53L4CD_StartRanging();
            }

            return true;
        }

        Distance_Result measureDistance() {
            Distance_Result result = Distance_Result();

            if (_xshut == 15) {
                const int measurement = _sensor2.readRangeContinuousMillimeters();
                if (!_sensor2.timeoutOccurred()) {
                    result.distance = measurement;
                }
            } else {
                uint8_t NewDataReady = 0;
                VL53L4CD_Result_t results;

                if (_sensor1.VL53L4CD_CheckForDataReady(&NewDataReady) == 0) {
                    _sensor1.VL53L4CD_ClearInterrupt();
                    _sensor1.VL53L4CD_GetResult(&results);

                    result.distance = results.distance_mm;
                    result.status = results.range_status;
                }
            }

            return result;
        }
};
#endif
