#ifndef LIDAR_H
#define LIDAR_H

#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <VL53L1X.h>
#include <RunningAverage.h> // Ensure this library is in your lib_deps

struct Distance_Result {
    int distance;
    uint8_t status;
};

class Distance_Sensor {
    private:
        int8_t _xshut;
        VL53L4CD _sensor1;
        VL53L1X _sensor2;
        RunningAverage _filter; // Using the library object

    public:
        // Initialize with a window size of 10 samples
        Distance_Sensor() : _sensor1(&Wire, -1), _filter(3) {}

        bool begin(int8_t xshut, uint8_t newAddress) {
            _xshut = xshut;
            delay(50);
            digitalWrite(_xshut, HIGH);

            if (_xshut == 15) {
                if (!_sensor2.init()) return false;
                _sensor2.setAddress(newAddress);
                _sensor2.setDistanceMode(VL53L1X::Long);
                _sensor2.setMeasurementTimingBudget(50000);
                _sensor2.startContinuous(50);
            } else {
                if (_sensor1.InitSensor(newAddress) != 0) return false;
                _sensor1.VL53L4CD_SetRangeTiming(50, 0);
                _sensor1.VL53L4CD_StartRanging();
            }
            return true;
        }

        Distance_Result measureDistance() {
            Distance_Result result = {0, 0};
            int raw_measurement = -1;

            // 1. Get raw reading from the specific sensor
            if (_xshut == 15) {
                int measurement = _sensor2.read();
                if (!_sensor2.timeoutOccurred()) {
                    raw_measurement = measurement;
                    result.status = _sensor2.ranging_data.range_status;
                }
            } else {
                uint8_t NewDataReady = 0;
                VL53L4CD_Result_t results;
                if (_sensor1.VL53L4CD_CheckForDataReady(&NewDataReady) == 0) {
                    _sensor1.VL53L4CD_ClearInterrupt();
                    _sensor1.VL53L4CD_GetResult(&results);
                    raw_measurement = results.distance_mm;
                    result.status = results.range_status;
                }
            }

            // 2. Add raw data to filter (only if a valid measurement exists)
            if (raw_measurement >= 0) {
                _filter.addValue(raw_measurement);
            }

            // 3. Return the smoothed average
            result.distance = static_cast<int>(_filter.getAverage());

            return result;
        }
};
#endif