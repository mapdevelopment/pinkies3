#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <Arduino.h>

// --- CONFIGURATION ---
const int NUM_SENSORS = 2; // Change this to add more sensors
const int lpPins[NUM_SENSORS] = {4, 5}; // Add more LP pins here (e.g., {4, 5, 6, 7})
const int baseAddress = 0x30; // Starting I2C address for reassignment
float SENSOR_DISTANCE[NUM_SENSORS];

const int resolution = 8; // 4x4 or 8x8
const int totalZones = resolution * resolution;

// --- ARRAYS FOR SENSORS AND DATA ---
SparkFun_VL53L5CX sensors[NUM_SENSORS];           // Array of sensor objects
int16_t sensorData[NUM_SENSORS][64];

void printGrid(int16_t data[]) {
  for (int y = 0; y < resolution; y++) {
    for (int x = 0; x < resolution; x++) {
      int index = (y * resolution) + x;
      
      Serial.print(data[index]);
      Serial.print("\t"); // Tab formatting
    }
    Serial.println(); // Next row
  }
  Serial.println(); 
}

void setup_lidar_sensors() {
  Serial.println("Starting multi-sensor initialization...");

  // 1. Set all LP pins as OUTPUT and pull them LOW (turn all sensors off)
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(lpPins[i], OUTPUT);
    digitalWrite(lpPins[i], LOW);
  }
  delay(100);

  // 2. Wake up and initialize sensors one by one
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print("Booting Sensor ");
    Serial.println(i + 1);

    // Wake up the specific sensor
    digitalWrite(lpPins[i], HIGH);
    delay(50); // Give it time to boot

    // Initialize the sensor
    if (sensors[i].begin() == false) {
      Serial.print("Failed to boot sensor ");
      Serial.println(i + 1);
      while (1); // Freeze if a sensor fails
    }

    // Assign a unique I2C address (e.g., 0x30, 0x32, 0x34...)
    uint8_t newAddress = baseAddress + (i * 2);
    sensors[i].setAddress(newAddress);

    // Configure and start
    sensors[i].setResolution(totalZones);
    sensors[i].setSharpenerPercent(20);
    sensors[i].startRanging();
    
    Serial.println(" - Online & Configured");
  }
  Serial.println("All sensors ready!");
  Serial.println("--------------------");
}
void printStatusGrid(uint8_t *statuses) {
    // This assumes a 4x4 grid (16 zones)
    for (int i = 0; i < resolution*resolution; i++) {
        Serial.printf("%3d ", statuses[i]); // Print the 1-byte status
        if ((i + 1) % resolution == 0) Serial.println(); // New line every 4 values
    }
    Serial.println();
}
void read_lidar_data() {
  VL53L5CX_ResultsData rawData;

  if (sensors[0].isDataReady()) {
    if (sensors[0].getRangingData(&rawData)) {
      SENSOR_DISTANCE[0] = (rawData.distance_mm[28] + rawData.distance_mm[36]) / 2.f;
      if (rawData.target_status[28] == 255 || rawData.target_status[36] == 255) {
        SENSOR_DISTANCE[0] = -1;
      }
    }
  }

  if (sensors[1].isDataReady()) {
    if (sensors[1].getRangingData(&rawData)) {
      SENSOR_DISTANCE[1] = (rawData.distance_mm[27] + rawData.distance_mm[35]) / 2.f;
      if (rawData.target_status[27] == 255 || rawData.target_status[35] == 255) {
        SENSOR_DISTANCE[1] = -1;
      }
    }
  }

  delay(50);
}