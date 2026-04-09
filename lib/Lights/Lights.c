#include "Lights.h"

int ready_sensor[50] = { -1 };
int initialized[50] = { 0 };

void set_light_state(uint8_t sensor, uint8_t state) {
    if (initialized[sensor] == 0) {
        pinMode(sensor, OUTPUT);
        initialized[sensor] = 1;
    }

    ready_sensor[sensor] = state;
    
    if (state == 3) return;

    digitalWrite(sensor, state);
}

void blink_lights() {
    for (int i = 0; i < 50; i++) {
        if (ready_sensor[i] != 3) 
            continue;
        
        if ((millis() / 500) % 2 == 0) {
            digitalWrite(i, 1);
        } else {
            digitalWrite(i, 0);
        }
    }
}