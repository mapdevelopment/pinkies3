#include "Lights.h"

int ready_sensor[50] = { -1 };
int initialized[50] = { 0 };
int blinks[50];

void set_light_state(uint8_t sensor, uint8_t state, int blink) {
    if (initialized[sensor] == 0) {
        pinMode(sensor, OUTPUT);
        initialized[sensor] = 1;
    }

    ready_sensor[sensor] = state;
    blinks[sensor] = blink;
    
    if (state == 3) return;

    digitalWrite(sensor, state);
}

void blink_lights() {
    for (int i = 0; i < 50; i++) {
        if (ready_sensor[i] != 3) 
            continue;
        
        // blinks every 300ms
        if ((millis() / 300) % 2 == 0) {
            if (blinks[i] != -1) {
                if (blinks[i] >= 0) {
                    digitalWrite(i, 1);
                    blinks[i]--;
                }
            } else {
                digitalWrite(i, 1);
            }
        } else {
            digitalWrite(i, 0);
        }
    }
}