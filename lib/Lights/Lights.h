#ifndef LIGHTS_H
#define LIGHTS_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

void set_light_state(uint8_t sensor, uint8_t state);
void blink_lights();

#ifdef __cplusplus
}
#endif

#endif