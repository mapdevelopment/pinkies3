#include <Arduino.h>

using namespace std;

HardwareSerial camera_serial(2);

string readSerial() {
    string tempBuffer = "";
    string readString = "";

    while (camera_serial.available() > 0) {
      char inChar = camera_serial.read();
      if (inChar == '\n') {
        readString = tempBuffer;
        tempBuffer = "";
      } else if (inChar != '\r') {
        tempBuffer += inChar;
      }
    }

    return readString;
}