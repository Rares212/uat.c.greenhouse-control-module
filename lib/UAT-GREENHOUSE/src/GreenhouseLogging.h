#include <Arduino.h>
#include <WebSerial.h>

class GreenhouseLogging {
    public:
        size_t print(const __FlashStringHelper *str) {
            WebSerial.print(str);
            return Serial.print(str);
        }
        size_t print(const String &str) {
            WebSerial.print(str);
            return Serial.print(str);
        }
        size_t print(const const char str[]) {
            WebSerial.print(str);
            return Serial.print(str);
        }
        size_t print(const char c) {
            WebSerial.print(c);
            return Serial.print(c);
        }

};