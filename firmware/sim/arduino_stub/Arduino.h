// Host stand-in for the Arduino-ESP32 3.x API used by emg_orthosis.ino.
// Signatures copied from arduino-esp32 3.3 (cores/esp32/esp32-hal-*.h) so a
// type error in the sketch fails here too. Only compiles and links; the real
// ESP32 build runs in CI.
#pragma once
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>

#define ESP_ARDUINO_VERSION_MAJOR 3
#define IRAM_ATTR
#define HIGH 0x1
#define LOW 0x0
#define INPUT 0x01
#define OUTPUT 0x03
#define INPUT_PULLUP 0x05
#define CHANGE 0x03
#define F(s) (s)
#define digitalPinToInterrupt(p) (p)

typedef enum { ADC_0db, ADC_2_5db, ADC_6db, ADC_11db, ADC_ATTENDB_MAX } adc_attenuation_t;

inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline int digitalRead(uint8_t) { return 0; }
inline uint16_t analogRead(uint8_t) { return 2048; }
inline void analogReadResolution(uint8_t) {}
inline void analogSetAttenuation(adc_attenuation_t) {}
inline bool ledcAttach(uint8_t, uint32_t, uint8_t) { return true; }
inline bool ledcWrite(uint8_t, uint32_t) { return true; }
inline void attachInterrupt(uint8_t, void (*)(void), int) {}
inline unsigned long millis() { return 0; }
inline unsigned long micros() { return 0; }
inline void delay(uint32_t) {}

struct HWCDCStub {
  void begin(unsigned long) {}
  int available() { return 0; }
  int read() { return -1; }
  size_t println(const char* s) { return std::printf("%s\n", s); }
  size_t printf(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
    va_list a; va_start(a, fmt); const int n = std::vprintf(fmt, a); va_end(a); return n;
  }
};
inline HWCDCStub Serial;

typedef struct { int owner; int count; } portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {0, 0}
#define portENTER_CRITICAL(m) ((void)(m))
#define portEXIT_CRITICAL(m) ((void)(m))
#define portENTER_CRITICAL_ISR(m) ((void)(m))
#define portEXIT_CRITICAL_ISR(m) ((void)(m))

void setup();
void loop();
